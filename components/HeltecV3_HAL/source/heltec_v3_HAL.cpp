#include "sx126x_hal.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char* TAG = "HELTEC_SX126X_HAL";

/* Heltec WiFi LoRa 32 V3.1 pin definitions based on the schematic (see header file) */
#define SX126X_SPI_HOST     SPI2_HOST
#define SX126X_MOSI_PIN     10
#define SX126X_MISO_PIN     11
#define SX126X_SCK_PIN      9
#define SX126X_NSS_PIN      8
#define SX126X_RST_PIN      12
#define SX126X_DIO1_PIN     14
#define SX126X_BUSY_PIN     13

/* SPI configuration */
#define SX126X_SPI_CLOCK_HZ 1000000  // 1 MHz

static spi_device_handle_t spi_handle = NULL;
static bool hal_initialized = false;

/**
 * @brief Initialize SPI and GPIO for SX126x communication
 */
static esp_err_t sx126x_hal_init(void)
{
    if (hal_initialized) 
    {
        return ESP_OK;
    }

    esp_err_t ret;

    /* #01 - Define the configuration for the SPI bus and apply it */
    spi_bus_config_t buscfg = 
    {
        .mosi_io_num = SX126X_MOSI_PIN,
        .miso_io_num = SX126X_MISO_PIN,
        .sclk_io_num = SX126X_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };

    ret = spi_bus_initialize(SX126X_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) 
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* #02 - Define the configuration for the SPI device and apply it */
    spi_device_interface_config_t devcfg = 
    {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                      // SPI mode 0
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SX126X_SPI_CLOCK_HZ,
        .input_delay_ns = 0,
        .spics_io_num = SX126X_NSS_PIN,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(SX126X_SPI_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    /* #03 - Define the configuration of the GPIO and apply it */
    gpio_config_t io_conf = 
    {
        .pin_bit_mask = (1ULL << SX126X_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SX126X_BUSY_PIN); // Configure BUSY pin as input
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << SX126X_DIO1_PIN); // Configure DIO1 pin as input (for interrupts)
    gpio_config(&io_conf);

    hal_initialized = true;
    ESP_LOGI(TAG, "SX126x HAL initialized successfully");

    return ESP_OK;
}

/**
 * @brief Wait for BUSY pin to go low
 */
static void sx126x_hal_wait_on_busy(void)
{
    while (gpio_get_level((gpio_num_t)SX126X_BUSY_PIN)) 
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * Radio data transfer - write
 */
sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length)
{
    (void)context; // Unused parameter (TODO - find out if we need this)
    esp_err_t ret;
    spi_transaction_t trans = {0};

    /* #01 - Make sure the HAL is initialized and not busy */
    if (sx126x_hal_init() != ESP_OK) 
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_wait_on_busy();

    /* #02 - Verify command and data lengths place them in the a tx buffer */
    uint16_t total_length = command_length + data_length;
    if (total_length > 256) 
    {
        ESP_LOGE(TAG, "Transfer too large: %d bytes", total_length);
        return SX126X_HAL_STATUS_ERROR;
    }

    uint8_t tx_buffer[256];
    memcpy(tx_buffer, command, command_length);
    if (data_length > 0 && data != NULL) 
    {
        memcpy(tx_buffer + command_length, data, data_length);
    }

    trans.length = total_length * 8; // Length in bits
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = NULL;

    /* #03 - Send the prepared buffer with the command and data over SPI */
    ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
        return SX126X_HAL_STATUS_ERROR;
    }

    return SX126X_HAL_STATUS_OK;
}

/**
 * Radio data transfer - read
 */
sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length, uint8_t* data, const uint16_t data_length)
{
    (void)context; // Unused parameter (TODO - find out if we need this)
    esp_err_t ret;
    spi_transaction_t trans = {0};

    /* #01 - Make sure the HAL is initialized and not busy */
    if (sx126x_hal_init() != ESP_OK) 
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    sx126x_hal_wait_on_busy();

    /* #02 - Verify command and data lengths and prepare the tx and rx buffers */
    uint16_t total_length = command_length + data_length;
    if (total_length > 256) 
    {
        ESP_LOGE(TAG, "Transfer too large: %d bytes", total_length);
        return SX126X_HAL_STATUS_ERROR;
    }

    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];

    memcpy(tx_buffer, command, command_length);
    /* Fill remaining TX buffer with NOP for read operation */
    /* Why do we do that you may ask? Here is an explanation:
     *  SPI (Serial Peripheral Interface) is a full-duplex communication protocol. 
     *   This means that data can be sent and received at the same time. For every 
     *   clock cycle the master device generates, one bit is shifted out from the master
     *   to the slave (on the MOSI line), and simultaneously, one bit is shifted out from 
     *   the slave to the master (on the MISO line).
     *   Think of it like a circular conveyor belt or a train on a single, circular track connecting 
     *   the master and slave. To get something from the slave, the master must put something on the 
     *   belt to move it along.
     * 
     * In summary:
     * We fill the remaining TX buffer with NOP commands to:
     * - Generate clock cycles for the SX126x to send data back during read operations.
     * - Ensure the transaction length matches the command plus the expected data.
     * - Avoid unintended commands by using a safe NOP value (typically 0x00 for SX126x).
     * This is a standard SPI practice for read operations, not specific to the SX126x.*/
    for (uint16_t i = command_length; i < total_length; i++) 
    {
        tx_buffer[i] = SX126X_NOP;
    }

    trans.length = total_length * 8; // Length in bits
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;

    /* #03 - Send the prepared buffer with the command and receive data over SPI */
    ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "SPI read failed: %s", esp_err_to_name(ret));
        return SX126X_HAL_STATUS_ERROR;
    }

    /* #04 - Copy the received data from the rx buffer (skipping the command part) */
    if (data_length > 0 && data != NULL) 
    {
        memcpy(data, rx_buffer + command_length, data_length);
    }

    return SX126X_HAL_STATUS_OK;
}

/**
 * Reset the radio
 */
sx126x_hal_status_t sx126x_hal_reset(const void* context)
{
    (void)context; // Unused parameter (TODO - find out if we need this)

    if (sx126x_hal_init() != ESP_OK) 
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    ESP_LOGI(TAG, "Resetting SX126x");

    /* Reset the SX126x by pulling the reset pin low for 10ms, then high, followed by a 10ms wait 
     * for the SX126x standard calibration procedure to complete. According to the SX126x datasheet,
     * (see sx1262_interface.hpp) the reset pin should be held low for typically 100µs for the Reset 
     * to happen but we do 10ms just to be sure (could be decreased if needed). */
    gpio_set_level((gpio_num_t)SX126X_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10)); // Keep reset pin low for 10ms
    gpio_set_level((gpio_num_t)SX126X_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for 10ms after reset

    return SX126X_HAL_STATUS_OK;
}

/**
 * Wake the radio up
 */
sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    (void)context; // Unused parameter (TODO - find out if we need this)

    if (sx126x_hal_init() != ESP_OK) 
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    ESP_LOGI(TAG, "Waking up SX126x");

    /* #01 - Prepare the wake-up request */
    /* As stated in the SX126x datasheet: "Once in SLEEP mode, it is possible to wake the device up
     * from the host processor with a falling edge on the NSS line."
     *
     * We achieve this by initiating a standard SPI transaction. The SPI driver automatically 
     * pulls NSS low to start the transaction, which provides the required wakeup signal.
     * A NOP (No Operation) command is sent as the payload because it is the 
     * safest option, guaranteed to have no side effects if the chip is already awake */
    uint8_t nop_cmd = SX126X_NOP;

    spi_transaction_t trans = {0};
    trans.length = 8; // 1 byte
    trans.tx_buffer = &nop_cmd;
    trans.rx_buffer = NULL;

    /* #02 - Transmit the NOP command to wake up the SX126x */
    esp_err_t ret = spi_device_transmit(spi_handle, &trans);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "SPI wakeup failed: %s", esp_err_to_name(ret));
        return SX126X_HAL_STATUS_ERROR;
    }

    /* #03 - Wait for the SX126x to wake up */
    sx126x_hal_wait_on_busy();

    return SX126X_HAL_STATUS_OK;
}