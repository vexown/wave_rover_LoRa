#include <stdio.h>
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "sx1262_interface.hpp"
#include "heltec_v3_HAL.hpp"

#define RTC_10_MS_TIMEOUT_IN_STEPS 640 // 10 ms in RTC steps (64 kHz clock, 1 step = 15.625 µs)

/* PA config values as per SX1262 datasheet */
#define PA_LUT_RESERVED             0x01
#define PA_DEVICE_SELECT_SX1262     0x00
#define PA_DEVICE_SELECT_SX1261     0x01
#define PA_DUTY_CYCLE_22DBM_SX1262  0x04
#define HP_MAX_22DBM_SX1262         0x07 
#define TX_POWER_SX1262_DB          22

/* LoRa Packet Configuration */
/* Per SX1262 datasheet, Semtech declares 1% PER (Packet Error Rate) with the following settings:
 * LoRa PER = 1%, packet 64 bytes, preamble 8 symbols, CR = 4/5, CRC on payload enabled, explicit header mode 
 *
 * Essentially, it means: if you send a packet with the above parameters you can expect 99 out of 100 packets to arrive without errors.*/
#define PREAMBLE_LENGTH_8_SYMBOLS   (uint16_t)(8)

/* Misc defines */
#define LDRO_ENABLED            0x01
#define LDRO_DISABLED           0x00
#define TIMEOUT_DISABLED        0x00
/* Extra margin (in ms) added to the RX timeout when waiting for the DIO1 IRQ semaphore.
 * This accounts for system scheduling delays, SPI latency, and ensures we don't miss the IRQ event.
 * Increase if you observe missed IRQs or false timeouts on slower systems. */
#define SX1262_RX_IRQ_MARGIN_MS 2000

/* Variable for storing the timestamp when the next transmission is allowed. */
static uint32_t next_tx_allowed_time_ms = 0;

/* LoRa modulation parameters, configured during initialization */
static sx126x_mod_params_lora_t lora_mod_params;

static float current_duty_cycle_ratio = 0.0f;

static const void* context = NULL; // Context is not used in the HAL, so it can remain NULL.

static const char* TAG = "SX1262_INTERFACE";

static inline uint32_t get_time_in_ms(void)
{
    return pdTICKS_TO_MS(xTaskGetTickCount());
}

sx1262_init_config_t sx1262_get_default_init_config(uint32_t freq_hz, float duty_cycle)
{
    sx1262_init_config_t config = {};
    
    config.frequency_hz = freq_hz;
    config.standby_mode = SX126X_STANDBY_CFG_RC;
    config.regulator_mode = SX126X_REG_MODE_DCDC;
    config.tcxo_voltage = SX126X_TCXO_CTRL_1_8V;
    config.tcxo_timeout_steps = RTC_10_MS_TIMEOUT_IN_STEPS;
    config.dio2_rf_switch_ctrl = true;
    config.packet_type = SX126X_PKT_TYPE_LORA;
    
    /* PA config for +22 dBm */
    config.pa_config.pa_duty_cycle = PA_DUTY_CYCLE_22DBM_SX1262;
    config.pa_config.hp_max = HP_MAX_22DBM_SX1262;
    config.pa_config.device_sel = PA_DEVICE_SELECT_SX1262;
    config.pa_config.pa_lut = PA_LUT_RESERVED;
    
    config.tx_power_dbm = TX_POWER_SX1262_DB;
    config.ramp_time = SX126X_RAMP_1700_US;
    
    /* Default LoRa modulation parameters */
    config.lora_mod_params.sf = SX126X_LORA_SF9;
    config.lora_mod_params.bw = SX126X_LORA_BW_125;
    config.lora_mod_params.cr = SX126X_LORA_CR_4_5;
    config.lora_mod_params.ldro = LDRO_DISABLED;
    
    /* IRQ configuration */
    config.irq_mask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
    
    config.duty_cycle_ratio = duty_cycle;
    
    return config;
}

sx1262_tx_config_t sx1262_get_default_tx_config(void)
{
    sx1262_tx_config_t config = {};
    
    config.preamble_length_symbols = PREAMBLE_LENGTH_8_SYMBOLS;
    config.header_type = SX126X_LORA_PKT_EXPLICIT;
    config.crc_enabled = true;
    config.invert_iq = false;  // Standard IQ for uplink
    config.sync_word = 0xAB;   // Custom sync word
    
    return config;
}

sx1262_rx_config_t sx1262_get_default_rx_config(uint32_t timeout_ms)
{
    sx1262_rx_config_t config = {};
    
    config.preamble_length_symbols = PREAMBLE_LENGTH_8_SYMBOLS;
    config.header_type = SX126X_LORA_PKT_EXPLICIT;
    config.crc_enabled = true;
    config.invert_iq = false;  // Standard IQ for uplink reception
    config.rx_timeout_ms = timeout_ms;
    
    return config;
}

sx126x_status_t sx1262_init_lora(const sx1262_init_config_t* config) 
{
    sx126x_status_t status;

    ESP_LOGI(TAG, "Initializing SX1262 interface...");

    /* #01 - Reset and wake up the radio */
    ESP_LOGI(TAG, "Resetting SX1262...");
    status = sx126x_reset(context);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Reset failed: %d", status);
        return status;
    }

    status = sx126x_wakeup(context);
     if (status != SX126X_STATUS_OK) 
     {
        ESP_LOGE(TAG, "Wakeup failed: %d", status);
        return status;
    }

    /* #02 - Set the SX1262 to standby mode */
    /* Chip should be in standby mode while it's being configured before entering RX or TX modes.
     *
     * We have two options for standby mode:
     * - SX126X_STANDBY_CFG_RC (STDBY_RC): The default mode that uses the 13MHz RC oscillator for reduced power
     * consumption. If the DC-DC converter is to be used, it should be enabled in this mode
     * using `SetRegulatorMode()`.
     *
     * - SX126X_STANDBY_CFG_XOSC (STDBY_XOSC): Keeps the XTAL (XOSC) turned ON. This mode
     * is suitable for time-critical applications. When switching from STDBY_RC to STDBY_XOSC,
     * the DC-DC automatically switches ON if previously enabled. */
    ESP_LOGI(TAG, "Setting standby mode...");
    status = sx126x_set_standby(context, config->standby_mode);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set standby failed: %d", status);
        return status;
    }

    /* #03 - Configure regulator and RF switch */
    /* SX126X_REG_MODE_LDO: Uses the Low-Dropout regulator (default), simpler but less efficient.
     * SX126X_REG_MODE_DCDC: Uses the DC-DC converter, more power-efficient for extended battery life.
     * DC-DC is clocked by RC 13MHz oscillator and activates automatically when entering STDBY_XOSC if enabled in STDBY_RC.*/
    ESP_LOGI(TAG, "Configuring regulator...");
    status = sx126x_set_reg_mode(context, config->regulator_mode);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set regulator failed: %d", status);
        return status;
    }

    /* #04 - Configure and enable the TCXO (on DIO3 pin of SX1262) */
    /* TCXO (Temperature-Compensated Crystal Oscillator) provides a stable clock reference for the radio transceiver.
     * These options define the supply voltage levels for the external TCXO, configurable via DIO3.
     *
     * The voltage range for TCX0 per SX1262 datasheet: | 1.6V (min) | 1.7V (typical) | 3.3V (max) |
     * 
     * The timeout is given in RTC steps. Real Time Clock (RTC) is a 64kHz RC oscillator meaning each step is 15.625 µs.
     * 10ms timeout = 10000 / 15.625 = 640 steps. */
    ESP_LOGI(TAG, "Configuring TCXO...");
    status = sx126x_set_dio3_as_tcxo_ctrl(context, config->tcxo_voltage, config->tcxo_timeout_steps);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set TCXO control failed: %d", status);
        return status;
    }


    /* #05 - Configure the RF switch on DIO2 pin */
    /* A RF switch (Radio Frequency switch) is an external component that the SX126X radio transceiver 
     * uses to route the radio signal.  In many radio systems, a single antenna is used for both transmitting 
     * (sending data) and receiving (getting data). An RF switch allows the SX126X chip to direct the high-frequency 
     * radio signal from its internal power amplifier to the antenna during transmission, and conversely, connect the antenna
     * to its low-noise amplifier during reception. This ensures the signal goes to the correct part of the radio at the right time. */
    ESP_LOGI(TAG, "Configuring DIO2 as RF switch...");
    status = sx126x_set_dio2_as_rf_sw_ctrl(context, config->dio2_rf_switch_ctrl);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set DIO2 RF switch failed: %d", status);
        return status;
    }

    /* #06 - Set the packet type */
    /* Defines the radio modulation schemes:
     * - GFSK: General-purpose, legacy FSK modulation.
     * - LORA: Long Range spread spectrum for LPWAN (Low-Power WAN) applications.
     * - BPSK: Binary Phase Shift Keying (less commonly highlighted in datasheet for this chip).
     * - LR_FHSS: Long Range Frequency Hopping Spread Spectrum for LPWAN, improving coexistence. */
    ESP_LOGI(TAG, "Setting LoRa packet type...");
    status = sx126x_set_pkt_type(context, config->packet_type);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set packet type failed: %d", status);
        return status;
    }

    /* #07 - Set the RF frequency */
    /* The actual LoRa transciever (SX1262) supports 150-960 MHz range but this particular 
     * Heltec V3 board is tuned to 863-928MHz as it says on the packaging */ 
    status = sx126x_set_rf_freq(context, config->frequency_hz);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set RF frequency failed: %d", status);
        return status;
    }

    /* #07a - Store the duty cycle ratio */
    current_duty_cycle_ratio = config->duty_cycle_ratio;
    ESP_LOGI(TAG, "Operating on frequency %lu Hz with a %.1f%% duty cycle.", config->frequency_hz, (current_duty_cycle_ratio * 100.0f));

    /* #08 - Set the PA (Power Amplifier) configuration */
    /* The PA config parameters as per SX1262 datasheet (13.1.14 SetPaConfig):
     *     - pa_duty_cycle: PA conduction angle, affects power, consumption, harmonics.
     *     - hp_max: (SX1262 only) Selects HP PA size, affects max output power (valid range 0-7, +22dBm at 0x07) (DO NOT INCREASE ABOVE 0x07 - POSSIBLE DAMAGE)
     *     - device_sel: Selects between SX1261 or SX1262 (We use 0x00 for SX1262)
     *     - pa_lut: Reserved parameter. (Always set to 0x01) 
     * 
     * For advice on optimal PA configuration, refer to the SX1262 datasheet Table 13-21: PA Operating Modes with Optimal Settings */
    ESP_LOGI(TAG, "Configuring PA for +%d dBm...", config->tx_power_dbm);
    /*********************************************************************************************************************************************************
     * WARNING - Make sure the ERP (Effective Radiated Power) does not exceed the legal limits for your region in the selected frequency band (refer to ETSI EN 300 220-2).
     *********************************************************************************************************************************************************
     For example, in Europe for band O (869,400 MHz to 869,650 MHz) the max ERP is 500 mW and ≤ 10 % duty cycle or polite spectrum access
     * In this case assuming there are no cable losses between the SX1262 chip and the antenna, and the antenna has 0 dBd gain, 
     * then the ERP will be no more than 158.49 mW which is well within the legal limits for this particular band. */
    status = sx126x_set_pa_cfg(context, &config->pa_config);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set PA config failed: %d", status);
        return status;
    }

    /* #09 - Set the TX parameters */
    /* Set output power and ramp time for the PA. This command sets the TX output power (second parameter) and the TX ramping time (third parameter) 
     * See 13.4.4 SetTxParams in the SX1262 datasheet for details. 
     *      - power parameter is connected to the PA config above and its value is suggestedin the Table 13-21: PA Operating Modes with Optimal Settings. 
     *      - ramp_time parameter defines the time it takes for the PA to ramp up to full power. They don't really suggest any specific values in the datasheet,
     *        TODO - find out how to choose this value */
    ESP_LOGI(TAG, "Setting TX power to +%d dBm...", config->tx_power_dbm);
    status = sx126x_set_tx_params(context, config->tx_power_dbm, config->ramp_time);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set TX params failed: %d", status);
        return status;
    }

    /* #10 - Configure LoRa modulation parameters */
    /* This is the bread and butter of LoRa modulation.
     * The parameters define how the LoRa signal is modulated, including:
     * - Spreading Factor (SF): Determines the time on air and range.
     * - Bandwidth (BW): Affects the data rate and sensitivity.
     * - Coding Rate (CR): Error correction level.
     * - Low Data Rate Optimization (LDRO): Optimizes for low data rates.
     *      For low data rates (typically for high SF or low BW) and very long payloads which may last several seconds in the air, the low data rate
     *      optimization (LDRO) can be enabled. This reduces the number of bits per symbol to the given SF minus two (see Section 6.1.4 "LoRa
     *      Time-on-Air" on page 41 ) in order to allow the receiver to have a better tracking of the LoRa signal. Depending on the payload size, the low
     *      data rate optimization is usually recommended when a LoRa symbol time is equal or above 16.38ms.*/
    ESP_LOGI(TAG, "Configuring LoRa modulation...");
    lora_mod_params = config->lora_mod_params;
    status = sx126x_set_lora_mod_params(context, &lora_mod_params);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set LoRa modulation failed: %d", status);
        return status;
    }

    /* #11 - Configure interrupts on DIO pins */
    /* The chip is interfaced through the 4 control lines which are composed of the BUSY pin and 3 DIOs pins that can be configured as interrupt,
     * debug or to control the radio immediate peripherals (TCXO or RF Switch) (see chapter 8.3 Multi-Purpose Digital Input/Output (DIO))
     * 
     * The SX1262 features versatile DIO pins (typically DIO1, DIO2, DIO3) that can be configured for:
     *      - General Digital I/O
     *      - Interrupts to the host microcontroller (e.g., packet events)
     *      - Debugging purposes
     *      - Control of radio peripherals:
     *          - DIO3: Controls external TCXO supply voltage.
     *          - DIO2: Controls an external RF switch for TX/RX signal routing.
     * 
     * DIO1 is used for various interrupts, including RX Done, TX Done, and other events. */
    ESP_LOGI(TAG, "Configuring IRQs...");
    status = sx126x_set_dio_irq_params( context,
                                        config->irq_mask,
                                        config->irq_mask,
                                        0,
                                        0
                                    );
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set DIO IRQ params failed: %d", status);
        return status;
    }

    return status; // This will return success, otherwise the function would've returned earlier with an error status.
}


sx126x_status_t sx1262_send_packet(uint8_t* payload, uint8_t payload_length, const sx1262_tx_config_t* config)
{
    sx126x_status_t status;
    sx126x_pkt_params_lora_t lora_packet_cfg;

    if (config == NULL)
    {
        ESP_LOGE(TAG, "Invalid config pointer!");
        return SX126X_STATUS_ERROR;
    }

    /* #01 - Check Duty Cycle compliance */
    uint32_t current_time_ms = get_time_in_ms();
    if (current_time_ms < next_tx_allowed_time_ms)
    {
        uint32_t wait_time_ms = next_tx_allowed_time_ms - current_time_ms;

        ESP_LOGE(TAG, "Duty cycle violation: Must wait %lu ms before next TX.", wait_time_ms);
        return SX126X_STATUS_ERROR; 
    }

    /* #02 - Define the configuration for the LoRa packet and apply it */
    ESP_LOGI(TAG, "Configuring LoRa packet parameters...");
    /* Set general LoRa packet configuraton*/
    lora_packet_cfg.preamble_len_in_symb = config->preamble_length_symbols;
    lora_packet_cfg.header_type = config->header_type;
    lora_packet_cfg.pld_len_in_bytes = payload_length;
    lora_packet_cfg.crc_is_on = config->crc_enabled;
    /* This inversion is critical for network operation, primarily to distinguish between data traffic directions.
    * A typical LoRaWAN network convention is as below but for private LoRa networks you can choose either way as long as the Tx and Rx settings match:
    * - UPLINK (Device to Gateway): Typically uses standard, non-inverted I/Q to produce UP-CHIRPS.
    * - DOWNLINK (Gateway to Device): Typically uses inverted I/Q to produce DOWN-CHIRPS.
    * 
    *  Key point - device configured as Standard IQ will IGNORE Inverted IQ packets and vice versa. This is the main point of IQ inversion in LoRa - 
    *  it allows the same radio to differentiate between uplink and downlink traffic by simply switching the I/Q configuration.
    *
    *  Therefore, a receiver MUST be configured with the SAME IQ setting as the transmitter it is listening to. If the settings mismatch 
    *  (e.g., Rx expects UP-CHIRPS but Tx sends DOWN-CHIRPS), the packet will not be detected.
    *
    *  The `invert_iq_is_on` flag controls this behavior.
    *   - `false`: Use standard I/Q. 
    *   - `true`: Use inverted I/Q. 
    */
    lora_packet_cfg.invert_iq_is_on = config->invert_iq;
  
    status = sx126x_set_lora_pkt_params(context, &lora_packet_cfg);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set LoRa packet params failed: %d", status);
        return status;
    }

    /* Sync Word comes after the Preamble and is used to distinguish between different networks
     * Default for private LoRa networks is 0x12 (so in registers it will result in 0x1424 based on sx1262 datasheet)
     * My personal choice for the sync word is 0xAB (in registers it will result in 0xA4B4)  */
    status = sx126x_set_lora_sync_word(context, config->sync_word);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set LoRa sync word failed: %d", status);
        return status;
    }

    /* #03 - Calculate the Time-on-Air (ToA) for the packet */
    uint32_t time_on_air_ms = sx126x_get_lora_time_on_air_in_ms(&lora_packet_cfg, &lora_mod_params);
    ESP_LOGI(TAG, "Packet ToA is %lu ms", time_on_air_ms);

    /* #04 - Write the payload to the radio TX buffer */
    ESP_LOGI(TAG, "Payload length: %u", payload_length);
    ESP_LOG_BUFFER_HEX(TAG, payload, payload_length);

    status = sx126x_write_buffer(context, 0x00, payload, payload_length);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Write buffer failed: %d", status);
        return status;
    }

    /* #05 - Clear any pending IRQs (so we know any new IRQs are fresh) */
    status = sx126x_clear_irq_status(context, SX126X_IRQ_ALL);  
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Clear IRQ failed: %d", status);
    }

    /* #06 - Send the packet by setting the radio to transmit mode */
    /* The transmit flow based on SX1262 datasheet (SetTx command description):
     * - Starting from STDBY_RC mode, the oscillator is switched ON followed by the PLL, then the PA is switched ON and
     *   the PA regulator starts ramping according to the ramping time defined by the command SetTxParams(...).
     * - When the ramping is completed, the packet handler starts the packet transmission.
     * - When the last bit of the packet has been sent, an IRQ TX_DONE is generated,
     *   the PA regulator is ramped down, the PA is switched OFF and the chip goes back to STDBY_RC mode.
     * - A TIMEOUT IRQ is triggered if the TX_DONE IRQ is not generated within the given timeout period.
     * - The chip goes back to STBY_RC mode after a TIMEOUT IRQ or a TX_DONE IRQ.
     */
    ESP_LOGI(TAG, "Starting transmission...");
    /* The timeout arg acts as a 'safety watchdog' and should be set to a value bit longer than the expected Time-on-Air (ToA) of your LoRa packet. 
     * - If for some reason the LoRa transmission does not complete within this specified time the SX1262 will automatically stop transmitting and 
     * return to standby mode.
     * - If set to 0 - timeout is disabled and the device stays in TX Mode until the packet is transmitted and returns in STBY_RC mode upon completion.
     * The value given for the timeout should be calculated for a given packet size, given modulation and packet parameters.
     */
    const uint32_t tx_timeout_ms = time_on_air_ms + 500; // Add 500ms buffer to the ToA to account for any delays or processing time
    status = sx126x_set_tx(context, tx_timeout_ms);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set TX failed: %d", status);
        return status;
    }

    /* #07 - Wait for the transmission to complete which will be signaled by TX_DONE IRQ (or to timeout which is also signaled by an IRQ) */
    bool tx_success = false;
    if (xSemaphoreTake(dio1_sem, pdMS_TO_TICKS(tx_timeout_ms + 1000)) == pdTRUE) 
    {
        sx126x_irq_mask_t irq_status;
        status = sx126x_get_and_clear_irq_status(context, &irq_status);
        if (status == SX126X_STATUS_OK) 
        {
            if (irq_status & SX126X_IRQ_TX_DONE)
            {
                ESP_LOGI(TAG, "Transmission completed successfully!");
                tx_success = true;

                /* #07a - Update Duty Cycle state on successful transmission */
                uint32_t end_tx_time_ms = get_time_in_ms();
                /* Calculate required off-time based on ToA and duty cycle: 
                 *            Off-Time = (ToA / DutyCycle) - ToA 
                 * So for example if ToA is 1000ms and DutyCycle is 10% (0.1), then:
                 * Off-Time = (1000ms / 0.1) - 1000ms = 10000ms - 1000ms = 9000ms
                 * This means after a 1000ms transmission, we must wait 9000ms before the next transmission 
                 * NOTE - THIS ASSUMES PER-PACKET DUTY CYCLE CALCULATION (NOT PER-HOUR)
                 * If you want to implement a per-hour duty cycle, you would need to keep track
                 * of the total transmission time in the last hour and calculate the off-time accordingly (TODO - implement both options?) 
                 */
                uint32_t off_time_ms = (uint32_t)(time_on_air_ms / current_duty_cycle_ratio) - time_on_air_ms;
                next_tx_allowed_time_ms = end_tx_time_ms + off_time_ms;

                ESP_LOGI(TAG, "Duty Cycle: Off-time is %lu ms. Next TX allowed at %lu ms.", off_time_ms, next_tx_allowed_time_ms);
            }
            else if (irq_status & SX126X_IRQ_TIMEOUT) 
            {
                ESP_LOGW(TAG, "Transmission timeout IRQ!");
            }
        }
    }
    else 
    {
        ESP_LOGE(TAG, "Weird case, we didn't get IRQ either for TX_DONE or TIMEOUT - check your interrupt configuration!");
    }

    /* #08 - If transmission was not successful, check for errors reported by the radio */
    if (!tx_success) 
    {
        sx126x_errors_mask_t errors;
        status = sx126x_get_device_errors(context, &errors);
        if (status == SX126X_STATUS_OK) 
        {
            if (errors != 0) 
            {
                ESP_LOGE(TAG, "Radio reported errors: 0x%04X", errors);
                if (errors & SX126X_ERRORS_PA_RAMP) ESP_LOGE(TAG, " - PA Ramping failed");
                if (errors & SX126X_ERRORS_PLL_LOCK) ESP_LOGE(TAG, " - PLL lock failed");
                if (errors & SX126X_ERRORS_XOSC_START) ESP_LOGE(TAG, " - XOSC start failed");
                if (errors & SX126X_ERRORS_IMG_CALIBRATION) ESP_LOGE(TAG, " - Image calibration failed");
                sx126x_clear_device_errors(context);
            } 
            else 
            {
                ESP_LOGI(TAG, "No errors reported by the radio.");
            }
        } 
        else 
        {
            ESP_LOGE(TAG, "Failed to get device errors.");
        }
    }

    /* #09 - Set the radio back to standby mode */
    status = sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set standby failed: %d", status);
        return status;
    }
    else if (tx_success) 
    {
        ESP_LOGI(TAG, "Transmission completed successfully and radio is back in standby mode.");
        return SX126X_STATUS_OK;
    }
    else 
    {
        ESP_LOGE(TAG, "Transmission failed or no packet sent.");
        return SX126X_STATUS_ERROR; 
    }
}

sx126x_status_t sx1262_receive_packet(uint8_t* payload, uint8_t payload_length, lora_packet_metrics_t* pkt_metrics, const sx1262_rx_config_t* config)
{
    sx126x_status_t status;
    sx126x_pkt_params_lora_t lora_packet_cfg;

    if (config == NULL)
    {
        ESP_LOGE(TAG, "Invalid config pointer!");
        return SX126X_STATUS_ERROR;
    }

    uint32_t rx_timeout_ms = config->rx_timeout_ms;

    /* #01 - Set the radio in receiver mode */
    /* This command sets the chip in RX mode, waiting for the reception of one or several packets. The receiver mode operates with a timeout
     * to provide maximum flexibility to end users */
    ESP_LOGI(TAG, "Setting radio to receive mode...");

    if (rx_timeout_ms == SX126X_RX_SINGLE_MODE)
    {
        ESP_LOGI(TAG, "RX single mode (waiting forever) is not supported by sx1262_receive_packet function - defaulting to max timeout...");
        rx_timeout_ms = SX126X_MAX_TIMEOUT_IN_MS;
    }
    else if (rx_timeout_ms == SX126X_RX_CONTINUOUS)
    {
        ESP_LOGW(TAG, "RX continuous mode not supported by sx1262_receive_packet function - defaulting to max timeout...");
        rx_timeout_ms = SX126X_MAX_TIMEOUT_IN_MS;
    }
    else if (rx_timeout_ms > SX126X_MAX_TIMEOUT_IN_MS)
    {
        ESP_LOGW(TAG, "RX timeout value too high - defaulting to max timeout...");
        rx_timeout_ms = SX126X_MAX_TIMEOUT_IN_MS;
    }

    status = sx126x_set_rx(context, rx_timeout_ms);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set RX failed: %d", status);
        return status;
    }

    /* #02 - Define the configuration for the LoRa packet and apply it */
    ESP_LOGI(TAG, "Configuring the expected LoRa packet parameters...");
    lora_packet_cfg.preamble_len_in_symb = config->preamble_length_symbols;
    lora_packet_cfg.header_type = config->header_type;
    lora_packet_cfg.pld_len_in_bytes = payload_length;
    lora_packet_cfg.crc_is_on = config->crc_enabled;
    lora_packet_cfg.invert_iq_is_on = config->invert_iq;
  
    status = sx126x_set_lora_pkt_params(context, &lora_packet_cfg);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set LoRa packet params failed: %d", status);
        return status;
    }

    /* #03 - Wait for the reception to complete which will be signaled by RX_DONE IRQ (or to timeout which is also signaled by an IRQ) */
    bool rx_success = false;
    TickType_t dio1_sem_block_time = pdMS_TO_TICKS(rx_timeout_ms + SX1262_RX_IRQ_MARGIN_MS); // will not overflow due to guards at the start of the func
    if (xSemaphoreTake(dio1_sem, dio1_sem_block_time) == pdTRUE) 
    {
        sx126x_irq_mask_t irq_status;
        status = sx126x_get_and_clear_irq_status(context, &irq_status);
        if (status == SX126X_STATUS_OK) 
        {
            if (irq_status & SX126X_IRQ_RX_DONE) 
            {
                ESP_LOGI(TAG, "Reception completed successfully!");
                rx_success = true;

                /* #04 - Read the received packet from the RX buffer and copy it to the provided payload buffer */
                sx126x_rx_buffer_status_t rx_status;
                status = sx126x_get_rx_buffer_status(context, &rx_status);
                if (status == SX126X_STATUS_OK && rx_status.pld_len_in_bytes > 0) 
                {
                    if(payload_length < rx_status.pld_len_in_bytes) 
                    {
                        ESP_LOGE(TAG, "Provided payload buffer is too small! Expected at least %u bytes, got %u bytes", rx_status.pld_len_in_bytes, payload_length);
                        ESP_LOGE(TAG, "Truncating received payload to fit the buffer size.");
                        rx_status.pld_len_in_bytes = payload_length; // Truncate to fit the buffer
                    }

                    status = sx126x_read_buffer(context, rx_status.buffer_start_pointer, payload, rx_status.pld_len_in_bytes);
                    if (status == SX126X_STATUS_OK) 
                    {
                        ESP_LOGI(TAG, "Received: %.*s", rx_status.pld_len_in_bytes, payload);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to get RX buffer status: %d", status);
                }
                /* #05 - Get the packet status (SNR and RSSI) */
                /* See GetPacketStatus command in SX1262 datasheet for details. Here is the breakdown of pkt_status fields:
                 *     - rssi_pkt_in_dbm: Average over last packet received of RSSI. Actual signal power is –RssiPkt/2 (dBm) 
                 *     - snr_pkt_in_db: Estimation of SNR on last packet received in two’s compliment format multiplied by 4. Actual SNR in dB = SnrPkt/4
                 *     - signal_rssi_pkt_in_dbm: Estimation of RSSI of the LoRa signal (after despreading) on last packet received. Actual Rssi in dB = -SignalRssiPkt/2
                 * It's a bit misleading but the above calculations are already done in the sx126x_get_lora_pkt_status function (see it in sx126x.c for yourself).
                 * So you can just use the values from sx126x_get_lora_pkt_status directly without any further calculations.
                 */
                sx126x_pkt_status_lora_t pkt_status;
                status = sx126x_get_lora_pkt_status(context, &pkt_status);
                pkt_metrics->snr_db = pkt_status.snr_pkt_in_db;
                pkt_metrics->rssi_dbm = pkt_status.rssi_pkt_in_dbm;
                pkt_metrics->signal_rssi_dbm = pkt_status.signal_rssi_pkt_in_dbm;
                if (status == SX126X_STATUS_OK) 
                {
                    ESP_LOGI(TAG, "Packet Status - SNR: %d, RSSI: %d, Signal RSSI: %d", pkt_metrics->snr_db, pkt_metrics->rssi_dbm, pkt_metrics->signal_rssi_dbm);
                }
                else 
                {
                    ESP_LOGE(TAG, "Failed to get packet status: %d", status); // TODO - add specific error prints depending on the status code (see GetPacketStatus in SX1262 datasheet)
                }
            }
            if (irq_status & SX126X_IRQ_TIMEOUT) 
            {
                ESP_LOGW(TAG, "Reception timeout IRQ!");
            }
        }
    }
    else 
    {
        ESP_LOGE(TAG, "Weird case, we didn't get IRQ either for RX_DONE or TIMEOUT - check your interrupt configuration or try increasing the SX1262_RX_IRQ_MARGIN_MS value!");
    }

    /* #06 - Set the radio back to standby mode */
    status = sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set standby failed: %d", status);
        return status;
    }
    else if (!rx_success) 
    {
        ESP_LOGE(TAG, "Reception failed or no packet received.");
        return SX126X_STATUS_ERROR; // Return error if reception was not successful
    }
    else 
    {
        ESP_LOGI(TAG, "Reception completed successfully and radio is back in standby mode.");
        return SX126X_STATUS_OK; // Return success if reception was successful
    }
}

sx126x_status_t sx1262_get_rssi_instant(int16_t* rssi_dbm)
{
    /* Get instantaneous RSSI - must be called while in RX mode */
    sx126x_status_t status = sx126x_get_rssi_inst(context, rssi_dbm);
    if (status == SX126X_STATUS_OK) 
    {
        ESP_LOGI(TAG, "Instantaneous RSSI: %d dBm", *rssi_dbm);
    }
    else 
    {
        ESP_LOGE(TAG, "Failed to get instantaneous RSSI: %d", status);
    }
    
    return status;
}

sx126x_status_t sx1262_set_continuous_rx(void)
{
    ESP_LOGI(TAG, "Setting radio to continuous RX mode...");
    
    /* Rx Continuous mode. The device remains in RX mode until the host sends a command to change the operation mode. 
     * The device can receive several packets. Each time a packet is received, a packet done indication is given to the host and the device 
     * automatically searches for a new packet. Useful for continuous monitoring applications but also for noise level measurements. 
     * As per sx126.h we need to use the sx126x_set_rx_with_timeout_in_rtc_step() function to achieve this mode. Regular sx126x_set_rx won't work. */
    sx126x_status_t status = sx126x_set_rx_with_timeout_in_rtc_step(context, SX126X_RX_CONTINUOUS);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Failed to set continuous RX mode: %d", status);
    }
    else 
    {
        ESP_LOGI(TAG, "Radio successfully set to continuous RX mode");
    }
    
    return status;
}

esp_err_t control_external_LED(bool state)
{
    static bool is_initialized = false;
    if (!is_initialized) 
    {
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << HELTEC_V3_PIN_37);
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

        gpio_config(&io_conf);

        is_initialized = true;
    }

    /* Control the external LED I've attached to the Heltec V3 (makes sure it's connected physically) */
    esp_err_t err = gpio_set_level(HELTEC_V3_PIN_37, state ? 0 : 1); // Active low, so 0 means ON, 1 means OFF
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to control external LED: %d", err);
        return err;
    }
    else
    {
        return ESP_OK;
    }
}