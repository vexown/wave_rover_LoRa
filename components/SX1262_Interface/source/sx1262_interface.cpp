#include <stdio.h>
#include "sx1262_interface.hpp"
#include "esp_log.h"

#define RTC_10_MS_TIMEOUT_IN_STEPS 640 // 10 ms in RTC steps (64 kHz clock, 1 step = 15.625 µs)

/* PA config values as per SX1262 datasheet */
#define PA_LUT_RESERVED             0x01
#define PA_DEVICE_SELECT_SX1262     0x00
#define PA_DEVICE_SELECT_SX1261     0x01
#define PA_DUTY_CYCLE_22DBM_SX1262  0x04
#define HP_MAX_22DBM_SX1262         0x07 
#define TX_POWER_SX1262_DB          22

/* Misc defines */
#define LDRO_ENABLED            0x01
#define LDRO_DISABLED           0x00

/* ETSI EN 300 220-2 V3.3.1 (2025-01) defines the frequency bands and power limits for "short-range" (more like low-power) (SRD) 
 * sub 1GHz (25MHz - 1000MHz) up to 500mW e.r.p RF devices in Europe (see Annex B for details):
 * https://www.etsi.org/deliver/etsi_en/300200_300299/30022002/03.03.01_30/en_30022002v030301va.pdf 
 * 
 * As for WHERE exactly within the Band to place your carrier frequency:
 * Per ETSI EN 300 220-2, Clause 4.2.3, for equipment where the
 * Permitted Frequency Band (PFB) is not channelized by regulation (band D or band J)
 * the Nominal Operating Frequency (carrier frequency) must always be
 * the mid-point of your specific Operating Channel (OC).
 * 
 * Your "Operating Channel" is defined by the bandwidth of your LoRa signal.
 * 
 * This rule applies regardless of how wide the overall permitted band is.
 * A wider band simply offers flexibility in *where* you place your LoRa
 * operating channel within that band. However, once placed, the carrier
 * for that particular operating channel must remain centered within it.
 * 
 * For example:
 * - If using the 869.400 MHz to 869.650 MHz band (Band O) with a 250 kHz OCW,
 *   the ideal carrier is 869.525 MHz to center the 250 kHz signal within that band.
 * - If using a wider band (e.g., 863 MHz - 870 MHz) and you choose to transmit
 *   a 125 kHz signal centered at 868.1 MHz, your Operating Channel would be
 *   868.0375 MHz to 868.1625 MHz, and 868.1 MHz is its midpoint.
 * 
 * This ensures your signal remains entirely within your chosen operating channel
 * and does not intrude on other frequencies.
 * 
 * Below are the definitions for ETSI bands relevant to what this specific Heltec V3 board is tuned to (says 863-928MHz on the packaging) 
 */
#define FREQUENCY_ETSI_EN_300_220_BAND_K 864000000 // (864MHz)      863 MHz to 865 MHz 25 mW e.r.p. ≤ 0,1 % duty cycle or polite spectrum access 
#define FREQUENCY_ETSI_EN_300_220_BAND_L 866500000 // (866.5MHz)    865 MHz to 868 MHz 25 mW e.r.p. ≤ 1 % duty cycle or polite spectrum access
#define FREQUENCY_ETSI_EN_300_220_BAND_M 868300000 // (868.3MHz)    868,000 MHz to 868,600 MHz 25 mW e.r.p. ≤ 1 % duty cycle or polite spectrum access
#define FREQUENCY_ETSI_EN_300_220_BAND_N 868950000 // (868.950MHz)  868,700 MHz to 869,200 MHz 25 mW e.r.p. ≤ 0,1 % duty cycle or polite spectrum access
#define FREQUENCY_ETSI_EN_300_220_BAND_O 869525000 // (869.525MHz)  869,400 MHz to 869,650 MHz 500 mW e.r.p. ≤ 10 % duty cycle or polite spectrum access

static const void* context = NULL; // Context is not used in the HAL, so it can remain NULL.

static const char* TAG = "SX1262_INTERFACE";

sx126x_status_t sx1262_init_lora(void) 
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
    status = sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
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
    status = sx126x_set_reg_mode(context, SX126X_REG_MODE_DCDC);
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
    status = sx126x_set_dio3_as_tcxo_ctrl(context, SX126X_TCXO_CTRL_1_8V, RTC_10_MS_TIMEOUT_IN_STEPS);
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
    status = sx126x_set_dio2_as_rf_sw_ctrl(context, true);
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
    status = sx126x_set_pkt_type(context, SX126X_PKT_TYPE_LORA);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set packet type failed: %d", status);
        return status;
    }

    /* #07 - Set the RF frequency */
    /* The actual LoRa transciever (SX1262) supports 150-960 MHz range but this particular 
     * Heltec V3 board is tuned to 863-928MHz as it says on the packaging */ 
    ESP_LOGI(TAG, "Setting RF frequency to 868 MHz...");
    uint32_t freq_in_hz = FREQUENCY_ETSI_EN_300_220_BAND_O;
    status = sx126x_set_rf_freq(context, freq_in_hz);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set RF frequency failed: %d", status);
        return status;
    }

    /* #08 - Set the PA (Power Amplifier) configuration */
    /* The PA config parameters as per SX1262 datasheet (13.1.14 SetPaConfig):
     *     - pa_duty_cycle: PA conduction angle, affects power, consumption, harmonics.
     *     - hp_max: (SX1262 only) Selects HP PA size, affects max output power (valid range 0-7, +22dBm at 0x07) (DO NOT INCREASE ABOVE 0x07 - POSSIBLE DAMAGE)
     *     - device_sel: Selects between SX1261 or SX1262 (We use 0x00 for SX1262)
     *     - pa_lut: Reserved parameter. (Always set to 0x01) 
     * 
     * For advice on optimal PA configuration, refer to the SX1262 datasheet Table 13-21: PA Operating Modes with Optimal Settings */
    ESP_LOGI(TAG, "Configuring PA for +22 dBm (158.49mW)...");
    /*********************************************************************************************************************************************************
     * WARNING - Make sure the ERP (Effective Radiated Power) does not exceed the legal limits for your region in the selected frequency band (refer to ETSI EN 300 220-2).
     *********************************************************************************************************************************************************
     For example, in Europe for band O (869,400 MHz to 869,650 MHz) the max ERP is 500 mW and ≤ 10 % duty cycle or polite spectrum access
     * In this case assuming there are no cable losses between the SX1262 chip and the antenna, and the antenna has 0 dBd gain, 
     * then the ERP will be no more than 158.49 mW which is well within the legal limits for this particular band. */
    sx126x_pa_cfg_params_t pa_config = 
    {
        .pa_duty_cycle = PA_DUTY_CYCLE_22DBM_SX1262,
        .hp_max = HP_MAX_22DBM_SX1262,     
        .device_sel = PA_DEVICE_SELECT_SX1262, 
        .pa_lut = PA_LUT_RESERVED
    };

    status = sx126x_set_pa_cfg(context, &pa_config);
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
    ESP_LOGI(TAG, "Setting TX power to +22 dBm (158.49mW)...");
    status = sx126x_set_tx_params(context, (int8_t)TX_POWER_SX1262_DB, SX126X_RAMP_1700_US);
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
    sx126x_mod_params_lora_t lora_mod_params = 
    {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = LDRO_DISABLED
    };

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
     * DIO1 is used for various interrupts, including RX Done, TX Done, and other events.
     * Here we configure it to trigger on TX events only for now (TODO - add other events if needed) */
    ESP_LOGI(TAG, "Configuring IRQ for TX Done...");
    status = sx126x_set_dio_irq_params(context, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, 0, 0);
    if (status != SX126X_STATUS_OK) 
    {
        ESP_LOGE(TAG, "Set DIO IRQ params failed: %d", status);
        return status;
    }

    return status; // This will return success, otherwise the function would've returned earlier with an error status.
}
