#ifndef ADC_MEAS_H
#define ADC_MEAS_H

#include <stdint.h>
#include "driver/adc.h"
#include "esp_err.h" 
#include "params_config.h"
#include "global_config.h"


// ---------------------------- ADC Class Definition ----------------------------
class ADC {
private:
    adc_unit_t _adc_unit;            // Which ADC unit (ADC1 or ADC2)
    adc_bits_width_t _adc_bitwidth;  // ADC resolution bits
    float _vref;                     // Reference voltage for calculations
    float _adc_max_counts;           // Max ADC value for resolution

    adc1_channel_t _channel;         // Specific ADC channel for this instance
    adc_atten_t _atten;              // Attenuation for the channel
    float _scale;                    // Scaling factor for physical conversion
    float _offset;                   // Offset for calibration

public:
    /**
     * @brief Constructor for the ADC class.
     * @param config The complete configuration struct for the ADC.
     * @param adc_unit_ The ADC unit (defaults to ADC_UNIT_VAL).
     * @param adc_bitwidth_ The ADC bit width (defaults to ADC_BITWIDTH_VAL).
     * @param vref_ The reference voltage (defaults to VREF).
     * @param adc_max_counts_ The max ADC counts (defaults to ADC_MAX_COUNTS).
     */
    ADC(
        const ADC_Config_t& config,
        adc_unit_t adc_unit_ = ADC_UNIT_VAL,
        adc_bits_width_t adc_bitwidth_ = ADC_BITWIDTH_VAL,
        float vref_ = VREF,
        float adc_max_counts_ = ADC_MAX_COUNTS
    );

    /**
     * @brief Initializes the ADC hardware based on the configured parameters.
     * @return ESP_OK on success, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads a value from the ADC, applies scaling and offset, returning the physical value.
     * @return The processed physical value (e.g., wind speed in m/s).
     */
    float get_value();
};

#endif // ADC_MEAS_H