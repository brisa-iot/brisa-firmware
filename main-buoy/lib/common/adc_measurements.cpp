#include "adc_measurements.h" 
#include "esp_log.h" 
#include "driver/adc.h" 
#include "hal/adc_types.h" 

static const char* TAG_ADC = "ADC Driver";


ADC::ADC(const ADC_Config_t& config,
         adc_unit_t adc_unit_,
         adc_bits_width_t adc_bitwidth_,
         float vref_,
         float adc_max_counts_)
    : _adc_unit(adc_unit_),
      _adc_bitwidth(adc_bitwidth_),
      _vref(vref_),
      _adc_max_counts(adc_max_counts_),
      _channel(config.adc_channel),
      _atten(config.adc_atten),
      _scale(config.scale),
      _offset(config.offset)
{}

esp_err_t ADC::initialize() {
    esp_err_t ret;

    if (_adc_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG_ADC, "Only ADC_UNIT_1 is supported in this implementation for now.");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ret = adc1_config_width(_adc_bitwidth);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ADC, "Failed to configure ADC width: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc1_config_channel_atten(_channel, _atten);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_ADC, "Failed to configure ADC channel %d attenuation: %s", _channel, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

float ADC::get_value() {
    int raw_adc_value = adc1_get_raw(_channel);
    float final_value = (float)raw_adc_value * _scale + _offset;
    return final_value;
}