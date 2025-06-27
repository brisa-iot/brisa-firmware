#include <iostream>
#include "adc_lib.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "hal/adc_types.h"

void SensorADC::initialize() {
    ESP_ERROR_CHECK(adc1_config_width(adcBitwidth));
    ESP_ERROR_CHECK(adc1_pad_get_io_num(adc_channel, &adc_gpio));
    ESP_ERROR_CHECK(adc1_config_channel_atten(adc_channel, adc_atten));
    status = 1;
}

float SensorADC::get_value() {
    adc_raw = adc1_get_raw(adc_channel);
    adc_voltage = (adc_raw * vRef) / adcMaxResolution;
    value = (adc_voltage - offset) / scale;
    return value;
}

