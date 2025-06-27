#include "one_wire_interface.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char* TAG = "DS18B20";

DS18B20::DS18B20(const DS18B20_Config_t& ds18b20_config)
    : _one_wire(ds18b20_config.pin), 
      _dallas_temp(&_one_wire),
      _conv_delay_ms(ds18b20_config.conv_delay_ms), 
      _device_idx(ds18b20_config.device_idx)  
{}

esp_err_t DS18B20::initialize() {
    _dallas_temp.begin();

    if (_dallas_temp.getDeviceCount() == 0) {
        ESP_LOGE(TAG, "No DS18B20 devices found on the OneWire bus!");
        return ESP_FAIL; 
    }

    if (!_dallas_temp.getAddress(&_device_address, _device_idx)) {
        ESP_LOGE(TAG, "Failed to get address for DS18B20 device at index %d!", _device_idx);
        return ESP_FAIL; 
    }

    ESP_LOGI(TAG, "DS18B20 device found at index %d with address: %02X", _device_idx, _device_address);
    return ESP_OK; 
}

esp_err_t DS18B20::read_temperature(float* temperature) {
    DallasTemperature::request_t req_status = _dallas_temp.requestTemperatures();

    if (!req_status.result) { 
        *temperature = DEVICE_DISCONNECTED_C; 
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(_conv_delay_ms));

    float temp_c = _dallas_temp.getTempCByIndex(_device_idx);
    if (temp_c == DEVICE_DISCONNECTED_C) {
        *temperature = DEVICE_DISCONNECTED_C; 
        return ESP_FAIL; 
    }

    *temperature = temp_c;
    return ESP_OK;
}
