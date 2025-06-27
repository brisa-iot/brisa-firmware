#include "esp_log.h"
#include "power_monitor.h"

static const char* TAG_PM = "PowerMonitor";


PowerMonitor::PowerMonitor(TwoWire* i2c_bus_ptr, const INA219_Config_t& config)
    : _ina219(i2c_bus_ptr, config) 
{}

esp_err_t PowerMonitor::initialize() {
    esp_err_t ret;

    ret = _ina219.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_PM, "Failed to initialize INA219 sensors: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_PM, "Power Monitor initialization complete.");
    return ESP_OK;
}

esp_err_t PowerMonitor::readSingle(PowerDynamics* data_k) {
    esp_err_t ret; 

    ret = _ina219.getRawData(
        &data_k->batteryVoltage, &data_k->batteryCurrent,
        &data_k->solarVoltage, &data_k->solarCurrent,
        &data_k->SoC 
    );

    if (ret != ESP_OK) {
        //ESP_LOGE(TAG_PM, "Failed to read INA219 data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}