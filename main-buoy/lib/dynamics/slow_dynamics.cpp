#include "esp_log.h"
#include "slow_dynamics.h"

static const char* TAG_SD = "SlowDynWrapper"; 


SlowDynamicsWrapper::SlowDynamicsWrapper(
    const ADC_Config_t& ph_cfg, 
    const ADC_Config_t& conductivity_cfg,
    const ADC_Config_t& dissolved_o2_cfg,
    const DS18B20_Config_t& temperature_cfg
)   
    : _ph_sensor(ph_cfg), 
      _conductivity_sensor(conductivity_cfg), 
      _dissolved_o2_sensor(dissolved_o2_cfg), 
      _temperature_sensor(temperature_cfg)
{}

esp_err_t SlowDynamicsWrapper::initialize() {
    esp_err_t ret; 

    ret = _ph_sensor.initialize(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize pH ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _conductivity_sensor.initialize(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize Conductivity ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _dissolved_o2_sensor.initialize(); 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize Dissolved O2 ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _temperature_sensor.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SD, "Failed to initialize One Wire Temperature: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_SD, "Slow Dynamics Wrapper initialization complete.");
    return ESP_OK; 
}

esp_err_t SlowDynamicsWrapper::readSingle(SlowDynamics* data_k) {
    esp_err_t ret; 

    data_k->pH = _ph_sensor.get_value(); 
    data_k->sigma = _conductivity_sensor.get_value(); 
    data_k->oxygen = _dissolved_o2_sensor.get_value(); 

    ret = _temperature_sensor.read_temperature(&data_k->temperature); 
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG_SD, "Failed to read water temperature data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK; 
}

