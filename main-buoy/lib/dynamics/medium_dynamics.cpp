#include "esp_log.h"
#include "medium_dynamics.h"

static const char* TAG_MD = "MedDynWrapper";


MediumDynamicsWrapper::MediumDynamicsWrapper(
    TwoWire* i2c_bus_ptr, 
    HardwareSerial& modbus_serial_port,
    const BME280_Config_t& bme280_cfg, 
    const ADC_Config_t& anemometer_cfg,
    const UART_Config_t& modbus_cfg
)
    : _bme280(i2c_bus_ptr, bme280_cfg),
      _anemometer(anemometer_cfg),
      _modbus_rs485(modbus_serial_port, modbus_cfg)
{}

esp_err_t MediumDynamicsWrapper::initialize() {
    esp_err_t ret;

    ret = _bme280.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MD, "Failed to initialize BME280: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _anemometer.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MD, "Failed to initialize Anemometer ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = _modbus_rs485.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_MD, "Failed to initialize Modbus RS-485: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_MD, "Medium Dynamics Wrapper initialization complete.");
    return ESP_OK;
}

esp_err_t MediumDynamicsWrapper::readSingle(MediumDynamics* data_k) {
    esp_err_t ret;

    ret = _bme280.getRawData(
        &data_k->temperature,
        &data_k->pressure,
        &data_k->humidity
    );
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG_MD, "Failed to read BME280 data: %s", esp_err_to_name(ret));
        return ret;
    }

    data_k->windSpeed = _anemometer.get_value();

    ret = _modbus_rs485.readHoldingRegister(&data_k->windDir);
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG_MD, "Failed to read Modbus RS-485 data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}