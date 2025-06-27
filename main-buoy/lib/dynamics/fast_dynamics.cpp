#include "esp_log.h"
#include "fast_dynamics.h"

static const char* TAG_FD = "FastDynWrapper"; 


FastDynamicsWrapper::FastDynamicsWrapper(TwoWire* i2c_bus_ptr, const IMU_Config_t& imu_cfg)
    : _imu(i2c_bus_ptr, imu_cfg)
{}

esp_err_t FastDynamicsWrapper::initialize() {
    esp_err_t ret;

    ret = _imu.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_FD, "Failed to initialize IMU: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG_FD, "Fast Dynamics Wrapper initialization complete.");
    return ESP_OK;
}

esp_err_t FastDynamicsWrapper::readSingle(IMUDynamics* data_k) {
    esp_err_t ret;

    // Get raw IMU data
    ret = _imu.getRawData(
        &data_k->accelX, &data_k->accelY, &data_k->accelZ,
        &data_k->gyroX, &data_k->gyroY, &data_k->gyroZ,
        &data_k->magX, &data_k->magY, &data_k->magZ
    );
    if (ret != ESP_OK) {
        //ESP_LOGE(TAG_FD, "Failed to read IMU data: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

void FastDynamicsWrapper::resetSensorFilters() {
    //ESP_LOGI(TAG_FD, "Resetting all sensor filter buffers.");
    _imu.resetFilterBuffers();
}