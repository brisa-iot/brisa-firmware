#ifndef FAST_DYNAMICS_H
#define FAST_DYNAMICS_H

#include <stdint.h>
#include <vector>
#include "esp_err.h"
#include "i2c_measurements.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


class FastDynamicsWrapper {
private:
    IMU _imu;                   
    
public: 
    /**
     * @brief Constructor for FastDynamicsWrapper.
     * @param i2c_bus_ptr A pointer to the globally initialized I2C bus.
     * @param imu_cfg The configuration for the IMU sensor.
     * @param anemometer_cfg The configuration for the Anemometer ADC.
     */
    FastDynamicsWrapper(TwoWire* i2c_bus_ptr, const IMU_Config_t& imu_cfg); 

    /**
     * @brief Initializes all sensors managed by the wrapper.
     * @return ESP_OK on success, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads a single, instantaneous, and filtered sample from all fast dynamics sensors.
     * @param data_k Pointer to the IMUDynamics struct to fill with data.
     * @return ESP_OK on success, an error code otherwise.
     */
    esp_err_t readSingle(IMUDynamics* data_k);

    /**
     * @brief Resets the internal filter buffers for all applicable sensors.
     */
    void resetSensorFilters();
}; 

#endif // FAST_DYNAMICS_H 