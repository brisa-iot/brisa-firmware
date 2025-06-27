#ifndef POWER_MONITOR_H
#define POWER_MONITOR_H

#include <stdint.h>
#include "esp_err.h"
#include "i2c_measurements.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


class PowerMonitor {
private: 
    INA219 _ina219; 

public: 
    /**
     * @brief Constructor for PowerMonitor.
     * @param i2c_bus_ptr A pointer to the globally initialized I2C bus.
     * @param config The configuration for the INA219 sensors.
     */
    PowerMonitor(TwoWire* i2c_bus_ptr, const INA219_Config_t& config);

    /**
     * @brief Initializes the INA219 sensors.
     * This method checks the sensor connection and sets it up.
     *
     * @return ESP_OK on successful initialization, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads a single sample from the INA219 sensors.
     * @param data_k Pointer to the PowerDynamics structure to store the read data.
     * @return ESP_OK on successful read, an error code otherwise.
     */
    esp_err_t readSingle(PowerDynamics* data_k);
}; 

#endif // POWER_MONITOR_H 