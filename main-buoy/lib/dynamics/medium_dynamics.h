#ifndef MEDIUM_DYNAMICS_H
#define MEDIUM_DYNAMICS_H

#include <stdint.h>
#include <Arduino.h>
#include "esp_err.h"
#include "adc_measurements.h"
#include "i2c_measurements.h"
#include "uart_interface.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


class MediumDynamicsWrapper {
private: 
    BME280 _bme280; 
    ADC _anemometer; 
    ModbusRS485 _modbus_rs485; 

public: 
    /**
     * @brief Constructor for MediumDynamicsWrapper.
     * @param i2c_bus_ptr A pointer to the globally initialized I2C bus.
     * @param serial_port A reference to the globally initialized HardwareSerial port for Modbus RS-485.
     * @param bme280_cfg The configuration for the BME280 sensor.
     * @param modbus_cfg The configuration for the Modbus RS-485 sensor.
     */
    MediumDynamicsWrapper(
        TwoWire* i2c_bus_ptr, 
        HardwareSerial& modbus_serial_port,
        const BME280_Config_t& bme280_cfg, 
        const ADC_Config_t& anemometer_cfg,
        const UART_Config_t& modbus_cfg
    );

    /**
     * @brief Initializes all sensors managed by the wrapper
     * @return ESP_OK on success, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads temperature, pressure, and humidity from the BME280 sensor. Reads wind direction from ModBus.
     * @param data_k Pointer to the MediumDynamics struct to fill with data.
     * @return ESP_OK on successful read, an error code otherwise.
     * 
     * Notice that latitude and longitud from GPS are also medium dynamic speed variables. 
     * These values are added to the MediumDynamics struct in the task implementation.
     */
    esp_err_t readSingle(MediumDynamics* data_k);
}; 

#endif // MEDIUM_DYNAMICS_H