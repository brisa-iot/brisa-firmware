#ifndef ONE_WIRE_INTERFACE_H
#define ONE_WIRE_INTERFACE_H

#include <stdint.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include "esp_err.h"
#include "global_config.h"


class DS18B20 {
private: 
    OneWire _one_wire;
    DallasTemperature _dallas_temp; 

    uint8_t _device_idx; 
    uint32_t _conv_delay_ms; 

    uint8_t _device_address; 

public: 
    /**
     * @brief Constructor for the DS18B20 class.
     * @param ds18b20_config The configuration for the DS18B20 sensor.
     */
    DS18B20(const DS18B20_Config_t& ds18b20_config); 

    /**
     * @brief Initializes the OneWire bus and searches for the DS18B20 sensor.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t initialize();

    /**
     * @brief Reads the temperature from the DS18B20 sensor.
     * @param temperature Pointer to store the read temperature in Celsius.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t read_temperature(float* temperature);
};

#endif // ONE_WIRE_INTERFACE_H