#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

#include <stdint.h>
#include <vector>
#include "driver/adc.h"

// ------------------------------------------- ADC Configurations -------------------------------- //
/**
 * @brief Configuration structure for ADC instances.
 * This structure holds all necessary parameters to configure an ADC channel.
 */
typedef struct {
    adc1_channel_t adc_channel; 
    adc_atten_t adc_atten;
    float scale;
    float offset;
} ADC_Config_t;

// --- Default ADC Global Configurations --- //
extern const ADC_Config_t anemometer_config;    // Global configuration for ADC: defined in tasks/_dynamics_task.cpp
extern const ADC_Config_t ph_config;            // Global configuration for ADC: defined in tasks/slow_dynamics_task.cpp
extern const ADC_Config_t conductivity_config;  // Global configuration for ADC: defined in tasks/slow_dynamics_task.cpp
extern const ADC_Config_t dissolved_o2_config;  // Global configuration for ADC: defined in tasks/slow_dynamics_task.cpp
// ------------------------------------------- ADC Configurations -------------------------------- //


// ------------------------------------------- i2c Configurations -------------------------------- //
/**
 * @brief Configuration structure for I2C sensors.
 * This structure holds all necessary parameters to configure each I2C sensor.
 */

typedef struct {
    uint8_t i2c_address;     
    float pressure_hPa_scale;                        
} BME280_Config_t;

typedef struct {
    uint8_t i2c_address_battery; 
    uint8_t i2c_address_solar;  
    uint32_t capacity_mah;        
    uint32_t dt_soc_ms;              
} INA219_Config_t;

typedef struct {
    uint8_t i2c_address;
    std::vector<float> filter_coeffs;
} IMU_Config_t;

// --- Default I2C Global Configurations --- //
extern const BME280_Config_t bme280_config;  // Global configuration for BME280 sensor: defined in tasks/_dynamics_task.cpp
extern const IMU_Config_t imu_config;        // Global configuration for IMU sensor: defined in tasks/imu_logger_task.cpp
extern const INA219_Config_t ina219_config;  // Global configuration for INA219 sensors: defined in tasks/_dynamics_task.cpp
// ------------------------------------------- i2c Configurations -------------------------------- //


// ------------------------------------------- UART Configurations -------------------------------- //
/**
 * @brief Configuration structure for UART sensors.
 * This structure holds all necessary parameters to configure each UART sensor.
 */
typedef struct {
    uint32_t baud_rate;  
    uint8_t rx_pin;      
    uint8_t tx_pin;     
    uint8_t uart_num;    
} UART_Config_t;

// --- Default UART Global Configurations --- //
extern const UART_Config_t gps_config;      // Global configuration for GPS NEO-8M: defined in tasks/gps_task.cpp
extern const UART_Config_t modbus_config;   // Global configuration for Modbus RS-485: defined in tasks/_dynamics_task.cpp
extern const UART_Config_t LoRa_config;     // Global configuration for LoRa: defined in tasks/LoRa_task.cpp
// ------------------------------------------- UART Configurations -------------------------------- //


// ------------------------------------------- OneWire Configurations -------------------------------- //
/**
 * @brief Configuration structure for OneWire sensors.
 * This structure holds all necessary parameters to configure each OneWire sensor.
 */
typedef struct {
    uint8_t pin;            // GPIO pin number for the OneWire bus
    uint8_t device_idx;     // Index of the DS18B20 device to read (0 for the first device)
    uint32_t conv_delay_ms; // Conversion delay in milliseconds for the DS18B20 sensor
} DS18B20_Config_t;

// --- Default OneWire Global Configurations --- //
extern const DS18B20_Config_t temperature_config;   // Global configuration for OneWire: defined in tasks/slow_dynamics_task.cpp
// ------------------------------------------- OneWire Configurations -------------------------------- //


#endif // GLOBAL_CONFIG_H