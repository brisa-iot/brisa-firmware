#ifndef ORCHESTRATOR_H
#define ORCHESTRATOR_H

#include "Wire.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"        
#include "esp_sleep.h"    
#include "driver/gpio.h"  
#include "fast_dynamics.h"
#include "power_monitor.h"
#include "medium_dynamics.h"
#include "slow_dynamics.h"
#include "uart_interface.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"
/**
 * Tasks -----------------------
 */
#include "gps_task.h"
#include "_dynamics_task.h"
#include "slow_dynamics_task.h"
#include "imu_logger_task.h"
#include "LoRa_task.h"


class Orchestrator {
public:
    Orchestrator(
        TwoWire* i2c_bus, 
        HardwareSerial& LoRa_serial, HardwareSerial& gps_serial, HardwareSerial& modbus_serial,
        const UART_Config_t& gps_cfg, 
        const IMU_Config_t& imu_cfg, 
        const INA219_Config_t& ina219_cfg,
        const BME280_Config_t& bme280_cfg, const ADC_Config_t& anem_cfg, const UART_Config_t& modbus_cfg,
        const ADC_Config_t& ph_cfg, const ADC_Config_t& cond_cfg, const ADC_Config_t& do_cfg,
        const DS18B20_Config_t& temp_cfg, 
        const UART_Config_t& LoRa_cfg
    );
    esp_err_t initialize_wakeups();
    esp_err_t initialize_tasks();
    void run_sleep_cycle();

private:
    /***** Wake-Up Cause *****/
    esp_sleep_wakeup_cause_t _wakeup_cause;        // Stores the cause of the last wakeup
    /***** Serial Pors *****/
    HardwareSerial& _LoRa_serial;                   // Serial port for LoRa communication
    HardwareSerial& _gps_serial;                    // Serial port for GPS communication
    HardwareSerial& _modbus_serial;                 // Serial port for Modbus RS485 communication
    /***** GPS *****/
    const UART_Config_t& _gps_config;               // Reference to GPS configuration
    /***** Fast Dynamics *****/
    const IMU_Config_t& _imu_config;                // Reference to IMU configuration
    const ADC_Config_t& _anemometer_config;         // Reference to anemometer ADC configuration
    /***** Power Monitor *****/
    const INA219_Config_t& _ina219_config;          // Reference to INA219 configuration for power monitoring
    /***** Medium Dynamics *****/
    const BME280_Config_t& _bme280_config;          // Reference to BME280 configuration
    const UART_Config_t& _modbus_config;            // Reference to Modbus RS485 configuration
    /***** Slow Dynamics *****/
    const ADC_Config_t& _ph_config;                 // Reference to pH sensor configuration
    const ADC_Config_t& _conductivity_config;       // Reference to conductivity sensor configuration
    const ADC_Config_t& _dissolved_o2_config;       // Reference to dissolved oxygen sensor configuration
    const DS18B20_Config_t& _temperature_config;    // Reference to temperature sensor configuration
    /***** LoRa Module *****/
    const UART_Config_t& _LoRa_config;              // Reference to LoRa configuration

    /***** GPS Wrappers *****/
    GPS _gps_sensor;
    /**** Dynamics Wrappers ****/
    PowerMonitor _power_monitor;
    MediumDynamicsWrapper _medium_dynamics_wrapper;
    DynamicsTaskWrappers_t _dynamics_task_wrapper;
    /**** Slow Dynamics Wrapper ****/
    SlowDynamicsWrapper _slow_dynamics_wrapper;
    /**** Fast Dynamics Wrapper ****/
    FastDynamicsWrapper _fast_dynamics_wrapper; 
    /**** LoRa Module Wrapper ****/
    LoRaModule _lora_module;

    // Task Handles for controlling suspension/resumption
    TaskHandle_t _gps_task_handle = NULL;
    TaskHandle_t _dynamics_task_handle = NULL;
    TaskHandle_t _slow_dynamics_task_handle = NULL;
    TaskHandle_t _lora_tx_task_handle = NULL;
    TaskHandle_t _imu_logging_task_handle = NULL;

    // Private helper methods
    esp_err_t initialize_sensors();
    esp_err_t initialize_LoRa();
    void power_on_sensors();
    void power_off_sensors();
    void start_measurement_window();
    void stop_measurement_window();
};


#endif // ORCHESTRATOR_H