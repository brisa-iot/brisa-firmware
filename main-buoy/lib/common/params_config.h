#ifndef PARAMS_CONFIG_H
#define PARAMS_CONFIG_H

#include <stdint.h>
#include <vector>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "driver/i2c.h"

#define FIRMWARE_VERSION "v1.0.0" // Firmware version for the project

// ------------------------------------------- General Configurations ---------------------------------- //
/**
 * MACRO: MEASUREMENT_INTERVAL_MINUTES
 * This macro defines the interval in minutes for the measurement cycle.
 * It is set to DEFAULT 60 minutes (1 hour) to wake up the system every hour for measurements.
 */
#define MEASUREMENT_INTERVAL_MINUTES    0.1//60     // Wake up every 1 hour
#define MEASUREMENT_INTERVAL_US         (MEASUREMENT_INTERVAL_MINUTES * 60 * 1000000ULL) // Convert to microseconds
/**
 * MACRO: MEASUREMENT_WINDOW_MINUTES
 * This macro defines the duration in minutes for the measurement window.
 * It is set to DEFAULT 20 minutes to take measurements for 20 minutes during each cycle.
 */
#define MEASUREMENT_WINDOW_MINUTES      2//10     // Take measurements for 10 minutes
#define MEASUREMENT_WINDOW_US           (MEASUREMENT_WINDOW_MINUTES * 60 * 1000000ULL) // Convert to microseconds
/**
 * MACRO: SENSOR_INITIALIZATION_DELAY_MS
 * This macro defines the delay in milliseconds for sensor initialization.
 * It is set to 500 ms to allow sensors to stabilize after power-on.
 */
#define SENSOR_INITIALIZATION_DELAY_MS (500) 
/**
 * MACRO: TIMESTAMP_INIT_MS
 * This macro defines the initial timestamp in milliseconds.
 * From January 01, 1970 to June 30, 2025.
 */
#define TIMESTAMP_INIT_MS  1748750400000ULL
/**
 * MACRO: SWITCH_LOAD_PIN
 * This macro defines the GPIO pin used to control the load switch.
 */
#define SWITCH_LOAD_PIN    GPIO_NUM_23
#define POWER_UP_DELAY_MS  (2000)       // Delay in milliseconds to power up the sensors after switching the load switch ON. Must be at least 1 second
// ------------------------------------------- General Configurations ---------------------------------- //



// ------------------------------------------- LoRa Module Configurations -------------------------------- //
#define LORA_BAUD_RATE     (115200)          // Baud rate for LoRa module
#define RXD0_LORA          (3)               // RX --> GPIO3 (Recommended to rename for clarity)
#define TXD0_LORA          (1)               // TX --> GPIO1 (Recommended to rename for clarity)
#define UART_NUM_LORA      (UART_NUM_0)      // UART Interface number for LoRa module (Recommended to rename for clarity)
/**
 * MACRO: LORA_TASK_DELAY_MS
 * Description: Delay in milliseconds for the LoRa task loop.
 * This delay is used to control the frequency of data transmission to the LoRa module.
 */
#define LORA_TASK_DELAY_MS (30000)           // Delay in milliseconds for the LoRa task loop (adjust as needed)
/**
 * MACRO: PAYLOAD_MAX_SIZE
 * This macro defines the maximum size of the payload for LoRa communication.
 */
#define PAYLOAD_MAX_SIZE   (237)
// ------------------------------------------- LoRa Module Configurations -------------------------------- //



// ------------------------------------------- Local (Flash Memory) Storage Configurations -------------------------------- //
//                                  >>> ONLY defined for the IMU Data Logger Task <<<                                       //
/**
 * Local Storage -> General Configuration ------------------------------------------------------------------
 */
#define FS_BASE_PATH            "/spiffs"   
/**
 * MACRO: PARTITION_LABEL
 * Description: Label for the partition where the file system is mounted.
 * This label MUST match the label defined in the partition table.
 */
#define PARTITION_LABEL         "storage"      
#define LOGS_DIR                FS_BASE_PATH "/logs"
/**
 * MACRO: MAX_NUM_DAYS_LOG_FILES
 * Description: Maximum number of days to keep the same log file.
 */
#define MAX_NUM_DAYS_LOG_FILES     (2)  
/**
 * Local Storage -> IMU Data Logger Task Configuration ------------------------------------------------------
 */
#define MEASUREMENT_WINDOW_IMU_MS         (180000)     // Measurement window for IMU data logging in seconds: DEFAULT is set to 180 seconds (3 minutes)
#define SAMPLE_TIME_IMU_LOGGER_MS         (100)     // 100 ms sample time for IMU data logging
#define SAMPLES_TO_AVERAGE_IMU            (5)       // 5 raw samples make one logged data point
#define SAMPLE_TIME_AVG_IMU_POINT_MS      (SAMPLE_TIME_IMU_LOGGER_MS * SAMPLES_TO_AVERAGE_IMU) // 100 ms * 5 = 500 ms for averaging IMU data points
/**
 * MACRO: TOTAL_SAMPLES_TO_LOG
 * Description: Total number of samples to log in the IMU Data Logger Task.
 * 
 * This parameter is computed by: length_imu_measurement_window (ms) / (sample_time_imu_logger (ms) * samples_to_log_imu).
 * Default is set to 360 samples, which corresponds to: 
 *          
 *         length_imu_measurement_window = DEFAULT 180 seconds (180000 ms) / (100 ms * 5) = 360.
 * 
 */
#define TOTAL_SAMPLES_TO_LOG      (240) //360
#define TOTAL_SAMPLES_TO_MEASURE  (TOTAL_SAMPLES_TO_LOG * SAMPLES_TO_AVERAGE_IMU) 
/**
 * MACRO: FLUSH_INTERVAL_SAMPLES
 * Description: Number of samples after which the data is flushed to the file system.
 */
#define FLUSH_INTERVAL_SAMPLES    (120) // Flush every 120 samples (60 seconds at 500 ms sample time)  
// ------------------------------------------- Local (Flash Memory) Storage Configurations -------------------------------- //



// ------------------------------------------- Local (RAM) Dynamic Storage Configurations -------------------------------- //
/**
 * MACRO: MAX_SAMPLES_IMU
 * Defines the maximum number of samples to store for fast-dynamics data,
 * MUST MATCH THE NUMBER OF SAMPLES TO LOG FOR IMU DATA LOGGER TASK.
 */
#define MAX_SAMPLES_IMU              (TOTAL_SAMPLES_TO_LOG)
/**
 * MACRO: MAX_SAMPLES_MEDIUM_DYNAMICS
 * Defines the maximum number of samples to store for medium-dynamics data,
 * DEFAULT 1 sample per minute over a 10-minute measurement window.
 */
#define MAX_SAMPLES_MEDIUM_DYNAMICS   (1 * 10)
/**
 * MACRO: MAX_SAMPLES_POWER_DYNAMICS
 * Defines the maximum number of samples to store for power-dynamics data,
 * DEFAULT 1 sample per minute over a 10-minute measurement window.
 */
#define MAX_SAMPLES_POWER_DYNAMICS    (1 * 10) 
/**
 * MACRO: MAX_SAMPLES_SLOW_DYNAMICS
 * Defines the maximum number of samples to store for slow-dynamics data,
 * DEFAULT 1 sample per 5 minutes over a 10-minute measurement window.
 */
#define MAX_SAMPLES_SLOW_DYNAMICS     (2)
// ------------------------------------------- Local (RAM) Dynamic Storage Configurations -------------------------------- //



// ------------------------------------------- Measurment Window Configurations -------------------------------- //
//                                  >>> Defined for the measurement window tasks <<<                             //
/**
 * Ambient Temperature, Humidity, Pressure, Wind Speed and Wind Direction Measurement Window Configurations ----
 * Power Monitor Measurement Window Configurations -------------------------------------------------------------
 */
// --- Configuration for Medium Dynamics Sampling ---
#define SAMPLE_TIME_MEDIUM_DYNAMICS_MS  (6000)  // 6 seconds per sample
#define SAMPLES_TO_AVERAGE_MEDIUM       (10)     // 10 raw samples make one averaged data point
// --- Configuration for Power Monitor Averaging ---
#define SAMPLE_TIME_POWER_MONITOR_MS    (6000)   // 6 seconds per sample
#define SAMPLES_TO_AVERAGE_POWER        (10)     // 10 raw samples make one averaged data point
// --- General Configuration for _dynamics_task.cpp (Sample Time) ---
#define SAMPLE_TIME_DYNAMICS        (SAMPLE_TIME_MEDIUM_DYNAMICS_MS)
#define SAMPLES_TO_AVERAGE_DYNAMICS (SAMPLES_TO_AVERAGE_MEDIUM) // 10 raw samples make one averaged data point
/**
 * pH, Conductivity, Dissolved Oxygen and Water Temperature Measurement Window Configurations ------------------
 */
#define SAMPLE_TIME_SLOW_DYNAMICS_MS (60000)     // Example: 1 minute (60000 ms) for slow dynamics sampling
#define SAMPLES_TO_AVERAGE_SLOW      (5)         // Example: 5 raw samples make one averaged data point
// ------------------------------------------- Measurment Window Configurations -------------------------------- //



// ------------------------------------------- Measurements ------------------------------------------- //
/**
 * ADC -> General Configurations ----------------------------------------------------------------------
 */
#define VREF                3.3f                // Reference voltage (e.g., ESP32 3.3V power rail)
#define ADC_MAX_COUNTS      4095                // Max value for a 12-bit ADC (2^12 - 1)
#define ADC_BITWIDTH_VAL    ADC_WIDTH_BIT_12    // ESP-IDF ADC bit width enum
#define ADC_UNIT_VAL        ADC_UNIT_1          // SAR ADC 1 (ADC1)
/**
 * ADC -> Anemometer Configuration --------------------------------------------------------------------
 */
#define ANEMOMETER_ADC_CHANNEL      ADC1_CHANNEL_3   // Default to GPIO39 for ADC1 channel 3
#define ANEMOMETER_ADC_ATTEN        ADC_ATTEN_DB_11 
#define ANEMOMETER_ADC_SCALE        ((5.0f * 6.0f ) / ADC_MAX_COUNTS) 
#define ANEMOMETER_ADC_OFFSET       0.0f
/**
 * ADC -> pH Sensor Configuration ---------------------------------------------------------------------
 */
#define PH_ADC_CHANNEL              ADC1_CHANNEL_4   // ADC1 channel 4 --> GPIO32
#define PH_ADC_ATTEN                ADC_ATTEN_DB_11
#define PH_SCALE                    1.0f
#define PH_OFFSET                   0.0f
/**
 * ADC -> Conductivity Sensor Configuration -----------------------------------------------------------
 */
#define CONDUCTIVITY_ADC_CHANNEL    ADC1_CHANNEL_6   // ADC1 channel 6 --> GPIO34
#define CONDUCTIVITY_ADC_ATTEN      ADC_ATTEN_DB_11
#define CONDUCTIVITY_SCALE          1.0f
#define CONDUCTIVITY_OFFSET         0.0f
/**
 * ADC -< Dissolved Oxygen Sensor Configuration -------------------------------------------------------
 */
#define DISSOLVED_O2_ADC_CHANNEL    ADC1_CHANNEL_7    // ADC1 channel 7 --> GPIO35
#define DISSOLVED_O2_ADC_ATTEN     ADC_ATTEN_DB_11
#define DISSOLVED_O2_SCALE          1.0f
#define DISSOLVED_O2_OFFSET         0.0f

/**
 * I2C -> General Configurations ----------------------------------------------------------------------
 */
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22
/**
 * i2c -> MPU9250 Configuration -----------------------------------------------------------------------
 */
#define MPU9250_ADDRESS     0x68                    // I2C address of the MPU9250
const std::vector<float> imu_filter_coeffs = {
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 
    0.1f, 0.1f, 0.1f, 0.1f, 0.1f
};
/**
 * i2c -> INA219 Configuration ------------------------------------------------------------------------
 */
#define INA219_ADDR1          (0x40)       // address INA Battery
#define INA219_ADDR2          (0x44)       // address INA Solar
#define CAPACITY              (5600)       // Battery capacity (mAh)
/**
 * MACRO: SAMPLING_TIME_SOC_MS
 * Defines the sampling time for State of Charge (SoC) calculation in milliseconds.
 * Needs to match the pre-established sampling rate for INA219 updates in "data_structures.h", mutiply by 
 * a scale factor that adjust for the need to average multiple samples for a stable reading.
 * 
 * The output is one 1 value every 1 minute, which is averaged over 10 samples. 
 * >>> Read an update value every 6 seconds
 */
#define SAMPLING_TIME_SOC_MS   (6*1000)  
/**
 * i2c -> BME280 Configuration -------------------------------------------------------------------------
 */  
#define BME280_ADDR                 (0x76)  // I2C address for the BME280 sensor
#define BME280_PRESSURE_HPA_SCALE   100.0f 

/**
 * UART -> Modbus RS-485 Configuration -----------------------------------------------------------------
 */
#define MODBUS_RS485_BAUD_RATE      (4800)            // Default baud rate for Modbus RS-485
#define RDX2_MODBUS_RS485           (16)              // RX2 pin for Modbus RS-485
#define TXD2_MODBUS_RS485           (17)              // TX2 pin for Modbus RS-485
#define UART_NUM_MODBUS_RS485       (UART_NUM_2)      // UART interface number for Modbus RS-485
/**
 * UART -> NEO-8M GPS Configuration --------------------------------------------------------------------
 */
#define GPS_BAUD_RATE     (9600)          // Baud rate for GPS NEO-8M
#define RXD1_GPS          (5)             // RX --> GPIO5 (Recommended to rename for clarity)
#define TXD1_GPS          (18)            // TX --> GPIO18 (Recommended to rename for clarity)
#define UART_NUM_GPS      (UART_NUM_1)    // UART Interface number for GPS NEO-8M (Recommended to rename for clarity)
#define GPS_TASK_DELAY_MS (100)           // Delay in milliseconds for the GPS task loop (adjust as needed)

/**
 * OneWire -> DS18B20 Configuration --------------------------------------------------------------------
 */
#define ONE_WIRE_BUS_PIN            GPIO_NUM_33    // GPIO33 pin for OneWire bus
#define DS18B20_DEVICE_IDX          (0)            // Index of the DS18B20 device to read (0 for the first device)
#define CONVERSION_DELAY            (750)          // (ms)
// ------------------------------------------- Measurements ------------------------------------------- //

#endif // PARAMS_CONFIG_H