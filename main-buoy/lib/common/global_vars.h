#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "time.h"
#include "params_config.h"

// ------------------------------------------- GPS Global Variables -------------------------------- //
// --- Data structure to hold the latest processed GPS data ---
typedef struct {
    double latitude;
    double longitude;
    struct tm gps_time_struct; 
    uint64_t timestamp_ms;          
    uint64_t esp32_timestamp_us;    
    bool is_location_valid;        
    bool is_time_valid;             
} GpsData;

// --- Default GPS Global Variables --- //
extern GpsData g_current_gps_data;          // Defined in common/uart_interface.cpp
extern SemaphoreHandle_t g_gps_data_mutex;  // Global mutex to protect g_current_gps_data when accessed by multiple task: defined in common/uart_interface.cpp
// ------------------------------------------- GPS Global Variables -------------------------------- //


// ------------------------------------------- Global IMU Variables -------------------------------- //
// --- Data structure to hold the instant IMU dynamics data ---
typedef struct {
    float accelX;       // Accelerometer X-axis value (m/s²)
    float accelY;       // Accelerometer Y-axis value (m/s²)
    float accelZ;       // Accelerometer Z-axis value (m/s²)
    float gyroX;        // Gyroscope X-axis value (rad/s)
    float gyroY;        // Gyroscope Y-axis value (rad/s)
    float gyroZ;        // Gyroscope Z-axis value (rad/s)
    float magX;         // Magnetometer X-axis value (µT)
    float magY;         // Magnetometer Y-axis value (µT)
    float magZ;         // Magnetometer Z-axis value (µT)
} IMUDynamics;

// --- Data structure to hold the averaged IMU dynamics data ---
typedef struct {
    float accelX[MAX_SAMPLES_IMU];
    float accelY[MAX_SAMPLES_IMU];
    float accelZ[MAX_SAMPLES_IMU];
    float gyroX[MAX_SAMPLES_IMU];
    float gyroY[MAX_SAMPLES_IMU];
    float gyroZ[MAX_SAMPLES_IMU];
    float magX[MAX_SAMPLES_IMU];
    float magY[MAX_SAMPLES_IMU];
    float magZ[MAX_SAMPLES_IMU];
    uint64_t timestamp[MAX_SAMPLES_IMU];  // This timestamp will represent the start or end time of the period from which the averaged data was calculated.
    size_t head;                          // Next write position
    size_t tail;                          // Oldest unread data position
    size_t count;                         // Number of elements currently in buffer
    SemaphoreHandle_t mutex;              // Protects access to the buffer
} IMUDynamicsCycle;

// --- Default Global IMU Variables --- //
extern IMUDynamicsCycle g_imu_data_logger_buffer;  // Global buffer for IMU data logger task: defined in tasks/imu_logger_task.cpp
// ------------------------------------------- Global IMU Variables -------------------------------- //



// ------------------------------------------- Global Dynamics Variables -------------------------------- //
// --- Data structure to hold the instant Med. Dynamics data ---
typedef struct {
    float temperature;      // Temperature value (°C)
    float humidity;         // Humidity value (%)
    float pressure;         // Pressure value (hPa)
    float windSpeed;        // Wind speed (m/s)
    uint8_t windDir;        // Wind direction value
    double latitude;        // Latitude value (degrees)
    double longitude;       // Longitude value (degrees)
} MediumDynamics;

// --- Data structure to hold the averaged Med. Dynamics data ---
typedef struct {
    float temperature[MAX_SAMPLES_MEDIUM_DYNAMICS];  
    float humidity[MAX_SAMPLES_MEDIUM_DYNAMICS];     
    float pressure[MAX_SAMPLES_MEDIUM_DYNAMICS];   
    float windSpeed[MAX_SAMPLES_MEDIUM_DYNAMICS]; 
    uint8_t windDir[MAX_SAMPLES_MEDIUM_DYNAMICS]; 
    double latitude[MAX_SAMPLES_MEDIUM_DYNAMICS];    
    double longitude[MAX_SAMPLES_MEDIUM_DYNAMICS];   
    uint64_t timestamp[MAX_SAMPLES_MEDIUM_DYNAMICS]; 
    size_t head;                // Next write position
    size_t tail;                // Oldest unread data position
    size_t count;               // Number of elements currently in buffer
    SemaphoreHandle_t mutex;    // Protects access to the buffer
} MediumDynamicsCycle;

// --- Default Global Medium Dynamics Variables --- //
extern MediumDynamicsCycle g_med_dynamics_buffer;  // Global buffer for Medium Dynamics data: defined in tasks/_dynamics_task.cpp

// --- Data structure to hold the instant Power Dynamics data ---
typedef struct {
    float batteryVoltage;   // Battery voltage (V)
    float batteryCurrent;   // Battery current (mA)
    float solarVoltage;     // Solar voltage (V)
    float solarCurrent;     // Solar current (mA)
    float SoC;       // Battery state of charge (%)
} PowerDynamics;

// --- Data structure to hold the averaged Power Dynamics data ---
typedef struct {
    float batteryVoltage[MAX_SAMPLES_POWER_DYNAMICS];  
    float batteryCurrent[MAX_SAMPLES_POWER_DYNAMICS];  
    float solarVoltage[MAX_SAMPLES_POWER_DYNAMICS];    
    float solarCurrent[MAX_SAMPLES_POWER_DYNAMICS];    
    float SoC[MAX_SAMPLES_POWER_DYNAMICS];      
    uint64_t timestamp[MAX_SAMPLES_POWER_DYNAMICS];    
    size_t head; // Next write position
    size_t tail; // Oldest unread data position
    size_t count; // Number of elements currently in buffer
    SemaphoreHandle_t mutex; // Protects access to the buffer
} PowerDynamicsCycle;

// --- Default Global Power Dynamics Variables --- //
extern PowerDynamicsCycle g_power_monitor_buffer;  // Global buffer for Power Monitor data: defined in tasks/_dynamics_task.cpp

// --- Data structure to hold the instant Slow Dynamics data ---
typedef struct {
    float pH;               // pH value
    float sigma;            // Conductivity value
    float oxygen;           // Oxygen value
    float temperature;      // Temperature value
} SlowDynamics;

// --- Data structure to hold the averaged Slow Dynamics data ---
typedef struct {
    float pH[MAX_SAMPLES_SLOW_DYNAMICS];          
    float sigma[MAX_SAMPLES_SLOW_DYNAMICS];       
    float oxygen[MAX_SAMPLES_SLOW_DYNAMICS];      
    float temperature[MAX_SAMPLES_SLOW_DYNAMICS]; 
    uint64_t timestamp[MAX_SAMPLES_SLOW_DYNAMICS]; 
    size_t head; // Next write position
    size_t tail; // Oldest unread data position
    size_t count; // Number of elements currently in buffer
    SemaphoreHandle_t mutex; // Protects access to the buffer
} SlowDynamicsCycle;

// --- Default Global Slow Dynamics Variables --- //
extern SlowDynamicsCycle g_slow_dynamics_buffer;  // Global buffer for Slow Dynamics data: defined in tasks/slow_dynamics_task.cpp
// ------------------------------------------- Global Dynamics Variables -------------------------------- //

#endif // GLOBAL_VARS_H