#ifndef IMU_DATA_LOGGER_TASK_H
#define IMU_DATA_LOGGER_TASK_H

#include <Wire.h>
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "fast_dynamics.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"
#include "file_managment.h"


typedef struct {
    FILE* current_log_file_handle;
    std::string current_log_filename_date_part; 
} DailyLogFileContext_t;

/**
 * @brief Initializes the IMU Data Logger system, including FreeRTOS queues,
 * the averaged data buffer, sensor wrapper, and creates the tasks.
 * 
 * @param fast_dynamics_wrapper A reference to the FastDynamicsWrapper instance for IMU data.
 * @param imu_logging_task_handler A reference to the TaskHandle_t for the IMU logging task.
 */
esp_err_t imu_logger_task_init(FastDynamicsWrapper& fast_dynamics_wrapper, TaskHandle_t& imu_logging_task_handler); 

/**
 * @brief Task that reads data from the IMU sensors.
 * This task reads raw data from the IMU sensors, averages it over a defined number of samples,
 * and stores the averaged data in the respective buffers.
 * 
 * IMPORTANT: This task is designed to use the i2c bus. Non other tasks should not use the i2c bus
 * while this task is running, as it may cause data interruption.
 */
extern "C" void imu_data_logging_task(void *pvParameters);

#endif // IMU_DATA_LOGGER_TASK_H