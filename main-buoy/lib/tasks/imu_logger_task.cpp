#include "imu_logger_task.h"

static const char* TAG = "IMULoggerTask";

// ------------------------- Global Configurations -------------------------
// IMU Data Logger system configuration
const IMU_Config_t imu_config = {
    .i2c_address = MPU9250_ADDRESS,
    .filter_coeffs = imu_filter_coeffs
};
// ------------------------- Global Configurations -------------------------

/**
 * @brief Global context for daily log file management.
 * This context holds the current log file handle and the date part of the filename.
 * It is used to manage the daily log files for IMU data logging.
 * The filename format is "YYYYMMDD_imu_log.csv", where YYYYMMDD is the date in year-month-day format.
 */
static DailyLogFileContext_t g_imu_daily_log_context = {
    .current_log_file_handle = NULL,
    .current_log_filename_date_part = ""
};

/**
 * @brief Global Measurement Cycle Buffer definition for IMU data logging.
 * This buffer is used to store IMU data samples during the measurement cycle.
 * It is protected by a mutex to ensure thread-safe access from the IMU data logger task.
 */
IMUDynamicsCycle g_imu_data_logger_buffer;


esp_err_t imu_logger_task_init(FastDynamicsWrapper& fast_dynamics_wrapper, TaskHandle_t& imu_logging_task_handler)
{
    // 1. Initialize internal filesystem for logging
    esp_err_t fs_init_status = initialize_filesystem();
    if (fs_init_status != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize filesystem for IMU data logging (%s)!", esp_err_to_name(fs_init_status));
        return fs_init_status;
    }

    esp_err_t ret;

    g_imu_data_logger_buffer.mutex = xSemaphoreCreateMutex();
    if (g_imu_data_logger_buffer.mutex == NULL) {
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to create mutex for IMU data logger buffer.");
        return ret; 
    }

    if(!xTaskCreatePinnedToCore(
        imu_data_logging_task,                
        "imu_data_logger_task",                  
        4096,                                  
        &fast_dynamics_wrapper, 
        5,                                     
        &imu_logging_task_handler,             
        0                
    )) {
        ret = ESP_FAIL;
        vSemaphoreDelete(g_imu_data_logger_buffer.mutex);
        return ESP_FAIL;
    }
    vTaskSuspend(imu_logging_task_handler); 

    ret = ESP_OK;
    ESP_LOGI(TAG, "IMU Data Logger Task created successfully.");
    return ret;
}

void imu_data_logging_task(void *pvParameters) {
    FastDynamicsWrapper* fast_dyn_wrapper = static_cast<FastDynamicsWrapper*>(pvParameters);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_IMU_LOGGER_MS);
    const TickType_t BUFFER_MUTEX_ACQUIRE_TIMEOUT_MS = pdMS_TO_TICKS(100); 

    IMUDynamics current_imu_data; 
    float sum_accelX = 0.0f; float sum_accelY = 0.0f; float sum_accelZ = 0.0f;
    float sum_gyroX = 0.0f; float sum_gyroY = 0.0f; float sum_gyroZ = 0.0f;
    float sum_magX = 0.0f; float sum_magY = 0.0f; float sum_magZ = 0.0f;
    size_t imu_raw_samples_count = 0;
    uint64_t imu_current_timestamp_ms = 0;

    size_t test_count = 0; 

    uint32_t flush_and_info_counter = 0;
    const uint32_t FLUSH_INTERVAL_SAMPLES_IMU = FLUSH_INTERVAL_SAMPLES; 

    xLastWakeTime = xTaskGetTickCount();
    for (;;) {

        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xLastWakeTime = xTaskGetTickCount();
        }

        if (fast_dyn_wrapper->readSingle(&current_imu_data) == ESP_OK) {
            sum_accelX += current_imu_data.accelX;
            sum_accelY += current_imu_data.accelY;
            sum_accelZ += current_imu_data.accelZ;
            sum_gyroX += current_imu_data.gyroX;
            sum_gyroY += current_imu_data.gyroY;
            sum_gyroZ += current_imu_data.gyroZ;
            sum_magX += current_imu_data.magX;
            sum_magY += current_imu_data.magY;
            sum_magZ += current_imu_data.magZ;
            imu_raw_samples_count++;

            if (imu_raw_samples_count >= SAMPLES_TO_AVERAGE_IMU) {
                float num_samples_inv = 1.0f / imu_raw_samples_count;
                
                // --- Retrive GPS timestamp whenever reading instantaneous data --- //
                if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {                 
                    if (g_current_gps_data.is_location_valid || g_current_gps_data.is_time_valid) {
                        imu_current_timestamp_ms = g_current_gps_data.timestamp_ms; 
                    } else {
                        imu_current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL);
                    }
                    xSemaphoreGive(g_gps_data_mutex);
                } else {
                    imu_current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL); 
                }
                // --- END NEW TIMESTAMP RETRIEVAL --- //

                std::string current_full_date_str = get_date_string(imu_current_timestamp_ms); // YYYYMMDD_HHMMSS
                std::string current_date_only_str = current_full_date_str.substr(0, 8); // YYYYMMDD

                if (g_imu_daily_log_context.current_log_file_handle == NULL || 
                    g_imu_daily_log_context.current_log_filename_date_part != current_date_only_str) {
                    
                    if (g_imu_daily_log_context.current_log_file_handle != NULL) {
                        //ESP_LOGI(TAG, "New day detected. Closing %s.", g_imu_daily_log_context.current_log_filename_date_part.c_str());
                        fflush(g_imu_daily_log_context.current_log_file_handle); 
                        fclose(g_imu_daily_log_context.current_log_file_handle);
                        g_imu_daily_log_context.current_log_file_handle = NULL; 
                        // Wait for a short period to ensure the file is closed properly before opening a new one:
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }

                    // Update the date part in context:
                    g_imu_daily_log_context.current_log_filename_date_part = current_date_only_str;
                    std::string new_log_filename = std::string(LOGS_DIR) + "/" + current_date_only_str + "_imu_log.csv";
                    
                    // Open the new day's log file:
                    g_imu_daily_log_context.current_log_file_handle = fopen(new_log_filename.c_str(), "a");
                    if (g_imu_daily_log_context.current_log_file_handle == NULL) {
                        //ESP_LOGE(TAG, "Failed to open/create log file: %s", new_log_filename.c_str());
                        /**
                         * TODO: error handling: delay, retry, or switch to no-logging mode
                         */
                    } else {

                        fseek(g_imu_daily_log_context.current_log_file_handle, 0, SEEK_END);
                        if (ftell(g_imu_daily_log_context.current_log_file_handle) == 0) {
                            fprintf(g_imu_daily_log_context.current_log_file_handle, "timestamp_ms," "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z\n");
                            //ESP_LOGI(TAG, "Wrote header to new log file: %s", new_log_filename.c_str());
                        }
                        //ESP_LOGI(TAG, "Opened new daily log file: %s", new_log_filename.c_str());

                        // Perform circular file management and get FS info when a new file is opened:
                        std::vector<std::string> existing_files = get_sorted_log_file_paths(); 
                        if (existing_files.empty()) {
                            //ESP_LOGI(TAG, "No existing log files found in %s", LOGS_DIR);
                        } else {
                            //ESP_LOGI(TAG, "Found %zu existing log files in %s", existing_files.size(), LOGS_DIR);
                        }
                        // Delete oldest files if the number exceeds the maximum allowed:
                        if (existing_files.size() > MAX_NUM_DAYS_LOG_FILES) {
                            std::string oldest_file_path = existing_files[0];
                            //ESP_LOGW(TAG, "Max log files reached (%zu). Deleting oldest file: %s", 
                            //    MAX_NUM_DAYS_LOG_FILES, oldest_file_path.c_str());
                            if (remove(oldest_file_path.c_str()) != 0) {
                                //ESP_LOGE(TAG, "Error deleting old log file %s: %s", oldest_file_path.c_str(), strerror(errno));
                            } else {
                                //ESP_LOGI(TAG, "Successfully deleted old log file: %s", oldest_file_path.c_str());
                            }
                        }
                        
                        /**
                         * UNCOMMENT THIS BLOCK TO ENABLE LITTLEFS INFO LOGGING
                         * Default partition label defined is "storage" for LittleFS
                         */
                        // size_t total_bytes = 0, used_bytes = 0;
                        // esp_err_t ret = esp_littlefs_info("storage", &total_bytes, &used_bytes); 
                        // if (ret != ESP_OK) {
                        //     ESP_LOGE(TAG, "Failed to get littlefs info (%s)", esp_err_to_name(ret));
                        // } else {
                        //     ESP_LOGI(TAG, "Partition size: %zu, used bytes: %zu", total_bytes, used_bytes); 
                        // }
                        /**
                         * END UNCOMMENT BLOCK
                         */
                    }
                }

                if (g_imu_daily_log_context.current_log_file_handle != NULL) { 
                    if (xSemaphoreTake(g_imu_data_logger_buffer.mutex, BUFFER_MUTEX_ACQUIRE_TIMEOUT_MS) == pdTRUE) {

                        g_imu_data_logger_buffer.accelX[g_imu_data_logger_buffer.head] = sum_accelX * num_samples_inv;
                        g_imu_data_logger_buffer.accelY[g_imu_data_logger_buffer.head] = sum_accelY * num_samples_inv;
                        g_imu_data_logger_buffer.accelZ[g_imu_data_logger_buffer.head] = sum_accelZ * num_samples_inv;
                        g_imu_data_logger_buffer.gyroX[g_imu_data_logger_buffer.head] = sum_gyroX * num_samples_inv;
                        g_imu_data_logger_buffer.gyroY[g_imu_data_logger_buffer.head] = sum_gyroY * num_samples_inv;
                        g_imu_data_logger_buffer.gyroZ[g_imu_data_logger_buffer.head] = sum_gyroZ * num_samples_inv;
                        g_imu_data_logger_buffer.magX[g_imu_data_logger_buffer.head] = sum_magX * num_samples_inv;
                        g_imu_data_logger_buffer.magY[g_imu_data_logger_buffer.head] = sum_magY * num_samples_inv;
                        g_imu_data_logger_buffer.magZ[g_imu_data_logger_buffer.head] = sum_magZ * num_samples_inv;
                        g_imu_data_logger_buffer.timestamp[g_imu_data_logger_buffer.head] = imu_current_timestamp_ms;

                        fprintf(
                            g_imu_daily_log_context.current_log_file_handle, 
                            "%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                            g_imu_data_logger_buffer.timestamp[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.accelX[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.accelY[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.accelZ[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.gyroX[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.gyroY[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.gyroZ[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.magX[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.magY[g_imu_data_logger_buffer.head],
                            g_imu_data_logger_buffer.magZ[g_imu_data_logger_buffer.head]
                        ); 
                        
                        g_imu_data_logger_buffer.head = (g_imu_data_logger_buffer.head + 1) % TOTAL_SAMPLES_TO_LOG;
                        if (g_imu_data_logger_buffer.count < TOTAL_SAMPLES_TO_LOG) {
                            g_imu_data_logger_buffer.count++;
                        } else {
                            g_imu_data_logger_buffer.tail = (g_imu_data_logger_buffer.tail + 1) % TOTAL_SAMPLES_TO_LOG;
                        }
                        xSemaphoreGive(g_imu_data_logger_buffer.mutex);
                        
                        /**
                         * UNCOMMENT THIS BLOCK TO ENABLE LOGGING INFO
                         */
                        // ESP_LOGI(TAG, "Logged averaged IMU Data: TS=%llu, Head=%d, Count=%d", 
                        //         g_imu_data_logger_buffer.timestamp[g_imu_data_logger_buffer.head],
                        //         g_imu_data_logger_buffer.head, g_imu_data_logger_buffer.count);
                        /**
                         * END UNCOMMENT BLOCK
                         */
                    } else {
                        //ESP_LOGE(TAG, "Failed to acquire mutex for IMU data logger buffer writing!");
                    }
                } else {
                    //ESP_LOGW(TAG, "Log file not open. Skipping writing IMU data to file.");
                }

                // Periodic fflush to commit data to physical storage
                flush_and_info_counter++;
                if (flush_and_info_counter % FLUSH_INTERVAL_SAMPLES_IMU == 0) {
                    if (g_imu_daily_log_context.current_log_file_handle != NULL) {
                        fflush(g_imu_daily_log_context.current_log_file_handle);
                        //ESP_LOGD(TAG, "Flushed IMU log file.");
                    }
                }
                
                // Reset accumulators and sample count
                sum_accelX = 0.0f; sum_accelY = 0.0f; sum_accelZ = 0.0f;
                sum_gyroX = 0.0f; sum_gyroY = 0.0f; sum_gyroZ = 0.0f;
                sum_magX = 0.0f; sum_magY = 0.0f; sum_magZ = 0.0f;
                imu_raw_samples_count = 0;
            }
        } else {
            //ESP_LOGE(TAG, "Error reading raw IMU data from wrapper.");
        }
        // Delay until the next cycle to maintain the desired frequency
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

