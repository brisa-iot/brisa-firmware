#include "gps_task.h"      
#include "esp_log.h"      
#include "esp_timer.h"     


static const char* TAG = "GPSTask"; 


// ------------------------- Global Configurations -------------------------
const UART_Config_t gps_config = {
    .baud_rate = GPS_BAUD_RATE,
    .rx_pin = RXD1_GPS,
    .tx_pin = TXD1_GPS,
    .uart_num = UART_NUM_GPS
};
// ------------------------- Global Configurations -------------------------

esp_err_t gps_task_init(GPS& gps_sensor_instance, TaskHandle_t& gps_task_handler)
{   
    esp_err_t ret;

    if (!xTaskCreatePinnedToCore(
        gps_data_task,                   
        "gps_data_task",                 
        4096,                             
        &gps_sensor_instance,             
        2,                                
        &gps_task_handler,                
        0                   
    )) {
        ret = ESP_FAIL;
        return ret;
    }
    vTaskSuspend(gps_task_handler); 

    ret = ESP_OK;
    ESP_LOGI(TAG, "GPS Task created successfully.");
    return ret; 
}


void gps_data_task(void *pvParameters) {
    GPS* gps_sensor = static_cast<GPS*>(pvParameters);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_TASK_DELAY_MS);

    xLastWakeTime = xTaskGetTickCount();
    for (;;) {

        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xLastWakeTime = xTaskGetTickCount();
        }

        bool new_sentence_encoded = gps_sensor->processIncomingSerialData();
        if (new_sentence_encoded) {
            gps_sensor->updateGlobalGpsData();

            /**
             * UNCOMMENT THIS BLOCK TO ENABLE LOGGING
             */
            // if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            //     if (g_current_gps_data.is_location_valid) {
            //         ESP_LOGI(TAG, "GPS FIX! Lat: %.6f, Lng: %.6f",
            //                  g_current_gps_data.latitude,
            //                  g_current_gps_data.longitude);
            //     }
            //     if (g_current_gps_data.is_time_valid) {
            //         ESP_LOGI(TAG, "GPS Time: %04d-%02d-%02d %02d:%02d:%02d UTC (UBLOX RTC: %llu ms) (ESP32: %llu ms)",
            //                 g_current_gps_data.gps_time_struct.tm_year + 1900,
            //                 g_current_gps_data.gps_time_struct.tm_mon + 1,
            //                 g_current_gps_data.gps_time_struct.tm_mday,
            //                 g_current_gps_data.gps_time_struct.tm_hour,
            //                 g_current_gps_data.gps_time_struct.tm_min,
            //                 g_current_gps_data.gps_time_struct.tm_sec,
            //                 g_current_gps_data.timestamp_ms,
            //                 g_current_gps_data.esp32_timestamp_us);
            //     }
            //      if (!g_current_gps_data.is_location_valid && !g_current_gps_data.is_time_valid) {
            //         ESP_LOGW(TAG, "GPS data processed, but no valid fix yet.");
            //      }
            //     xSemaphoreGive(g_gps_data_mutex); // Release the mutex
            // } else {
            //     ESP_LOGW(TAG, "Could not acquire GPS data mutex for logging.");
            // }
            /**
             * END UNCOMMENT BLOCK
             */
        }
        // Delay until the next cycle to maintain the desired frequency
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}