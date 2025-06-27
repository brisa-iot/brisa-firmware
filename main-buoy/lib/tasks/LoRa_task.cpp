#include "LoRa_task.h"
#include "esp_log.h"
#include "esp_timer.h"


static const char* TAG = "LoRaTask";

// ------------------------- Global Configurations ------------------------- //
const UART_Config_t LoRa_config = {
    .baud_rate = LORA_BAUD_RATE,
    .rx_pin = RXD0_LORA,
    .tx_pin = TXD0_LORA,
    .uart_num = UART_NUM_LORA
};
// ------------------------- Global Configurations ------------------------- //

esp_err_t lora_tx_task_init(LoRaModule& lora_module, TaskHandle_t& lora_task_handler)
{   
    esp_err_t ret;

    if (!xTaskCreatePinnedToCore(
        lora_tx_data_task, 
        "lora_tx_task", 
        4096, 
        &lora_module, 
        3, 
        &lora_task_handler, 
        1 
    )){
        ret = ESP_FAIL;
        return ret;
    }
    vTaskSuspend(lora_task_handler);

    ret = ESP_OK;
    ESP_LOGI(TAG, "LoRa TX Task created successfully.");
    return ESP_OK;
}


void lora_tx_data_task(void *pvParameters) {
    LoRaModule* lora_module = static_cast<LoRaModule*>(pvParameters);

    esp_err_t send_status; 
    TickType_t xLastWakeTime ; 
    const TickType_t xFrequency = pdMS_TO_TICKS(LORA_TASK_DELAY_MS);
    const TickType_t MUTEX_ACQUIRE_TIMEOUT_MS = pdMS_TO_TICKS(50); 

    xLastWakeTime = xTaskGetTickCount();
    for (;;) {

        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xLastWakeTime = xTaskGetTickCount();
        }

        SemaphoreHandle_t acquired_mutexes[3] = {NULL, NULL, NULL}; 

        PowerDynamicsCycle* power_dynamics_data = NULL;
        MediumDynamicsCycle* medium_dynamics_data = NULL;
        SlowDynamicsCycle* slow_dynamics_data = NULL;

        if (xSemaphoreTake(g_power_monitor_buffer.mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != pdTRUE) {
            ESP_LOGW(TAG, "Could not acquire mutex for Power Dynamics buffer. Skipping LoRa send this cycle.");
            goto end_cycle; 
        }
        acquired_mutexes[0] = g_power_monitor_buffer.mutex;

        if (xSemaphoreTake(g_med_dynamics_buffer.mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != pdTRUE) {
            ESP_LOGW(TAG, "Could not acquire mutex for Medium Dynamics buffer. Skipping LoRa send this cycle.");
            goto end_cycle; 
        }
        acquired_mutexes[1] = g_med_dynamics_buffer.mutex;

        if (xSemaphoreTake(g_slow_dynamics_buffer.mutex, MUTEX_ACQUIRE_TIMEOUT_MS) != pdTRUE) {
            ESP_LOGW(TAG, "Could not acquire mutex for Slow Dynamics buffer. Skipping LoRa send this cycle.");
            goto end_cycle; 
        }
        acquired_mutexes[2] = g_slow_dynamics_buffer.mutex;

        power_dynamics_data = &g_power_monitor_buffer;
        medium_dynamics_data = &g_med_dynamics_buffer;
        slow_dynamics_data = &g_slow_dynamics_buffer;

        send_status = lora_module->sendMessageAll(
            power_dynamics_data,
            medium_dynamics_data,
            slow_dynamics_data
        );

        if (send_status != ESP_OK) {
            // ESP_LOGE(TAG, "Failed to send LoRa message: %s", esp_err_to_name(send_status));
            /**
             * TODO: Handle the error gracefully.
             * For example, you might want to log the error or retry sending the message.
             */
        } 

    end_cycle: 
        bool any_mutex_acquired = false;
        for (int i = 2; i >= 0; --i) {
            if (acquired_mutexes[i] != NULL) { 
                xSemaphoreGive(acquired_mutexes[i]);
                any_mutex_acquired = true;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}