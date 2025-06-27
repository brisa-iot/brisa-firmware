#include "slow_dynamics_task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "SlowDynamicsTask";

// ------------------------- Global Configurations -------------------------
const ADC_Config_t ph_config ={
    .adc_channel = PH_ADC_CHANNEL, 
    .adc_atten = ADC_PH_ATTEN,
    .scale = PH_SCALE,
    .offset = PH_OFFSET
}; 

const ADC_Config_t conductivity_config = {
    .adc_channel = CONDUCTIVITY_ADC_CHANNEL, 
    .adc_atten = ADC_CONDUCTIVITY_ATTEN,
    .scale = CONDUCTIVITY_SCALE,
    .offset = CONDUCTIVITY_OFFSET
};

const ADC_Config_t dissolved_o2_config = {
    .adc_channel = DISSOLVED_O2_ADC_CHANNEL, 
    .adc_atten = ADC_DISSOLVED_O2_ATTEN,
    .scale = DISSOLVED_O2_SCALE,
    .offset = DISSOLVED_O2_OFFSET
};

const DS18B20_Config_t temperature_config = {
    .pin = ONE_WIRE_BUS_PIN,
    .device_idx = DS18B20_DEVICE_IDX,
    .conv_delay_ms = CONVERSION_DELAY
};
// ------------------------- Global Configurations -------------------------


/**
 * @brief Global Measurement Cycle Buffer definition for slow dynamics data.
 * This buffer is used to store slow dynamics data samples during the measurement cycle.
 * It is protected by a mutex to ensure thread-safe access from the slow dynamics task.
 */
SlowDynamicsCycle g_slow_dynamics_buffer;


esp_err_t slow_dynamics_task_init(SlowDynamicsWrapper& slow_dynamics_wrapper, TaskHandle_t& slow_dynamics_task_handler)
{   
    esp_err_t ret;

    g_slow_dynamics_buffer.mutex = xSemaphoreCreateMutex();
    if (g_slow_dynamics_buffer.mutex == NULL) {
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to create mutex for Slow Dynamics buffer.");
        return ret; 
    }

    if (!xTaskCreatePinnedToCore(
        slow_data_acquisition_task,
        "slow_dynamics_task",
        4096,
        &slow_dynamics_wrapper,
        4,
        &slow_dynamics_task_handler,
        0
    )) {
        ret = ESP_FAIL;
        vSemaphoreDelete(g_slow_dynamics_buffer.mutex);
        return ret;
    }
    vTaskSuspend(slow_dynamics_task_handler); // Suspend the task initially

    ret = ESP_OK;
    ESP_LOGI(TAG, "Slow Dynamics Task created successfully.");
    return ESP_OK;
}


void slow_data_acquisition_task(void *pvParameters) {
    SlowDynamicsWrapper* p_wrapper = static_cast<SlowDynamicsWrapper*>(pvParameters);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_SLOW_DYNAMICS_MS); 

    SlowDynamics slow_current_sample; 
    float sum_pH = 0.0f;
    float sum_sigma = 0.0f;
    float sum_oxygen = 0.0f;
    float sum_temperature = 0.0f;
    size_t slow_raw_sample_count = 0; 
    uint64_t slow_current_timestamp_ms = 0; 

    xLastWakeTime = xTaskGetTickCount(); 
    for (;;) {

        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xLastWakeTime = xTaskGetTickCount();
        }

        if (p_wrapper->readSingle(&slow_current_sample) == ESP_OK) {
            sum_pH += slow_current_sample.pH;
            sum_sigma += slow_current_sample.sigma;
            sum_oxygen += slow_current_sample.oxygen;
            sum_temperature += slow_current_sample.temperature;
            slow_raw_sample_count++;

            if (slow_raw_sample_count >= SAMPLES_TO_AVERAGE_SLOW){
                if (slow_raw_sample_count > 0) {
                    float num_samples_inv = 1.0f / slow_raw_sample_count;
                    // --- Retrive GPS timestamp whenever reading instantaneous data --- //
                    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {                
                        if (g_current_gps_data.is_location_valid || g_current_gps_data.is_time_valid) {
                            slow_current_timestamp_ms = g_current_gps_data.timestamp_ms;
                        } else {
                            slow_current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL) ;
                        }
                        xSemaphoreGive(g_gps_data_mutex);
                    } else {
                        slow_current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL); 
                    }
                    // --- END NEW TIMESTAMP RETRIEVAL --- //

                    // --- Store this single sample into the global SlowDynamicsCycle buffer ---
                    if (xSemaphoreTake(g_slow_dynamics_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                        g_slow_dynamics_buffer.pH[g_slow_dynamics_buffer.head] = slow_current_sample.pH; 
                        g_slow_dynamics_buffer.sigma[g_slow_dynamics_buffer.head] = slow_current_sample.sigma;
                        g_slow_dynamics_buffer.oxygen[g_slow_dynamics_buffer.head] = slow_current_sample.oxygen;
                        g_slow_dynamics_buffer.temperature[g_slow_dynamics_buffer.head] = slow_current_sample.temperature;
                        g_slow_dynamics_buffer.timestamp[g_slow_dynamics_buffer.head] = slow_current_timestamp_ms; 

                        /**
                         * UNCOMMENT THIS BLOCK TO ENABLE LOGGING
                         */
                        // ESP_LOGI("SlowDynTask", "pH: %.2f, Conductivity: %.2f mS/cm, Dissolved O2: %.2f mg/L, Temperature: %.2f °C",
                        //     g_slow_dynamics_buffer.pH[g_slow_dynamics_buffer.head],
                        //     g_slow_dynamics_buffer.sigma[g_slow_dynamics_buffer.head],
                        //     g_slow_dynamics_buffer.oxygen[g_slow_dynamics_buffer.head],
                        //     g_slow_dynamics_buffer.temperature[g_slow_dynamics_buffer.head]);
                        /**
                         * END UNCOMMENT BLOCK
                         */

                        g_slow_dynamics_buffer.head = (g_slow_dynamics_buffer.head + 1) % MAX_SAMPLES_SLOW_DYNAMICS;
                        if (g_slow_dynamics_buffer.count < MAX_SAMPLES_SLOW_DYNAMICS) {
                            g_slow_dynamics_buffer.count++;
                        } else {
                            g_slow_dynamics_buffer.tail = (g_slow_dynamics_buffer.tail + 1) % MAX_SAMPLES_SLOW_DYNAMICS;
                        }
                        xSemaphoreGive(g_slow_dynamics_buffer.mutex);
                    } else {
                        //ESP_LOGE("SlowDynTask", "Failed to acquire mutex for slow dynamics buffer writing!");
                    }
                }
            
                // -- Reset the counters and sums for the next cycle --
                slow_raw_sample_count = 0;
                sum_pH = 0.0f;
                sum_sigma = 0.0f;
                sum_oxygen = 0.0f;
                sum_temperature = 0.0f;
            }
        }
        // Delay until the next cycle to maintain the desired frequency
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
