#include "_dynamics_task.h"
#include "esp_log.h"      
#include "esp_timer.h"    


static const char* TAG = "DynamicsTask";

// ------------------------- Global Configurations -------------------------
/**
 * Power Monitor system configuration for INA219.
 */
const INA219_Config_t ina219_config = {
    .i2c_address_battery = INA219_ADDR1,
    .i2c_address_solar = INA219_ADDR2,
    .capacity_mah = CAPACITY,
    .dt_soc_ms = SAMPLING_TIME_SOC_MS
};

/**
 * Medium Dynamics system configuration for BME280 and Modbus.
 */
const BME280_Config_t bme280_config = {
    .i2c_address = BME280_ADDR,
    .pressure_hPa_scale = BME280_PRESSURE_HPA_SCALE
};
const ADC_Config_t anemometer_config = {
    .adc_channel = ADC_ANEMOMETER_CHANNEL,
    .adc_atten = ADC_ANEMOMETER_ATTEN,
    .scale = ADC_ANEMOMETER_SCALE,
    .offset = ADC_ANEMOMETER_OFFSET
};
const UART_Config_t modbus_config = {
    .baud_rate = MODBUS_RS485_BAUD_RATE,
    .rx_pin = RDX2_MODBUS_RS485,
    .tx_pin = TXD2_MODBUS_RS485,
    .uart_num = UART_NUM_MODBUS_RS485
};
// ------------------------- Global Configurations -------------------------

/**
 * @brief Global Measurement Cycle Buffer definition for: 
 * 1. Power Monitor data.
 * 2. Medium Dynamics data.
 * This buffer is used to store dynamic data samples during the measurement cycle.
 * It is protected by a mutex to ensure thread-safe access from the fast dynamics task.
 */
PowerDynamicsCycle g_power_monitor_buffer;
MediumDynamicsCycle g_med_dynamics_buffer;


esp_err_t _dynamics_task_init(DynamicsTaskWrappers_t& dynamics_task_wrapper, TaskHandle_t& dynamics_task_handler)
{   
    esp_err_t ret; 

    g_power_monitor_buffer.mutex = xSemaphoreCreateMutex();
    g_med_dynamics_buffer.mutex = xSemaphoreCreateMutex();
    if (g_power_monitor_buffer.mutex == NULL || g_med_dynamics_buffer.mutex == NULL) {
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to create mutex for Power or Medium Dynamics buffers.");
        return ret; 
    }

    if(!xTaskCreatePinnedToCore(
        _data_acquisition_tasks,
        "_dynamics_data_acquisition_task",
        4096,
        &dynamics_task_wrapper,
        3,
        &dynamics_task_handler,
        0
    )) {
        ret = ESP_FAIL;
        vSemaphoreDelete(g_power_monitor_buffer.mutex);
        vSemaphoreDelete(g_med_dynamics_buffer.mutex);
        return ret;
    }
    vTaskSuspend(dynamics_task_handler); 

    ret = ESP_OK;
    ESP_LOGI(TAG, "Dynamics Task created successfully.");
    return ret;
}


void _data_acquisition_tasks(void *pvParameters) {
    DynamicsTaskWrappers_t* p_wrappers = static_cast<DynamicsTaskWrappers_t*>(pvParameters);

    PowerMonitor* power_monitor = p_wrappers->power_monitor_ptr;
    MediumDynamicsWrapper* med_dyn_wrapper = p_wrappers->med_dyn_wrapper_ptr;

    // --- Define Sampling Frequencies and Counters ---
    TickType_t xLastWakeTime; 
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_TIME_DYNAMICS); 

    PowerDynamics current_power_sample;
    // Accumulators for SAMPLES_TO_AVERAGE_POWER raw samples
    float sum_batteryVoltage = 0.0f; float sum_batteryCurrent = 0.0f;
    float sum_solarVoltage = 0.0f; float sum_solarCurrent = 0.0f;
    float sum_SoC = 0.0f;

    MediumDynamics current_medium_sample;
    // Accumulators for SAMPLES_TO_AVERAGE_MEDIUM raw samples
    float sum_temperature = 0.0f;
    float sum_humidity = 0.0f;
    float sum_pressure = 0.0f;
    float sum_windSpeed = 0.0f;

    // Shared timestamp variables
    size_t raw_sample_count = 0;
    uint64_t current_timestamp_ms = 0;

    xLastWakeTime = xTaskGetTickCount();
    for (;;){

        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            xLastWakeTime = xTaskGetTickCount();
        }
        /**
         * Power Monitor Data Acquisition -------------------------------------------------------------
         */

        if ((power_monitor->readSingle(&current_power_sample) == ESP_OK) ||
            (med_dyn_wrapper->readSingle(&current_medium_sample) == ESP_OK)) {

            sum_batteryVoltage += current_power_sample.batteryVoltage; 
            sum_batteryCurrent += current_power_sample.batteryCurrent;
            sum_solarVoltage += current_power_sample.solarVoltage; 
            sum_solarCurrent += current_power_sample.solarCurrent;
            sum_SoC += current_power_sample.SoC;
            sum_temperature += current_medium_sample.temperature;
            sum_humidity += current_medium_sample.humidity;
            sum_pressure += current_medium_sample.pressure;
            sum_windSpeed += current_medium_sample.windSpeed;
            raw_sample_count++;

            if (raw_sample_count >= SAMPLES_TO_AVERAGE_DYNAMICS) {
                float num_samples_inv = 1.0f / raw_sample_count;
                // --- Retrieve timestamp AFTER averaging is completed --- //
                if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {                 
                    if (g_current_gps_data.is_location_valid || g_current_gps_data.is_time_valid) {
                        current_timestamp_ms = g_current_gps_data.timestamp_ms;
                    } else {
                        current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL) ;
                    }
                    xSemaphoreGive(g_gps_data_mutex);
                } else {
                    current_timestamp_ms = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL); 
                }
                // --- END NEW TIMESTAMP RETRIEVAL --- //

                // --- Store averaged results into the global PowerDynamicsCycle buffer ---
                if (xSemaphoreTake(g_power_monitor_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    g_power_monitor_buffer.batteryVoltage[g_power_monitor_buffer.head] = sum_batteryVoltage * num_samples_inv;
                    g_power_monitor_buffer.batteryCurrent[g_power_monitor_buffer.head] = sum_batteryCurrent * num_samples_inv;
                    g_power_monitor_buffer.solarVoltage[g_power_monitor_buffer.head] = sum_solarVoltage * num_samples_inv;
                    g_power_monitor_buffer.solarCurrent[g_power_monitor_buffer.head] = sum_solarCurrent * num_samples_inv;
                    g_power_monitor_buffer.SoC[g_power_monitor_buffer.head] = sum_SoC * num_samples_inv;
                    g_power_monitor_buffer.timestamp[g_power_monitor_buffer.head] = current_timestamp_ms; 

                    /**
                     * UNCOMMENT THIS BLOCK TO ENABLE LOGGING
                     */
                    // ESP_LOGI("PowerMonAvgTask", "Battery Voltage Avg: %.2f V, Current Avg: %.2f mA, Solar Voltage Avg: %.2f V, Current Avg: %.2f mA, SoC Avg: %.2f%%",
                    //        g_power_monitor_buffer.batteryVoltage[g_power_monitor_buffer.head],
                    //        g_power_monitor_buffer.batteryCurrent[g_power_monitor_buffer.head],
                    //        g_power_monitor_buffer.solarVoltage[g_power_monitor_buffer.head],
                    //        g_power_monitor_buffer.solarCurrent[g_power_monitor_buffer.head],
                    //        g_power_monitor_buffer.SoC[g_power_monitor_buffer.head]);
                    /**
                     * END UNCOMMENT BLOCK
                     */

                    g_power_monitor_buffer.head = (g_power_monitor_buffer.head + 1) % MAX_SAMPLES_POWER_DYNAMICS;
                    if (g_power_monitor_buffer.count < MAX_SAMPLES_POWER_DYNAMICS) {
                        g_power_monitor_buffer.count++;
                    } else {
                        g_power_monitor_buffer.tail = (g_power_monitor_buffer.tail + 1) % MAX_SAMPLES_POWER_DYNAMICS;
                    }
                    xSemaphoreGive(g_power_monitor_buffer.mutex);
                } else {
                    //ESP_LOGE("FastAvgTask", "Failed to acquire mutex for averaged buffer writing!");
                }

                if (xSemaphoreTake(g_med_dynamics_buffer.mutex, portMAX_DELAY) == pdTRUE) {
                    g_med_dynamics_buffer.humidity[g_med_dynamics_buffer.head] = current_medium_sample.humidity; 
                    g_med_dynamics_buffer.pressure[g_med_dynamics_buffer.head] = current_medium_sample.pressure;
                    g_med_dynamics_buffer.temperature[g_med_dynamics_buffer.head] = current_medium_sample.temperature;
                    g_med_dynamics_buffer.windSpeed[g_med_dynamics_buffer.head] = current_medium_sample.windSpeed;
                    g_med_dynamics_buffer.windDir[g_med_dynamics_buffer.head] = current_medium_sample.windDir;
                    g_med_dynamics_buffer.latitude[g_med_dynamics_buffer.head] = current_medium_sample.latitude;
                    g_med_dynamics_buffer.longitude[g_med_dynamics_buffer.head] = current_medium_sample.longitude;
                    g_med_dynamics_buffer.timestamp[g_med_dynamics_buffer.head] = current_timestamp_ms; 

                    /**
                     * UNCOMMENT THIS BLOCK TO ENABLE LOGGING
                     */
                    // ESP_LOGI("MedDynTask", "Temperature: %.2f °C, Humidity: %.2f %, Pressure: %.2f hPa, WinSpeed: %.2f m/s, WindDir: %d, Lat: %.6f, Lon: %.6f, Timestamp: %llu",
                    //        g_med_dynamics_buffer.temperature[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.humidity[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.pressure[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.windSpeed[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.windDir[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.latitude[g_med_dynamics_buffer.head],
                    //        g_med_dynamics_buffer.longitude[g_med_dynamics_buffer.head], 
                    //        g_med_dynamics_buffer.timestamp[g_med_dynamics_buffer.head]);   
                    /**
                     * END UNCOMMENT BLOCK
                     */
                    g_med_dynamics_buffer.head = (g_med_dynamics_buffer.head + 1) % MAX_SAMPLES_MEDIUM_DYNAMICS;
                    if (g_med_dynamics_buffer.count < MAX_SAMPLES_MEDIUM_DYNAMICS) {
                        g_med_dynamics_buffer.count++;
                    } else {
                        g_med_dynamics_buffer.tail = (g_med_dynamics_buffer.tail + 1) % MAX_SAMPLES_MEDIUM_DYNAMICS;
                    }
                    xSemaphoreGive(g_med_dynamics_buffer.mutex); // Release mutex
                } else {
                    //ESP_LOGE("MedDynTask", "Failed to acquire mutex for medium dynamics buffer writing!");
                }
                // --- Reset accumulators and counter for the next averaging cycle ---
                sum_batteryVoltage = 0.0f;
                sum_batteryCurrent = 0.0f;
                sum_solarVoltage = 0.0f;
                sum_solarCurrent = 0.0f;
                sum_SoC = 0.0f;
                sum_temperature = 0.0f;
                sum_humidity = 0.0f;
                sum_pressure = 0.0f;
                sum_windSpeed = 0.0f;
                raw_sample_count = 0;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}