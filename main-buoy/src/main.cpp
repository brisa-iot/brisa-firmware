#include <Arduino.h>
#include "Wire.h"       
#include "esp_log.h"
#include "_dynamics_task.h"
#include "slow_dynamics_task.h"
#include "gps_task.h"
#include "imu_logger_task.h"
#include "LoRa_task.h"
#include "orchestrator.h"

static const char* ORCHESTRATOR_TAG = "Orchestrator";
static const char* TAG = "MAIN"; 

TwoWire i2c_bus_0(0); 

Orchestrator orchestratorSystem(
    &i2c_bus_0,
    Serial, Serial1, Serial2,
    gps_config, imu_config, ina219_config, bme280_config, anemometer_config, modbus_config,
    ph_config, conductivity_config, dissolved_o2_config, temperature_config,
    LoRa_config
); 


void orchestrator_task_wrapper(void* pvParameters) {
    esp_err_t ret; 

    Orchestrator* myOrchestrator = static_cast<Orchestrator*>(pvParameters);

    ESP_LOGI(ORCHESTRATOR_TAG, "Starting i2c bus on SDA: %d, SCL: %d", I2C_SDA_PIN, I2C_SCL_PIN);
    i2c_bus_0.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    ret = myOrchestrator->initialize_wakeups();
    if (ret != ESP_OK) {
        ESP_LOGE(ORCHESTRATOR_TAG, "Failed to initialize wakeups! Error: %s. Deleting task.", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }
    ret = myOrchestrator->initialize_tasks();
    if (ret != ESP_OK) {
        ESP_LOGE(ORCHESTRATOR_TAG, "Failed to initialize tasks! Error: %s. Deleting task.", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }

    for (;;) {
        myOrchestrator->run_sleep_cycle();
    }
    vTaskDelete(NULL);
}



void setup() {
    ESP_LOGI(TAG, "Starting setup.");

    if (!xTaskCreatePinnedToCore(
        orchestrator_task_wrapper,
        "OrchestratorTask",
        4096,
        &orchestratorSystem,
        1,
        NULL,
        1
    )) {
        ESP_LOGE(TAG, "Failed to create Orchestrator task! Halting.");
        while(1) { vTaskDelay(pdMS_TO_TICKS(10)); }
    }
}


void loop() {
    /**
     * Empty loop function. 
     */
}
