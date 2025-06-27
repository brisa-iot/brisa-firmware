#include "orchestrator.h"

static const char* TAG = "Orchestrator"; 

// --- Constructor ---
Orchestrator::Orchestrator(
    TwoWire* i2c_bus, 
    HardwareSerial& LoRa_serial, HardwareSerial& gps_serial, HardwareSerial& modbus_serial,
    const UART_Config_t& gps_cfg, 
    const IMU_Config_t& imu_cfg,  
    const INA219_Config_t& ina219_cfg,
    const BME280_Config_t& bme280_cfg, const ADC_Config_t& anem_cfg, const UART_Config_t& modbus_cfg,
    const ADC_Config_t& ph_cfg, const ADC_Config_t& cond_cfg, const ADC_Config_t& do_cfg,
    const DS18B20_Config_t& temp_cfg, 
    const UART_Config_t& LoRa_cfg
)
    : _LoRa_serial(LoRa_serial),                                  
      _gps_serial(gps_serial),                                    
      _modbus_serial(modbus_serial),                              
      _gps_config(gps_cfg),                                       
      _imu_config(imu_cfg),                                       
      _anemometer_config(anem_cfg),                               
      _ina219_config(ina219_cfg),                                 
      _bme280_config(bme280_cfg),                                 
      _modbus_config(modbus_cfg),                                 
      _ph_config(ph_cfg),                                         
      _conductivity_config(cond_cfg),                             
      _dissolved_o2_config(do_cfg),                               
      _temperature_config(temp_cfg),                              
      _LoRa_config(LoRa_cfg),                                     
      _gps_sensor(gps_serial, gps_cfg),                           
      _fast_dynamics_wrapper(i2c_bus, imu_cfg),                   
      _power_monitor(i2c_bus, ina219_cfg),                    
      _medium_dynamics_wrapper(i2c_bus, modbus_serial, bme280_cfg, anem_cfg, modbus_cfg), 
      _slow_dynamics_wrapper(ph_cfg, cond_cfg, do_cfg, temp_cfg),
      _lora_module(LoRa_serial, LoRa_cfg)                         
{}

// --- Initialization Method ---
esp_err_t Orchestrator::initialize_wakeups(){
    esp_err_t ret;

    ret = esp_sleep_enable_timer_wakeup(MEASUREMENT_INTERVAL_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configure timer as wakeup source failed: %s", esp_err_to_name(ret));
        return ret;
    }
    /**
     * LoRa UART wakeup configuration:
     * - Set the wakeup threshold to 3 characters.
     */
    ret = uart_set_wakeup_threshold(UART_NUM_LORA, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configure GPIO as wakeup source failed: %s", esp_err_to_name(ret));
        return ret;
    }
    /**
     * LoRa UART wakeup configuration:
     * - Enable UART wakeup source.
     */
    ret = esp_sleep_enable_uart_wakeup(UART_NUM_LORA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Configure UART as wakeup source failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Timer-UART wakeup source is ready");

    // Configure GPIO for switch load control
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;        
    io_conf.mode = GPIO_MODE_OUTPUT;             
    io_conf.pin_bit_mask = (1ULL << SWITCH_LOAD_PIN); 
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; 
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     
    gpio_config(&io_conf);                        

    // Define initial wakeup cause
    _wakeup_cause = ESP_SLEEP_WAKEUP_TIMER;

    return ESP_OK;
}


esp_err_t Orchestrator::initialize_sensors() {
    esp_err_t ret; 

    // Initialize Fast Dynamics Wrapper (IMU)
    ret = _fast_dynamics_wrapper.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fast Dynamics Wrapper initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));
    // 1. Initialize GPS dynamics: 
    ret = _gps_sensor.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS sensor initialization failed! %s", esp_err_to_name(ret));
        return ret; 
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS)); 
    // 2. Initialize Medium Dynamics Wrapper (BME280, Modbus, Anemometer)
    ret = _medium_dynamics_wrapper.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Medium Dynamics Wrapper initialization failed! %s", esp_err_to_name(ret));
        return ret; 
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));
    // 3. Initialize Power Monitor (INA219)
    ret = _power_monitor.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power Monitor initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));
    // 4. Initialize Slow Dynamics Wrapper (pH, Conductivity, Dissolved O2, Temperature)
    ret = _slow_dynamics_wrapper.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Slow Dynamics Wrapper initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));

    ESP_LOGI(TAG, "All sensors initialized successfully.");
    return ESP_OK; 
}


esp_err_t Orchestrator::initialize_LoRa() {
    esp_err_t ret;

    // Initialize LoRa Module: Tx/Rx
    ret = _lora_module.initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa Module initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));

    return ESP_OK;
}


esp_err_t Orchestrator::initialize_tasks() {
    ESP_LOGI(TAG, "Initializing Orchestrator and system components.");
    esp_err_t ret;

    // --------------------- Sensor & LoRa Module Initialization ---------------------
    // ESP_LOGI(TAG, "------------------- First Initialization -------------------");
    // ret = initialize_sensors(); 
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Sensor initialization failed! %s", esp_err_to_name(ret));
    //     return ret; 
    // }
    // vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));
    ret = initialize_LoRa();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_INITIALIZATION_DELAY_MS));
    // ESP_LOGI(TAG, "-------------- First Initialization Complete ---------------");
    // --------------------- Sensor & LoRa Module Initialization ---------------------

    // Initialize GPS task
    ret = gps_task_init(_gps_sensor, _gps_task_handle);   
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS task initialization failed! %s", esp_err_to_name(ret));
        return ret; 
    }
    // Initialize Dynamics task
    _dynamics_task_wrapper.power_monitor_ptr = &_power_monitor; 
    _dynamics_task_wrapper.med_dyn_wrapper_ptr = &_medium_dynamics_wrapper; 
    ret = _dynamics_task_init(_dynamics_task_wrapper, _dynamics_task_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Dynamics task initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    // Initialize Slow Dynamics task
    ret = slow_dynamics_task_init(_slow_dynamics_wrapper, _slow_dynamics_task_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Slow Dynamics task initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }
    // Initialize IMU Data Logger task
    // ret = imu_logger_task_init(_fast_dynamics_wrapper, _imu_logging_task_handle);
    // if (ret != ESP_OK) {
    //     // ESP_LOGE(TAG, "IMU Data Logger task initialization failed! %s", esp_err_to_name(ret));
    //     vSemaphoreDelete(g_imu_data_logger_buffer.mutex);
    //     return ret;
    // }
    // Initialize LoRa TX task
    ret = lora_tx_task_init(_lora_module, _lora_tx_task_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa TX task initialization failed! %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize LoRa RX task
    /**
     * TODO: Implement LoRa RX task initialization.
     */

    ESP_LOGI(TAG, "Orchestrator initialization complete. All tasks and sensors initialized. Tasks initially suspended.");
    return ret;
}



// --- Sensor Power Control (Placeholders) ---
void Orchestrator::power_on_sensors() {
    gpio_set_level(SWITCH_LOAD_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(POWER_UP_DELAY_MS)); 
}

void Orchestrator::power_off_sensors() {
    gpio_set_level(SWITCH_LOAD_PIN, 0);
}


// --- Measurement Window Control ---
void Orchestrator::start_measurement_window() {
    //ESP_LOGI(TAG, "--- Starting Measurement Window (for %lu minutes) ---", MEASUREMENT_WINDOW_MINUTES);
    esp_err_t ret; 

    ret = initialize_sensors();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed! %s", esp_err_to_name(ret));
        /**
         * TODO: Handle the error gracefully.
         * For example, you might want to log the error, notify the user, or retry
         */
        return; 
    }

    if (_gps_task_handle != NULL) {
        vTaskResume(_gps_task_handle);
        xTaskNotifyGive(_gps_task_handle);
        ESP_LOGI(TAG, "GPS Task Resumed.");
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 
    if (_dynamics_task_handle != NULL) {
        vTaskResume(_dynamics_task_handle);
        xTaskNotifyGive(_dynamics_task_handle);
        ESP_LOGI(TAG, "Dynamics Task Resumed.");
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 
    if (_slow_dynamics_task_handle != NULL) {
        vTaskResume(_slow_dynamics_task_handle);
        xTaskNotifyGive(_slow_dynamics_task_handle);
        ESP_LOGI(TAG, "Slow Dynamics Task Resumed.");
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 
    if (_lora_tx_task_handle != NULL) {
        vTaskResume(_lora_tx_task_handle);
        xTaskNotifyGive(_lora_tx_task_handle); 
        ESP_LOGI(TAG, "LoRa Task Resumed.");
    }
    vTaskDelay(pdMS_TO_TICKS(1)); 

    ESP_LOGI(TAG, "Measurement window started. All tasks resumed and sensors powered ON.");
}


void Orchestrator::stop_measurement_window() {
    ESP_LOGI(TAG, "--- Stopping Measurement Window ---");

    if (_gps_task_handle != NULL) {
        vTaskSuspend(_gps_task_handle);
        ESP_LOGI(TAG, "GPS Task Suspended.");
    }
    if (_dynamics_task_handle != NULL) {
        vTaskSuspend(_dynamics_task_handle);
        ESP_LOGI(TAG, "Dynamics Task Suspended.");
    }
    if (_slow_dynamics_task_handle != NULL) {
        vTaskSuspend(_slow_dynamics_task_handle);
        ESP_LOGI(TAG, "Slow Dynamics Task Suspended.");
    }
    if (_lora_tx_task_handle != NULL) {
        vTaskSuspend(_lora_tx_task_handle);
        ESP_LOGI(TAG, "LoRa Task Suspended.");
    }

    // ------------------------------------ Resume the IMU logging task: ------------------------------------
    /**
     * Perform data logging over MEASUREMENT_WINDOW_IMU_MS milliseconds.
     */
    // if (_imu_logging_task_handle != NULL) {
    //     vTaskResume(_imu_logging_task_handle);
    //     xTaskNotifyGive(_imu_logging_task_handle);
    //     //ESP_LOGI(TAG, "IMU Task Resumed for logging.");
    // }

    // vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_WINDOW_IMU_MS)); 

    // if (_imu_logging_task_handle != NULL) {
    //     vTaskSuspend(_imu_logging_task_handle);
    //     //ESP_LOGI(TAG, "IMU Task Suspended after logging.");
    // }
    // vTaskDelay(pdMS_TO_TICKS(1)); 
    // ------------------------------------ Resume the IMU logging task: ------------------------------------
}


// --- Main Sleep Cycle Orchestration Method ---
void Orchestrator::run_sleep_cycle() {

    ESP_LOGI(TAG, "Starting sleep cycle. Configuring wakeup sources...");
    switch (_wakeup_cause) {

        case ESP_SLEEP_WAKEUP_TIMER:
            ESP_LOGI(TAG, "Woke up from Timer.");

            power_on_sensors();
            start_measurement_window(); 

            // Wait for the duration of the measurement window
            vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_WINDOW_US / 1000));

            stop_measurement_window(); 
            power_off_sensors(); 

            esp_sleep_enable_timer_wakeup(MEASUREMENT_INTERVAL_US); 
            /**
             * Making sure the complete line is printed before entering light sleep.
             * We must wait for the UART TX FIFO (register) to be empty.
             */
            uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM); 
            break;

        case ESP_SLEEP_WAKEUP_UART:
            ESP_LOGI(TAG, "Woke up from UART.");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            ESP_LOGI(TAG, "Woke up from an undefined source.");
            break;
        default:
            ESP_LOGI(TAG, "Woke up from an unknown source: %d", _wakeup_cause);
            break;
    }
    esp_light_sleep_start();
    _wakeup_cause = esp_sleep_get_wakeup_cause();
}

