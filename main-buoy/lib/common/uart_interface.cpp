#include "uart_interface.h"
#include "esp_log.h"     
#include "esp_timer.h"   
#include "time.h"        

static const char* TAG_GPS = "GPS";
static const char* TAG_MODBUS = "ModbusRS485";
static const char* TAG_LORA = "LoRa";

/**
 * @brief Global Measurement Data Struct definition for gps data.
 * This struct is used to store the latest GPS data samples.
 * 
 * This struct is protected by a mutex to ensure thread-safe access from the GPS task.
 */
GpsData g_current_gps_data;
SemaphoreHandle_t g_gps_data_mutex = NULL;


// ------------------------------------- GPS Class Method Definitions ------------------------------------- //
GPS::GPS(HardwareSerial& serial_port,  const UART_Config_t& config)
    : _gps_serial(serial_port), 
      _baud_rate(config.baud_rate), 
      _rx_pin(config.rx_pin), 
      _tx_pin(config.tx_pin),  
      _uartn(config.uart_num)
{}

esp_err_t GPS::initialize() {
    ESP_LOGI(TAG_GPS, "Initializing GPS on UART%d, RX:%d, TX:%d at %d baud.",
             _uartn, _rx_pin, _tx_pin, _baud_rate);
             
    _gps_serial.begin(_baud_rate, SERIAL_8N1, _rx_pin, _tx_pin);

    // Create the mutex for global GPS data: protects `g_current_gps_data`.
    g_gps_data_mutex = xSemaphoreCreateMutex();
    if (g_gps_data_mutex == NULL) {
        ESP_LOGE(TAG_GPS, "Failed to create GPS data mutex!");
        return ESP_FAIL;
    }

    // Initialize global GPS data struct to default/invalid values: umeric members are zeroed out & boolean flags set to false.
    memset(&g_current_gps_data, 0, sizeof(GpsData));
    g_current_gps_data.is_location_valid = false;
    g_current_gps_data.is_time_valid = false;

    ESP_LOGI(TAG_GPS, "GPS initialization complete.");
    return ESP_OK;
}

bool GPS::processIncomingSerialData() {
    bool new_sentence_encoded = false;
    while (_gps_serial.available() > 0) {
        if (_gps.encode(_gps_serial.read())) {
            new_sentence_encoded = true; // A complete NMEA sentence was processed
        }
    }
    return new_sentence_encoded;
}

void GPS::updateGlobalGpsData() {

    if (xSemaphoreTake(g_gps_data_mutex, portMAX_DELAY) == pdTRUE) {
        // Update location data
        if (_gps.location.isValid()) {
            g_current_gps_data.latitude = _gps.location.lat();
            g_current_gps_data.longitude = _gps.location.lng();
            g_current_gps_data.is_location_valid = true;
        } else {
            g_current_gps_data.is_location_valid = false;
        }

        // Update time data
        if (_gps.date.isValid() && _gps.time.isValid()) {
            g_current_gps_data.gps_time_struct.tm_year = _gps.date.year() - 1900; // tm_year is years since 1900
            g_current_gps_data.gps_time_struct.tm_mon = _gps.date.month() - 1;  
            g_current_gps_data.gps_time_struct.tm_mday = _gps.date.day();
            g_current_gps_data.gps_time_struct.tm_hour = _gps.time.hour();
            g_current_gps_data.gps_time_struct.tm_min = _gps.time.minute();
            g_current_gps_data.gps_time_struct.tm_sec = _gps.time.second();
            g_current_gps_data.gps_time_struct.tm_isdst = 0; 

            // Convert to epoch milliseconds
            time_t epoch_seconds = mktime(&g_current_gps_data.gps_time_struct);
            if (epoch_seconds != (time_t)-1) { // mktime returns -1 on error
                uint16_t milliseconds_from_centiseconds = _gps.time.centisecond() * 10;
                g_current_gps_data.timestamp_ms = (uint64_t)epoch_seconds * 1000 + milliseconds_from_centiseconds;
            } else {
                g_current_gps_data.timestamp_ms = 0; 
            }

            g_current_gps_data.is_time_valid = true;
        } else {
            g_current_gps_data.is_time_valid = false;
        }

        // Update other useful GPS data fields: ESP32 timestamp
        g_current_gps_data.esp32_timestamp_us = TIMESTAMP_INIT_MS + (esp_timer_get_time() / 1000ULL); 

        xSemaphoreGive(g_gps_data_mutex); // Release mutex
    } else {
        //ESP_LOGE(TAG_GPS, "Failed to acquire mutex for global GPS data update!");
    }
}


// ------------------------------------- ModBus RS-485 Class Method Definitions ------------------------------------- //
ModbusRS485::ModbusRS485(HardwareSerial& serial_port, const UART_Config_t& config)
    : _modbus_serial(serial_port), 
      _baud_rate(config.baud_rate), 
      _rx_pin(config.rx_pin), 
      _tx_pin(config.tx_pin),  
      _uartn(config.uart_num)
{}

esp_err_t ModbusRS485::initialize() {
    ESP_LOGI(TAG_MODBUS, "Initializing Modbus RS-485 on UART%d, RX:%d, TX:%d at %d baud.",
             _uartn, _rx_pin, _tx_pin, _baud_rate);

    // Begin serial communication with the specified baud rate and pins
    _modbus_serial.begin(_baud_rate, SERIAL_8N1, _rx_pin, _tx_pin);
    _node_modbus.begin(_slave_id, _modbus_serial); 

    ESP_LOGI(TAG_MODBUS, "RS-485 Modbus serial initialized on UART%d.", _uartn);
    return ESP_OK;
}

esp_err_t ModbusRS485::readHoldingRegister(uint8_t* wind_direction) {
    uint8_t result = _node_modbus.readInputRegisters(_addr_first_input_register, _read_quantity);
    
    if (result == ModbusMaster::ku8MBSuccess) {
        *wind_direction = _node_modbus.getResponseBuffer(_idx_response_buffer); 
        return ESP_OK; 
    } else {
        ESP_LOGE(TAG_MODBUS, "Modbus read failed with error code: %d", result);
        return ESP_FAIL; 
    }
}

// ------------------------------------- LoRa Class Method Definitions ------------------------------------- //
LoRaModule::LoRaModule(HardwareSerial& serial_port, const UART_Config_t& config)
    : _lora_serial(serial_port),
      _baud_rate(config.baud_rate),
      _rx_pin(config.rx_pin),
      _tx_pin(config.tx_pin),
      _uartn(config.uart_num)
{}

esp_err_t LoRaModule::initialize() {
    ESP_LOGI(TAG_LORA, "Initializing LoRa on UART%d, RX:%d, TX:%d at %d baud.",
             _uartn, _rx_pin, _tx_pin, _baud_rate);

    _lora_serial.begin(_baud_rate, SERIAL_8N1, _rx_pin, _tx_pin);

    ESP_LOGI(TAG_LORA, "LoRa serial initialized on UART%d.", _uartn);
    return ESP_OK;
}


esp_err_t LoRaModule::sendMessageAll(
    PowerDynamicsCycle* power_dynamics_data,
    MediumDynamicsCycle* medium_dynamics_data,
    SlowDynamicsCycle* slow_dynamics_data
){  
    float accel_x = 1.1f;
    float accel_y = 1.2f;
    float accel_z = 1.3f;
    float gyro_x = 2.4f;
    float gyro_y = 2.5f;
    float gyro_z = 2.6f;
    float mag_x = 3.7f;
    float mag_y = 3.8f;
    float mag_z = 3.9f;

    uint64_t timestamp = power_dynamics_data->timestamp[power_dynamics_data->head];

    float avg_battery_voltage = power_dynamics_data->batteryVoltage[power_dynamics_data->head];
    float avg_battery_current = power_dynamics_data->batteryCurrent[power_dynamics_data->head];
    float avg_solar_voltage = power_dynamics_data->solarVoltage[power_dynamics_data->head];
    float avg_solar_current = power_dynamics_data->solarCurrent[power_dynamics_data->head];
    float avg_soc = power_dynamics_data->SoC[power_dynamics_data->head];

    float _temp = medium_dynamics_data->temperature[medium_dynamics_data->head];
    float _humidity = medium_dynamics_data->humidity[medium_dynamics_data->head];
    float _pressure = medium_dynamics_data->pressure[medium_dynamics_data->head];
    float _wind_speed = medium_dynamics_data->windSpeed[medium_dynamics_data->head];
    uint8_t _wind_dir = medium_dynamics_data->windDir[medium_dynamics_data->head]; 
    
    double _latitude = medium_dynamics_data->latitude[medium_dynamics_data->head];
    double _longitude = medium_dynamics_data->longitude[medium_dynamics_data->head];
    
    float _ph = slow_dynamics_data->pH[slow_dynamics_data->head];
    float _sigma = slow_dynamics_data->sigma[slow_dynamics_data->head];
    float _oxygen = slow_dynamics_data->oxygen[slow_dynamics_data->head];
    float _temperature = slow_dynamics_data->temperature[slow_dynamics_data->head];

    static char payload[PAYLOAD_MAX_SIZE];  
    snprintf(payload, sizeof(payload),
        "{"
        "\"ts\":%llu,"
        "\"ta\":%.1f,\"hr\":%.1f,\"pa\":%.1f,"
        "\"ws\":%.1f,\"wd\":%d,"
        "\"tw\":%.1f,\"ph\":%.2f,\"ec\":%.1f,\"do\":%.2f,"
        "\"lat\":%.4f,\"lon\":%.4f,"
        "\"a\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"g\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"
        "}",
        timestamp,
        _temp, _humidity, _pressure,
        _wind_speed, _wind_dir,
        _temperature, _ph, _sigma, _oxygen,
        _latitude, _longitude, 
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z
    );

    _lora_serial.println(payload);
    return ESP_OK;
}

esp_err_t LoRaModule::sendMessageFastDyn(IMUDynamicsCycle* fast_dynamics_data) {
    uint64_t timestamp = fast_dynamics_data->timestamp[fast_dynamics_data->head];
    float avg_accel_x = fast_dynamics_data->accelX[fast_dynamics_data->head];
    float avg_accel_y = fast_dynamics_data->accelY[fast_dynamics_data->head];
    float avg_accel_z = fast_dynamics_data->accelZ[fast_dynamics_data->head];
    float avg_gyro_x = fast_dynamics_data->gyroX[fast_dynamics_data->head];
    float avg_gyro_y = fast_dynamics_data->gyroY[fast_dynamics_data->head];
    float avg_gyro_z = fast_dynamics_data->gyroZ[fast_dynamics_data->head];
    float avg_mag_x = fast_dynamics_data->magX[fast_dynamics_data->head];
    float avg_mag_y = fast_dynamics_data->magY[fast_dynamics_data->head];
    float avg_mag_z = fast_dynamics_data->magZ[fast_dynamics_data->head];

    static char payload[PAYLOAD_MAX_SIZE];
    snprintf(payload, sizeof(payload),
        "{"
        "\"ts\":%llu,"
        "\"a\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"g\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
        "\"m\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"
        "}",
        timestamp,
        avg_accel_x, avg_accel_y, avg_accel_z,
        avg_gyro_x, avg_gyro_y, avg_gyro_z,
        avg_mag_x, avg_mag_y, avg_mag_z
    );

    _lora_serial.println(payload);
    return ESP_OK;
}

esp_err_t LoRaModule::sendMessagePowerDyn(PowerDynamicsCycle* power_dynamics_data) {
    uint64_t timestamp = power_dynamics_data->timestamp[power_dynamics_data->head];
    float avg_battery_voltage = power_dynamics_data->batteryVoltage[power_dynamics_data->head];
    float avg_battery_current = power_dynamics_data->batteryCurrent[power_dynamics_data->head];
    float avg_solar_voltage = power_dynamics_data->solarVoltage[power_dynamics_data->head];
    float avg_solar_current = power_dynamics_data->solarCurrent[power_dynamics_data->head];
    float avg_soc = power_dynamics_data->SoC[power_dynamics_data->head];

    static char payload[PAYLOAD_MAX_SIZE];
    snprintf(payload, sizeof(payload),
        "{"
        "\"ts\":%llu,"
        "\"bv\":%.2f,\"bc\":%.2f,\"sv\":%.2f,\"sc\":%.2f,\"soc\":%.2f"
        "}",
        timestamp,
        avg_battery_voltage, avg_battery_current,
        avg_solar_voltage, avg_solar_current,
        avg_soc
    );

    _lora_serial.println(payload);
    return ESP_OK;
}

esp_err_t LoRaModule::sendMessageMediumDyn(MediumDynamicsCycle* medium_dynamics_data) {
    uint64_t timestamp = medium_dynamics_data->timestamp[medium_dynamics_data->head];
    float _temp = medium_dynamics_data->temperature[medium_dynamics_data->head];
    float _humidity = medium_dynamics_data->humidity[medium_dynamics_data->head];
    float _pressure = medium_dynamics_data->pressure[medium_dynamics_data->head];
    float _wind_speed = medium_dynamics_data->windSpeed[medium_dynamics_data->head];
    uint8_t _wind_dir = medium_dynamics_data->windDir[medium_dynamics_data->head];

    double _latitude = medium_dynamics_data->latitude[medium_dynamics_data->head];
    double _longitude = medium_dynamics_data->longitude[medium_dynamics_data->head];

    static char payload[PAYLOAD_MAX_SIZE];
    snprintf(payload, sizeof(payload),
        "{"
        "\"ts\":%llu,"
        "\"t\":%.2f,\"h\":%.2f,\"p\":%.2f,"
        "\"ws\":%.2f,\"wd\":%d,"
        "\"lat\":%.6f,\"lon\":%.6f"
        "}",
        timestamp,
        _temp, _humidity, _pressure,
        _wind_speed, _wind_dir,
        _latitude, _longitude
    );

    _lora_serial.println(payload);
    return ESP_OK;
}

esp_err_t LoRaModule::sendMessageSlowDyn(SlowDynamicsCycle* slow_dynamics_data) {
    uint64_t timestamp = slow_dynamics_data->timestamp[slow_dynamics_data->head];
    float _ph = slow_dynamics_data->pH[slow_dynamics_data->head];
    float _sigma = slow_dynamics_data->sigma[slow_dynamics_data->head];
    float _oxygen = slow_dynamics_data->oxygen[slow_dynamics_data->head];
    float _temperature = slow_dynamics_data->temperature[slow_dynamics_data->head];

    static char payload[PAYLOAD_MAX_SIZE];
    snprintf(payload, sizeof(payload),
        "{"
        "\"ts\":%llu,"
        "\"ph\":%.2f,\"sigma\":%.2f,\"o2\":%.2f,\"t\":%.2f"
        "}",
        timestamp,
        _ph, _sigma, _oxygen, _temperature
    );

    _lora_serial.println(payload);
    return ESP_OK;
}
