#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include <stdint.h>
#include <Arduino.h>  
#include "freertos/FreeRTOS.h"       
#include "freertos/semphr.h"        
#include "TinyGPS++.h"
#include "ModbusMaster.h"
#include "time.h"          
#include "esp_err.h"  
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


// ------------------------------------ Classes: Sensors ------------------------------------ //
// --- NEO-8M GPS: Class Definition ---
class GPS {
private:
    HardwareSerial& _gps_serial;         // Serial port for GPS communication
    TinyGPSPlus _gps;                    // GPS library instance
    uint32_t _baud_rate;                 // Baud rate for GPS communication
    uint8_t _rx_pin;                     // RX pin for GPS
    uint8_t _tx_pin;                     // TX pin for GPS
    uint8_t _uartn;                      // UART interface number for GPS

public:
    /**
     * @brief Constructor for GPS sensor.
     * @param config The configuration struct for the GPS sensor.
     */
    GPS(HardwareSerial& serial_port, const UART_Config_t& config); 

    /**
     * @brief Initializes the GPS serial communication and shared data mutex.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t initialize();

    /**
     * @brief Continuously processes incoming serial data for TinyGPS++.
     * This method should be called repeatedly in a loop (e.g., within a FreeRTOS task).
     * It decodes NMEA sentences and updates the internal TinyGPS++ object.
     * @return true if a complete NMEA sentence was decoded, false otherwise.
     */
    bool processIncomingSerialData();

    /**
     * @brief Updates the global g_current_gps_data structure with the latest
     * information from the TinyGPS++ parser, ensuring thread safety.
     * This should be called *after* processIncomingSerialData() indicates
     * new data is available.
     */
    void updateGlobalGpsData();
};


// --- RS-485 Modbus: Class Definition ---
class ModbusRS485 {
private: 
    HardwareSerial& _modbus_serial; // Serial port for Modbus communication
    ModbusMaster _node_modbus;      // Modbus library instance
    uint32_t _baud_rate;            // Baud rate for Modbus communication
    uint8_t _rx_pin;                // RX pin for Modbus
    uint8_t _tx_pin;                // TX pin for Modbus
    uint8_t _uartn;                 // UART interface number for Modbus

    uint8_t _slave_id = 1;                        // Modbus slave ID (default is 1)
    uint16_t _addr_first_input_register = 0x0000; // First input register address for Modbus
    uint8_t _read_quantity = 0x01;                // Number of registers to read in a single Modbus request
    uint8_t _idx_response_buffer = 0;             // Index for the response buffer

public:
    /**
     * @brief Constructor for Modbus RS-485 sensor.
     * @param config The configuration struct for the Modbus sensor.
     */
    ModbusRS485(HardwareSerial& serial_port, const UART_Config_t& config); 

    /**
     * @brief Initializes the Modbus serial communication.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t initialize();

    /**
     * @brief Reads a holding register from the Modbus slave device.
     * @param wind_direction Pointer to a uint8_t where the wind direction value will be stored.
     * @return The value read from the holding register, or -1 on error.
     */
    esp_err_t readHoldingRegister(uint8_t* wind_direction);
};
// ------------------------------------ Classes: Sensors ------------------------------------ //

// ------------------------------------ Classes: LoRA Tx/Rx ------------------------------------ //
// --- LoRa: Class Definition ---
class LoRaModule {
private:
    HardwareSerial& _lora_serial; // Serial port for LoRa communication
    uint32_t _baud_rate;           // Baud rate for LoRa communication
    uint8_t _rx_pin;               // RX pin for LoRa
    uint8_t _tx_pin;               // TX pin for LoRa
    uint8_t _uartn;                // UART interface number for LoRa

public:
    /**
     * @brief Constructor for LoRa communication.
     * @param config The configuration struct for the LoRa module.
     */
    LoRaModule(HardwareSerial& serial_port, const UART_Config_t& config); 

    /**
     * @brief Initializes the LoRa serial communication.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t initialize();

    /**
     * @brief Sends a message via LoRa.
     * @param fast_dynamics_data Pointer to the FastDynamicsCycle data structure.
     * @param power_dynamics_data Pointer to the PowerDynamicsCycle data structure.
     * @param medium_dynamics_data Pointer to the MediumDynamicsCycle data structure.
     * @param slow_dynamics_data Pointer to the SlowDynamicsCycle data structure.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t sendMessageAll(
        PowerDynamicsCycle* power_dynamics_data,
        MediumDynamicsCycle* medium_dynamics_data,
        SlowDynamicsCycle* slow_dynamics_data
    );

    /**
     * @brief Sends a message containing the FastDynamicsCycle data via LoRa.
     * @param fast_dynamics_data Pointer to the FastDynamicsCycle data structure.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t sendMessageFastDyn(IMUDynamicsCycle* fast_dynamics_data);

    /**
     * @brief Sends a message containing the PowerDynamicsCycle data via LoRa.
     * @param power_dynamics_data Pointer to the PowerDynamicsCycle data structure.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t sendMessagePowerDyn(PowerDynamicsCycle* power_dynamics_data);

    /**
     * @brief Sends a message containing the MediumDynamicsCycle data via LoRa.
     * @param medium_dynamics_data Pointer to the MediumDynamicsCycle data structure.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t sendMessageMediumDyn(MediumDynamicsCycle* medium_dynamics_data);

    /**
     * @brief Sends a message containing the SlowDynamicsCycle data via LoRa.
     * @param slow_dynamics_data Pointer to the SlowDynamicsCycle data structure.
     * @return ESP_OK on success, an error code on failure.
     */
    esp_err_t sendMessageSlowDyn(SlowDynamicsCycle* slow_dynamics_data);

    /**
     * @brief Receives a message via LoRa.
     * @param buffer Pointer to a buffer where the received message will be stored.
     * @param buffer_size Size of the buffer.
     * @return The number of bytes received, or -1 on error.
     */
    int receiveMessage(char* buffer, size_t buffer_size);
};
// ------------------------------------ Classes: LoRA Tx/Rx ------------------------------------ //

#endif // UART_INTERFACE_H