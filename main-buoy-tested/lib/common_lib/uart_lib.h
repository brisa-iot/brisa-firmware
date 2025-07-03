#ifndef UART_LIB_H 
#define UART_LIB_H

#include <stdint.h>
#include <Arduino.h>
#include "TinyGPS++.h"
#include "time.h"
#include <ModbusMaster.h>  
#include "data_structs.h"

// ------------------------------------ NEO-8M ------------------------------------ //
#define GPS_BAUD_RATE   9600          // Baud rate for GPS NEO-8M
#define RXD1            5             // RX --> GPIO5
#define TXD1            18            // TX --> GPIO18  
#define UART_1          1             // UART Interface number for GPS NEO-8M 
#define INIT_TIMESTAMP  1751511036ULL // Initial timestamp for testing (epoch seconds)
// ------------------------------------ NEO-8M ------------------------------------ //

// ------------------------------------ RS485: Vane ------------------------------------ //
#define RS485_BAUD_RATE 4800          // Baud rate for RS485 communication
#define RXD2            16            // RX (RO) --> GPIO16 
#define TXD2            17            // TX (DI) --> GPIO17
#define UART_2          2             // UART Interface number for RS485
#define BUFFER_SIZE     256           // Buffer size for RS485 communication
// ------------------------------------ RS485: Vane ------------------------------------ //


class GPSSensor {
private:

    HardwareSerial gpsSerial;           // Serial port for GPS communication
    TinyGPSPlus gps;                    // GPS library instance 
    uint32_t baudRate;                  // Baud rate for GPS communication
    uint8_t rxPin;                      // RX pin for GPS
    uint8_t txPin;                      // TX pin for GPS

public: 
    /**
     * Constructor for GPSSensor
     * @param baudRate_ Baud rate for GPS communication
     * @param rxPin_    RX pin for GPS
     * @param txPin_    TX pin for GPS
     * @param uartn_    UART interface number for GPS
     */
    GPSSensor(
        uint32_t baudRate_ = GPS_BAUD_RATE, 
        uint8_t rxPin_ = RXD1, 
        uint8_t txPin_ = TXD1, 
        uint8_t uartn_ = UART_1
    ) : gpsSerial(uartn_), 
        baudRate(baudRate_), 
        rxPin(rxPin_), 
        txPin(txPin_) {}

    void initialize(); 
    void get_location(SensorData* data);
    void get_gps_time(SensorData* data); 

public:
    double latitude;                    // Latitude value
    double longitude;                   // Longitude value
    struct tm gpsTime;                  // Real-time clock time structure
    uint64_t timestampMs;               // Epoch seconds from GPS time

    int status;                         // Status of the GPS initialization
}; 


class RS485 {
private:
    HardwareSerial rs485Serial;         // Serial port for RS485 communication
    ModbusMaster node;                  // Node to manage Modbus
    uint32_t baudRate;                  // Baud rate for RS485 communication
    uint8_t rxPin;                      // RX pin for RS485
    uint8_t txPin;                      // TX pin for RS485


public:
    /**
     * Constructor for RS485
     * @param baudRate_ Baud rate for RS485 communication
     * @param rxPin_    RX pin for RS485
     * @param txPin_    TX pin for RS485
     * @param uartn_    UART interface number for RS485
     */
    RS485(
        uint32_t baudRate_ = RS485_BAUD_RATE, 
        uint8_t rxPin_ = RXD2, 
        uint8_t txPin_ = TXD2, 
        uint8_t uartn_ = UART_2
    ) : rs485Serial(uartn_), 
        baudRate(baudRate_), 
        rxPin(rxPin_), 
        txPin(txPin_) {}


    void initialize();
    void get_data();
    int get_value();

public: 
    char buffer[BUFFER_SIZE];           // Buffer for RS485 data
    char dataChar;                      // Character for RS485 data
    uint8_t bufferIdx;                  // Index for the buffer
    uint8_t windDirGear;                 // Value of de wind

    int status;                         // Status of the RS485 initialization
};



#endif // UART_LIB_H