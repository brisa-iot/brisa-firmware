#include <iostream>
#include "uart_lib.h"

// Variables globales para acceder a los pines desde las funciones estáticas
//static uint8_t gRE = Reciver_En;
//static uint8_t gDE = Driver_En;

/**
 * GPSSensor class method definition 
 */
void GPSSensor::initialize() {
    gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    status = 1; 
}

void GPSSensor::get_location(SensorData* data) {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    } 

    if (gps.location.isUpdated()) {
        data->gpsData.latitude  = gps.location.lat();
        data->gpsData.longitude = gps.location.lng();
        data->gpsData.altitude  = gps.altitude.meters();
    } else {
        std::cout << "No GPS loctaion data available." << std::endl;
    }
}

void GPSSensor::get_gps_time(SensorData* data) {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Build a tm struct from GPS date and time
    gpsTime.tm_year = gps.date.year() - 1900;    // tm_year: years since 1900
    gpsTime.tm_mon  = gps.date.month() - 1;      // tm_mon: 0-11
    gpsTime.tm_mday = gps.date.day();
    data->gpsData.hour = gps.time.hour();
    data->gpsData.minute  = gps.time.minute();
    data->gpsData.second  = gps.time.second();
    gpsTime.tm_isdst = 0;                        // No DST correction

    // Convert to epoch seconds
    time_t epochSeconds = mktime(&gpsTime);

    // Approximate milliseconds from centiseconds
    int milliseconds = gps.time.centisecond() * 10;

    // Full UNIX timestamp in milliseconds
    timestampMs = (uint64_t)epochSeconds * 1000 + milliseconds;
}


/**
 * RS485 class method definition 
 */

 // Static Function to manage control of DE/RE
//void RS485::preTransmission() {
//  digitalWrite(gRE, HIGH);
//  digitalWrite(gDE, HIGH);
//}

//void RS485::postTransmission() {
//  digitalWrite(gRE, LOW);
//  digitalWrite(gDE, LOW);
//}
void RS485::initialize() {
    rs485Serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    //pinMode(RE, OUTPUT);
    //pinMode(DE, OUTPUT);
    //digitalWrite(RE, LOW);
    //digitalWrite(DE, LOW);

    // Save it as global variables to use it in the statics function
    //gRE = RE;
    //gDE = DE;

    node.begin(1, rs485Serial);  // Slave and port direction
    //node.preTransmission(preTransmission);
    //node.postTransmission(postTransmission);

    status = 1;
}; 

void RS485::get_data() {
    while (1) {
        uint8_t result = node.readInputRegisters(0x0000, 1);  //read in gears (0–7)

        if (result == node.ku8MBSuccess) {
            uint16_t windDirGear = node.getResponseBuffer(0);

            if (windDirGear <= 7) {
                // Convert the number in char ASCII from 0 to 7
                dataChar = '0' + windDirGear;

                if (dataChar == '\n') {
                    buffer[bufferIdx] = '\0';
                    std::cout << "Received: " << buffer << std::endl;
                    bufferIdx = 0;
                } else if (bufferIdx < BUFFER_SIZE - 1) {
                    buffer[bufferIdx++] = dataChar;
                } else {
                    std::cout << "Buffer overflow, resetting." << std::endl;
                    bufferIdx = 0;
                }
            }
        }
    }
}

uint8_t RS485::get_value() {
    uint8_t result = node.readInputRegisters(0x0000, 1);  //read in gears (0–7)

    if (result == node.ku8MBSuccess) {
        windDirGear = node.getResponseBuffer(0);
    }
    return windDirGear;
}
