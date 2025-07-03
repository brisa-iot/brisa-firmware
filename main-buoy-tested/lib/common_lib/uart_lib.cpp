#include <iostream>
#include "uart_lib.h"
#include <time.h>  


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

    if (gps.location.isValid()) {
        data->gpsData.latitude  = gps.location.lat();
        data->gpsData.longitude = gps.location.lng();
        data->gpsData.altitude  = gps.altitude.meters();
    } else {
        data->gpsData.latitude  = -33.0;   // Valor por defecto si no hay actualización
        data->gpsData.longitude = -71.54;  // Valor por defecto si no hay actualización
        data->gpsData.altitude  = 0.0;     // Valor por defecto si no hay actualización
    }
}

void GPSSensor::get_gps_time(SensorData* data) {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    if (gps.time.isValid() && gps.date.isValid()) {
        // Construir estructura de tiempo
        gpsTime.tm_year = gps.date.year() - 1900;
        gpsTime.tm_mon  = gps.date.month() - 1;
        gpsTime.tm_mday = gps.date.day();
        gpsTime.tm_hour = gps.time.hour();
        gpsTime.tm_min  = gps.time.minute();
        gpsTime.tm_sec  = gps.time.second();
        gpsTime.tm_isdst = 0;

        time_t timestamp = mktime(&gpsTime);

        data->gpsData.hour       = gpsTime.tm_hour;
        data->gpsData.minute     = gpsTime.tm_min;
        data->gpsData.second     = gpsTime.tm_sec;
        data->gpsData.timestamp_s = timestamp;
    }

    else {
        data->gpsData.timestamp_s = INIT_TIMESTAMP + (millis()/1000);
    }
}


/**
 * RS485 class method definition 
 */

void RS485::initialize() {
    rs485Serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    node.begin(1, rs485Serial);  
    status = 1;
}; 


int RS485::get_value() {
    uint8_t result = node.readInputRegisters(0x0001, 1);  

    if (result == node.ku8MBSuccess) {
        return node.getResponseBuffer(0);  
    }

    return -1;  
}

