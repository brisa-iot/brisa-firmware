#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <stdint.h>

struct WaterData {
    float pH;               // pH value
    float sigma;            // Conductivity value
    float oxygen;           // Oxygen value
    float temperature;      // Temperature value
}; 

struct EnvData {
    float temperature;      // Temperature value (°C)
    float humidity;         // Humidity value (%)
    float pressure;         // Pressure value (hPa)
    float windSpeed;        // Wind speed value 
    int windDirection;   // Wind direction value
};

struct PowerData {
    float batteryVoltage;   // Battery voltage (V)
    float batteryCurrent;   // Battery current (mA)
    float solarVoltage;     // Solar voltage (V)
    float solarCurrent;     // Solar current (mA)
    float batterySoC;       // Battery state of charge (%)
};

struct ImuPosData {
    float accelX;           // Accelerometer X-axis value
    float accelY;           // Accelerometer Y-axis value
    float accelZ;           // Accelerometer Z-axis value
    float gyroX;            // Gyroscope X-axis value
    float gyroY;            // Gyroscope Y-axis value
    float gyroZ;            // Gyroscope Z-axis value
    float magX;             // Magnetometer X-axis value
    float magY;             // Magnetometer Y-axis value
    float magZ;             // Magnetometer Z-axis value
};

struct GPSData {
    double latitude; 
    double longitude;   
    int hour; 
    int minute;
    int second;
    uint64_t timestamp_s;   // Timestamp in milliseconds
}; 

struct SensorData {
    WaterData waterData;
    EnvData envData;
    PowerData powerData;
    ImuPosData imuPosData;
    GPSData gpsData;
}; 

#endif // DATA_STRUCTS_H