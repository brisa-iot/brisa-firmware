#ifndef I2C_LIB_H
#define I2C_LIB_H

#include <stdint.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA219.h>
#include <MPU9250.h>
#include <Wire.h>
#include "data_structs.h"
#include <MPU9250_asukiaaa.h>

// ------------------------------------ BME280 ------------------------------------ //
#define DEFAULT_DELAY   1000
// ------------------------------------ BME280 ------------------------------------ //

// ------------------------------------ MPU9250 ------------------------------------ //
#define MPU9250_ADDRESS 0x68    // I2C address of the MPU9250
#define SAMPLING_TIME1  10      // Sampling time for MPU9250 (ms)
// ------------------------------------ MPU9250 ------------------------------------ //

// ------------------------------------ INA219 ------------------------------------ //
#define INA219_ADDR1    (0x40) // address INA Battery
#define INA219_ADDR2    (0x44) // address INA Solar
#define SAMPLING_TIME2  1000   // Sampling time for INA219 (ms)
#define CAPACITY        2000   // Battery capacity (mAh)
// ------------------------------------ INA219 ------------------------------------ //



class BME280Sensor {
private:

    Adafruit_BME280 bme;        // BME280 object
    uint8_t i2cAddress;         // I2C address of the BME280

    uint16_t delayTime;         // Delay time for sensor reading/sampling

public:
    /**
     * Constructor for BME280Sensor
     * @param i2cAddress_  I2C address of the BME280
     * @param delayTime_   Delay time for sensor reading/sampling
     */
    BME280Sensor(
        uint8_t i2cAddress_ = BME280_ADDRESS, 
        uint16_t delayTime_ = DEFAULT_DELAY
    )
        : i2cAddress(i2cAddress_), delayTime(delayTime_) {}

    void initialize();
    float get_temperature();
    float get_humidity();
    float get_pressure();

public: 
    float temperature;          // Temperature value (°C)
    float humidity;             // Humidity value (%)
    float pressure;             // Pressure value (hPa)

    int status;                 // Status of the BME280 initialization
}; 


class IMUSensor {
private: 

    uint16_t Ts;             // Sampling time for MPU9250 (ms)
    MPU9250_asukiaaa imu;             // MPU9250 object
    uint8_t i2cAddress;      // I2C address of the MPU9250

public: 
    /**
     * Constructor for MPU9520Sensor
     * @param Ts_           Sampling time for MPU9250 (ms)
     * @param i2cAddress_   I2C address of the MPU9250
     * @param Wire_         Wire object for I2C communication
     */
    IMUSensor(
        uint16_t Ts_ = SAMPLING_TIME1, 
        uint8_t i2cAddress_ = MPU9250_ADDRESS
    )            
        : Ts(Ts_), i2cAddress(i2cAddress_), imu() {}

    void initialize();
    void get_accel_gyro_mag(SensorData* data);

public:
    float accelX;           // Accelerometer X-axis value (m/s²)
    float accelY;           // Accelerometer Y-axis value (m/s²)
    float accelZ;           // Accelerometer Z-axis value (m/s²)
    float gyroX;            // Gyroscope X-axis value (rad/s)
    float gyroY;            // Gyroscope Y-axis value (rad/s)
    float gyroZ;            // Gyroscope Z-axis value (rad/s)
    float magX;             // Magnetometer X-axis value (µT)
    float magY;             // Magnetometer Y-axis value (µT)
    float magZ;             // Magnetometer Z-axis value (µT)
}; 


class INA219Sensor {
private: 

    uint16_t Ts;                    // Sampling time for INA219 (ms)
    uint16_t capacity;              // Battery capacity (mAh)       
    Adafruit_INA219 ina219Battery; 
    Adafruit_INA219 ina219Solar;  

    /* Initial SoC Look-Up Table */
    float initSoC[2][21] = {
        {8.4, 8.3, 8.22, 8.16, 8.05, 7.97, 7.91, 7.83, 7.75, 7.71, 7.67, 7.63, 7.59, 7.57, 7.53, 7.49, 7.45, 7.41, 7.37, 7.22, 6.55},
        {1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0.05, 0.00}
    };

public: 
    /**
     * Constructor for INA219Sensor
     * @param Ts_              Sampling time for INA219 (ms)
     * @param capacity_        Battery capacity (mAh)
     * @param batteryAddress_  I2C address of the INA219 - Battery
     * @param solarAddress_    I2C address of the INA219 - PV Solar
     */
    INA219Sensor(
        uint16_t Ts_ = SAMPLING_TIME2,
        uint16_t capacity_ = CAPACITY,
        uint8_t batteryAddress_ = INA219_ADDR1, 
        uint8_t solarAddress_ = INA219_ADDR2           
    )
        : Ts(Ts_),
          capacity(capacity_),
          ina219Battery(batteryAddress_), 
          ina219Solar(solarAddress_) {}

    void initialize();
    float get_battery_voltage();
    float get_battery_current();
    float get_solar_voltage();
    float get_solar_current();
    float get_soc();

public:
    float batteryVoltage;   // Battery voltage (V)
    float batteryCurrent;   // Battery current (mA)
    float solarVoltage;     // Solar voltage (V)
    float solarCurrent;     // Solar current (mA)
    float SoC;              // Battery state of charge (%)

    int status;             // Status of the INA219 initialization

}; 

#endif // I2C_LIB_H