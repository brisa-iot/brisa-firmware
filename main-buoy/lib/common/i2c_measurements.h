#ifndef I2C_MEAS_H
#define I2C_MEAS_H

#include <stdint.h>
#include <vector>          
#include "Adafruit_BME280.h"         
#include "Adafruit_INA219.h"       
#include "MPU9250_asukiaaa.h"       
#include "filters.h"                
#include "Wire.h"                   
#include "esp_err.h"   
#include "global_config.h"       


// ------------------------------------ Classes ------------------------------------ //
// --- BME280: Class Definition ---
class BME280 {
private: 
    Adafruit_BME280 _bme280; 
    TwoWire* _wire_bus; 

    uint8_t _i2c_address; 
    float _pressure_hPa_scale; 

public:
    /**
     * @brief Constructor for the BME280 class.
     *
     * @param wire_bus_ptr A pointer to the initialized TwoWire (I2C) bus object.
     * The BME280 class does not own or initialize the I2C bus.
     * @param config The configuration struct for the BME280 sensor.
     */
    BME280(TwoWire* wire_bus_ptr, const BME280_Config_t& config);

    /**
     * @brief Initializes the BME280 sensor.
     * This method checks the sensor connection and sets it up.
     *
     * @return ESP_OK on successful initialization, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads temperature, pressure, and humidity from the BME280 sensor.
     *
     * @param temperature Pointer to store the temperature value (°C).
     * @param pressure    Pointer to store the pressure value (hPa).
     * @param humidity    Pointer to store the humidity value (%).
     * @return ESP_OK on successful read, an error code otherwise.
     */
    esp_err_t getRawData(float* temperature, float* pressure, float* humidity);
}; 


// --- INA219: Class Definition ---
class INA219 {
private: 
    Adafruit_INA219 _ina219_battery; 
    Adafruit_INA219 _ina219_solar;  
    TwoWire* _wire_bus; 

    uint8_t _i2c_address_battery;    
    uint8_t _i2c_address_solar;     

    /* State of Charge (SoC) related variables */
    uint32_t _capacity_mah;          // Battery capacity in mAh
    uint32_t _dt_soc;                // Sampling time for SoC calculation in ms
    /* Initial SoC Look-Up Table */
    float _soc;                      // State of Charge (SoC) value
    float _soc_lut[2][21] = {
        {8.4, 8.3, 8.22, 8.16, 8.05, 7.97, 7.91, 7.83, 7.75, 7.71, 7.67, 7.63, 7.59, 7.57, 7.53, 7.49, 7.45, 7.41, 7.37, 7.22, 6.55},
        {100.0, 95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 55.0, 50.0, 45.0, 40.0, 35.0, 30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 0.00}
    }; 

public:
    /**
     * @brief Constructor for the INA219 class.
     *
     * @param wire_bus_ptr A pointer to the initialized TwoWire (I2C) bus object.
     * The INA219 class does not own or initialize the I2C bus.
     * @param config The configuration struct for the INA219 sensors.
     */
    INA219(TwoWire* wire_bus_ptr, const INA219_Config_t& config);

    /**
     * @brief Initializes the INA219 sensors.
     * This method checks the sensor connection and sets it up.
     *
     * @return ESP_OK on successful initialization, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Computes the State of Charge (SoC) based on battery current.
     * Performs Coulomb counting to calculate the SoC.
     * 
     * @param battery_current Pointer to the current value (mA) used for SoC calculation.
     */
    void updateSoC(float* battery_current);

    void clampSoC() {
        if (_soc < 0.0f) {
            _soc = 0.0f;
        } else if (_soc > 100.0f) {
            _soc = 100.0f;
        }
    }

    /**
     * @brief Reads voltage, current, and power from the INA219 sensors.
     *
     * @param battery_voltage Pointer to store the voltage value (V).
     * @param battery_current Pointer to store the current value (mA).
     * @param solar_voltage   Pointer to store the power value (V).
     * @param solar_current   Pointer to store the current value (mA).
     * @param soc             Pointer to store the state of charge (SoC) value (0-100%).
     * @return ESP_OK on successful read, an error code otherwise.
     */
    esp_err_t getRawData(
        float* battery_voltage, float* battery_current, 
        float* solar_voltage, float* solar_current,
        float* soc
    );
}; 


// --- IMU: Class Definition ---
class IMU {
private:
    MPU9250_asukiaaa _imu; 
    TwoWire* _wire_bus;      

    uint8_t _i2c_address;        

    std::vector<float> _filter_coeffs; 
    FIRFilter _accelX_filter;
    FIRFilter _accelY_filter;
    FIRFilter _accelZ_filter;
    FIRFilter _gyroX_filter;
    FIRFilter _gyroY_filter;
    FIRFilter _gyroZ_filter;
    FIRFilter _magX_filter;
    FIRFilter _magY_filter;
    FIRFilter _magZ_filter;

public:
    /**
     * @brief Constructor for the IMU class.
     *
     * @param wire_bus_ptr A pointer to the initialized TwoWire (I2C) bus object.
     * The IMU class does not own or initialize the I2C bus.
     * @param config The configuration struct for the IMU sensor.
     */
    IMU(TwoWire* wire_bus_ptr, const IMU_Config_t& config);

    /**
     * @brief Initializes the MPU9250 sensor.
     * This method checks the sensor connection and sets it up.
     *
     * @return ESP_OK on successful initialization, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads raw data from the MPU9250 (accelerometer, gyroscope, magnetometer).
     * This method updates the sensor's internal buffers.
     *
     * @param accelX, accelY, accelZ Pointers to floats to store raw accelerometer data (m/s²).
     * @param gyroX, gyroY, gyroZ Pointers to floats to store raw gyroscope data (rad/s).
     * @param magX, magY, magZ Pointers to floats to store raw magnetometer data (µT).
     * @return ESP_OK on successful read, an error code otherwise.
     */
    esp_err_t getRawData(
        float* accelX, float* accelY, float* accelZ,
        float* gyroX, float* gyroY, float* gyroZ,
        float* magX, float* magY, float* magZ
    );

    /**
     * @brief Reads raw data from the MPU9250 and applies FIR filtering.
     * This is the primary method for getting processed IMU data.
     *
     * @param accelX, accelY, accelZ Pointers to floats to store filtered accelerometer data (m/s²).
     * @param gyroX, gyroY, gyroZ Pointers to floats to store filtered gyroscope data (rad/s).
     * @param magX, magY, magZ Pointers to floats to store filtered magnetometer data (µT).
     * @return ESP_OK on successful read and filter, an error code otherwise.
     */
    esp_err_t getFilteredData(
        float* accelX, float* accelY, float* accelZ,
        float* gyroX, float* gyroY, float* gyroZ,
        float* magX, float* magY, float* magZ
    );

    /**
     * @brief Resets the internal buffers of all FIR filters.
     * Useful for clearing old data if measurement conditions change.
     */
    void resetFilterBuffers();
};

#endif // I2C_MEAS_H 