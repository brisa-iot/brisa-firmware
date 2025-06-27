#include <stdint.h>
#include <iostream>
#include "i2c_lib.h"
#include <Arduino.h>

/**
 * BME280Sensor class method definition 
 */
void BME280Sensor::initialize() {
    if (!bme.begin(i2cAddress, &Wire)) {
        std::cerr << "Could not find a valid BME280 sensor, check wiring!" << std::endl;
        return;
    }
    status = 1; // Set status to indicate successful initialization
}

float BME280Sensor::get_temperature() {
    if (status != 1) {
        std::cerr << "BME280 sensor not initialized!" << std::endl;
        return 0.0f; 
    }
    temperature = bme.readTemperature();
    return temperature;
}

float BME280Sensor::get_humidity() {
    if (status != 1) {
        std::cerr << "BME280 sensor not initialized!" << std::endl;
        return 0.0f; 
    }
    humidity = bme.readHumidity();
    //Serial.print("Humedad: ");
    //Serial.println(humidity);
    return humidity;
}

float BME280Sensor::get_pressure() {
    if (status != 1) {
        std::cerr << "BME280 sensor not initialized!" << std::endl;
        return 0.0f; 
    }
    pressure = bme.readPressure() / 100.0F; 
    return pressure;
}


/**
 * MPU9250Sensor class method definition 
 */
void IMUSensor::initialize() {
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
}

void IMUSensor::get_accel_gyro_mag(SensorData* data) {
    imu.accelUpdate();
    imu.gyroUpdate();
    imu.magUpdate();
    data->imuPosData.accelX = imu.accelX();
    data->imuPosData.accelY = imu.accelY();
    data->imuPosData.accelZ = imu.accelZ();

    data->imuPosData.gyroX = imu.gyroX();
    data->imuPosData.gyroY = imu.gyroY();
    data->imuPosData.gyroZ = imu.gyroZ();

    data->imuPosData.magX = imu.magX();
    data->imuPosData.magY = imu.magY();
    data->imuPosData.magZ = imu.magZ();
    
}


/**
 * INA219Sensor class method definition 
 */
void INA219Sensor::initialize() {
    if (!ina219Battery.begin()) {
        std::cerr << "Could not find a valid INA219 sensor for battery, check wiring!" << std::endl;
        return;
    }
    if (!ina219Solar.begin()) {
        std::cerr << "Could not find a valid INA219 sensor for solar, check wiring!" << std::endl;
        return;
    }

    // ********** Find initial SoC Value: Binary Search + Interpolation ********** //
    batteryVoltage = ina219Battery.getBusVoltage_V();

    int num_voltages = sizeof(initSoC[0]) / sizeof(initSoC[0][0]);
    int left = 0, right = num_voltages - 1;
    int mid = 0;
    while (left < right) {
        mid = left + (right - left) / 2;
        if (initSoC[0][mid] > batteryVoltage) {
            left = mid + 1;
        } else {
            right = mid;
        }
    }
    if (left == 0) {
        SoC = initSoC[1][0];
    } else if (left == num_voltages) {
        SoC = initSoC[1][num_voltages - 1];
    } else {
        // Linear interpolation: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        SoC = initSoC[1][left - 1] + 
             (batteryVoltage - initSoC[0][left - 1]) * 
             (initSoC[1][left] - initSoC[1][left - 1]) / 
             (initSoC[0][left] - initSoC[0][left - 1]);
    }

}

float INA219Sensor::get_battery_voltage() {
    batteryVoltage = ina219Battery.getBusVoltage_V();
    return batteryVoltage;
}

float INA219Sensor::get_battery_current() {
    batteryCurrent = ina219Battery.getCurrent_mA(); 
    batteryCurrent = batteryCurrent / 1000.0f; 
    return batteryCurrent;
}

float INA219Sensor::get_solar_voltage() {
    solarVoltage = ina219Solar.getBusVoltage_V();
    return solarVoltage;
}

float INA219Sensor::get_solar_current() {
    solarCurrent = ina219Solar.getCurrent_mA();
    solarCurrent = solarCurrent / 1000.0f;
    return solarCurrent;
}

float INA219Sensor::get_soc() {
    SoC = SoC - 
    (batteryCurrent * (float)Ts) / ((float)capacity * 3600.0f * 1000.0f);
    return SoC;
}
