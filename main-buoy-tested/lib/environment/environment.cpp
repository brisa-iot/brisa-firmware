#include <stdint.h>
#include <iostream>
#include "environment.h"
#include <Arduino.h>

/**
 * EnvironmentWrapper -------------------------------------------------
 */
int EnvironmentWrapper::initializeAll() {
    bme280Sensor.initialize();
    anemometerSensor.initialize();
    vaneSensor.initialize();
    status = 1; 
    return status;
}

void EnvironmentWrapper::readAll(SensorData* data) {
    if (status != 1) {
        std::cerr << "Sensors not initialized!" << std::endl;
        return;
    }
    data->envData.temperature = bme280Sensor.get_temperature();
    data->envData.humidity = bme280Sensor.get_humidity();
    data->envData.pressure = bme280Sensor.get_pressure();
    data->envData.windSpeed = 6.0*anemometerSensor.get_value()*(5.0/3.3);
    data->envData.windDirection = (float)vaneSensor.get_value();
}