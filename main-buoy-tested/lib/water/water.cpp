#include <iostream>
#include "water.h"

/**
 * TemperatureSensor class method definition 
 */

void DS18B20Sensor::initialize() {
    dallas.begin();

    int count = dallas.getDeviceCount();
    delay(50);
    if (count == 0) {
        sensorFound = false;
        return;
    }

    if (!dallas.getAddress(deviceAddress, deviceIndex)) {
        sensorFound = false;
        return;
    }

    sensorFound = true;
}


float DS18B20Sensor::get_temp() {
    if (!sensorFound) {
        tempValue = DEVICE_DISCONNECTED_C;
        return tempValue;
    }

    dallas.requestTemperatures();
    delay(convDelayMs);

    float tempC = dallas.getTempCByIndex(deviceIndex);
    if (tempC == DEVICE_DISCONNECTED_C) {
        tempValue = DEVICE_DISCONNECTED_C;
        return tempValue;
    }

    tempValue = tempC;
    return tempValue;
}


/**
 * WaterWrapper ----------------------------------------------------------
 */
int WaterWrapper::initializeAll(){
    pHsensor.initialize();
    conductivitySensor.initialize();
    oxygenSensor.initialize();
    tempSensor.initialize();
    status = 1; 
    return status; 
}

void WaterWrapper::readAll(SensorData* data){
    if (status != 1) {
        std::cerr << "Sensors not initialized!" << std::endl;
        return;
    }
    data->waterData.pH = pHsensor.get_value();
    data->waterData.sigma = conductivitySensor.get_value();
    data->waterData.oxygen = oxygenSensor.get_value();
    data->waterData.temperature = tempSensor.get_temp();
}

