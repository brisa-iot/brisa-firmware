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
        //Serial.println("❌ No se detectó ningún sensor DS18B20.");
        return;
    }

    if (!dallas.getAddress(deviceAddress, deviceIndex)) {
        sensorFound = false;
        //Serial.println("❌ No se pudo obtener la dirección del sensor.");
        return;
    }

    sensorFound = true;
    //Serial.println("✅ Sensor DS18B20 inicializado correctamente.");
    //oneWire.reset_search(); 
    
    //if (!oneWire.search(addr)) {
    //   sensorFound = false;
    //    return;
    //}

    //if (OneWire::crc8(addr, 7) != addr[7]) {
    //    sensorFound = false;
    //    return;
    //}

    //if (addr[0] != 0x10 && addr[0] != 0x28) {
    //    sensorFound = false;
    //    return;
    //}

    //sensorFound = true;
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
    //if (!sensorFound) return -1000.0f;
    
    //oneWire.reset();
    //oneWire.select(addr);
    //oneWire.write(0x44, 1); // Start conversion, with parasite power on at the end

    //delay(delayConv);       // Wait for conversion to complete

    //oneWire.reset();
    //oneWire.select(addr);
    //oneWire.write(0xBE);    // Read Scratchpad

    //for (int i = 0; i < 9; i++) { // We need 9 bytes
    //    data[i] = oneWire.read();
    //}

    //MSB = data[1];
    //LSB = data[0];

    //tempRead = ((MSB << 8) | LSB); // Using two's complement
    //tempValue = tempRead / 16.0f;
    //return tempValue;
}


/**
 * WaterWrapper ----------------------------------------------------------
 */
int WaterWrapper::initializeAll(){
    //pHsensor.initialize();
    conductivitySensor.initialize();
    //oxygenSensor.initialize();
    tempSensor.initialize();
    status = 1; 
    return status; 
}

void WaterWrapper::readAll(SensorData* data){
    if (status != 1) {
        std::cerr << "Sensors not initialized!" << std::endl;
        return;
    }
    //data->waterData.pH = pHsensor.get_value();
    data->waterData.sigma = conductivitySensor.get_value();
    //data->waterData.oxygen = oxygenSensor.get_value();
    data->waterData.temperature = tempSensor.get_temp();
}

