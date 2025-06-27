#include <iostream>
#include "power.h"

/**
 * PowerWrapper class method definition 
 */
int PowerWrapper::initializeAll(){
    buoyPower.initialize();
    status = 1; 
    return status;
}

void PowerWrapper::readAll(SensorData* data){
    if (status != 1) {
        std::cerr << "Power sensors not initialized!" << std::endl;
        return;
    }
    data->powerData.batteryVoltage = buoyPower.get_battery_voltage();
    data->powerData.solarVoltage = buoyPower.get_solar_voltage();
    data->powerData.batteryCurrent = buoyPower.get_battery_current();
    data->powerData.solarCurrent = buoyPower.get_solar_current();
    data->powerData.batterySoC = buoyPower.get_soc();   // Not in (%)
}