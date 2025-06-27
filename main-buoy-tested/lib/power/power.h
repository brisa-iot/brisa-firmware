#ifndef POWER_H
#define POWER_H

#include <stdint.h>
#include "i2c_lib.h"
#include "data_structs.h"

// ------------------------------------ INA219 ------------------------------------ //
#define INA219_ADDR1    (0x40) // address INA Battery
#define INA219_ADDR2    (0x44) // address INA Solar
#define SAMPLING_TIME   1000   // Sampling time for INA219 (ms)
#define CAPACITY        2000   // Battery capacity (mAh)
// ------------------------------------ INA219 ------------------------------------ //


class PowerWrapper {
private: 

    INA219Sensor buoyPower; 
    int status;

public: 
    /**
     * Constructor for PowerWrapper
     * @param Ts            Sampling time for INA219 (ms)
     * @param capacity      Battery capacity (mAh)
     * @param ina219_addr1  I2C address for the first INA219 sensor
     * @param ina219_addr2  I2C address for the second INA219 sensor
     */
    PowerWrapper(
        uint16_t Ts = SAMPLING_TIME,
        uint16_t capacity = CAPACITY,
        uint8_t ina219_addr1 = INA219_ADDR1, 
        uint8_t ina219_addr2 = INA219_ADDR2) 
        : buoyPower(
            Ts, capacity, ina219_addr1, ina219_addr2
        ) {}
    
    /**
     * Initialize all sensors
     */
    int initializeAll();
    /**
     * Read all sensors: power data
     * @param data Pointer to the SensorData union data structure
     */
    void readAll(SensorData* data);
};
#endif // POWER_H