#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <stdint.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "driver/adc.h"
#include "adc_lib.h"
#include "i2c_lib.h"
#include "uart_lib.h"
#include "data_structs.h"

#define ADC_CH          ADC1_CHANNEL_3   // ADC1 channel 3 --> GPIO39
#define BME280_ADDR     (0x76)           // I2C address of the BME280


class EnvironmentWrapper {
private: 
    SensorADC anemometerSensor; 
    RS485 vaneSensor;
    BME280Sensor bme280Sensor;
    int status;

public: 
    /**
     * Constructor for EnvironmentWrapper
     * -------------- ADC ---------------
     * @param adc_channel_  ADC channel for the sensor
     * @param adc_atten_    ADC attenuation for the sensor
     * @param scale_       Scale factor for the sensor
     * @param offset_      Offset for the sensor
     */
    EnvironmentWrapper()
        : anemometerSensor(ADC_CH, ADC_ATTEN_DB_11, 1.0f, 0.0f), 
          vaneSensor(),
          bme280Sensor(BME280_ADDR) {} 

    /**
     * Initialize all sensors
     */
    int initializeAll();   
    /**
     * Read all sensor values
     * @param data  Pointer to SensorData structure to store sensor values
     */
    void readAll(SensorData* data);
}; 

#endif // ENVIRONMENT_H