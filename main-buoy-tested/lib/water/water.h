#ifndef WATER_H 
#define WATER_H

#include <stdint.h>
#include <OneWire.h>
#include "driver/adc.h"
#include "adc_lib.h"
#include "data_structs.h"
#include "DallasTemperature.h"



#define ADC_CH_1        ADC1_CHANNEL_4  // ADC1 channel 4 --> GPIO32

// ---------------- Conductivity sensor ---------------- //
#define ADC_CH_2                ADC1_CHANNEL_6                // ADC1 channel 6 --> GPIO34
#define VREF_CONDUCTIVITY       (3.4f)                        // Reference voltage for conductivity sensor
#define SCALE_CONDUCTIVITY      (3.4f/(20.0f*1000.0f))        // Scale factor for conductivity sensor: for reading from 0 t0 20 uS/cm
#define OFFSET_CONDUCTIVITY     (0.0f)                        // Offset for conductivity sensor calibration
// ---------------- Conductivity sensor ---------------- //

#define ADC_CH_3        ADC1_CHANNEL_7  // ADC1 channel 7 --> GPIO35

#define ONE_WIRE_BUS    33              // GPIO33 pin for OneWire bus
#define CONVERSION_DELAY 750            // Delay for DS18B20 conversion in milliseconds


class DS18B20Sensor {
private:
    OneWire oneWire;
    DallasTemperature dallas;
    uint8_t deviceIndex;
    uint32_t convDelayMs;
    uint8_t deviceAddress[8];
    float tempValue;
    bool sensorFound;

public:
    DS18B20Sensor(uint8_t pin = ONE_WIRE_BUS, uint32_t delayConv = CONVERSION_DELAY)
        : oneWire(pin),
          dallas(&oneWire),
          deviceIndex(0),
          convDelayMs(delayConv),
          tempValue(DEVICE_DISCONNECTED_C),
          sensorFound(false)
    {}

    void initialize();
    float get_temp();
    bool isSensorFound() const { return sensorFound; }
};


class WaterWrapper {
private: 
    SensorADC pHsensor;
    SensorADC conductivitySensor;
    SensorADC oxygenSensor;
    DS18B20Sensor tempSensor;
    int status;

public: 
    /**
     * Constructor for WaterWrapper
     * @param adc_channel_  ADC channel for the sensor
     * @param adc_atten_    ADC attenuation for the sensor
     * @param scale_        Scale factor for the sensor
     * @param offset_       Offset for the sensor
     */
    WaterWrapper() 
        : pHsensor(ADC_CH_1, ADC_ATTEN_DB_11, 1.0f, 0.0f), 
          conductivitySensor(ADC_CH_2, ADC_ATTEN_DB_11, SCALE_CONDUCTIVITY, OFFSET_CONDUCTIVITY, VREF_CONDUCTIVITY), 
          oxygenSensor(ADC_CH_3, ADC_ATTEN_DB_11, 1.0f, 0.0f), 
          tempSensor() 
    {}

    /**
     * Initializes all sensors
     */
    int initializeAll();
    /**
     * Reads all sensor values
     * @param data Pointer to the data structure to store sensor values
     */
    void readAll(SensorData* data);
};

#endif // WATER_H