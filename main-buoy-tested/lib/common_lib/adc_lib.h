#ifndef ADC_LIB_H
#define ADC_LIB_H

#include <stdint.h>
#include "driver/adc.h"

#define VREF            3.3f              // Reference voltage
#define ADC_MAX         4095              // 12-bit resolution (0-4095)
#define ADC_BITWIDTH    ADC_WIDTH_BIT_12  // 12-bit resolution
#define ADC_UNIT        ADC_UNIT_1        // SAR ADC 1

#define DEFAULT_CHANNEL ADC1_CHANNEL_0    // Default ADC channel (GPIO36)
#define DEFAULT_ATTEN   ADC_ATTEN_DB_0    // Default attenuation
#define DEFAULT_SCALE   1.0f              // Default scale factor
#define DEFAULT_OFFSET  0.0f              // Default offset for calibration


class SensorADC {
protected:
    adc_unit_t adcUnit;              // ADC unit (ADC1 or ADC2)
    adc_bits_width_t adcBitwidth;    // ADC resolution bits
    uint16_t adcMaxResolution;       // Max ADC value (e.g. 4095 for 12-bit)
    float vRef;                      // Reference voltage
    float scale;                     // Conversion factor for scaling ADC readings to physical values
    float offset;                    // Offset for sensor calibration

    adc1_channel_t adc_channel;      // ADC channel for the sensor (physical pin)
    gpio_num_t adc_gpio;             // GPIO pin for the ADC channel
    adc_atten_t adc_atten;           // ADC attenuation

public:
    // Constructor with optional scale and offset; ADC params initialized by macros
    SensorADC(
        adc1_channel_t adc_channel_ = DEFAULT_CHANNEL,    // Default channel
        adc_atten_t adc_atten_ = DEFAULT_ATTEN,           // Default attenuation
        float scale_ = DEFAULT_SCALE,                     // Default scale factor
        float offset_ = DEFAULT_OFFSET,                   // Default offset
        float vRef_ = VREF,
        adc_unit_t adcUnit_ = ADC_UNIT,
        adc_bits_width_t adcBitwidth_ = ADC_BITWIDTH,
        uint16_t adcMaxResolution_ = ADC_MAX
    ) 
    : adc_channel(adc_channel_),
      adc_atten(adc_atten_),
      scale(scale_),
      offset(offset_),
      adcUnit(adcUnit_),
      adcBitwidth(adcBitwidth_),               
      vRef(vRef_),
      adcMaxResolution(adcMaxResolution_) {}

    virtual ~SensorADC() {}

    // Pure virtual functions to be implemented by derived classes
    void initialize();
    float get_value();

public: 

    int adc_raw;                     // Raw ADC value
    float adc_voltage;               // ADC voltage value
    float value;                     // Scaled value

    int status;                      // Status of the ADC initialization
};

#endif // ADC_LIB_H
