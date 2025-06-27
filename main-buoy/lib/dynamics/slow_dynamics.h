#ifndef SLOW_DYNAMICS_H
#define SLOW_DYNAMICS_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "adc_measurements.h"
#include "one_wire_interface.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


class SlowDynamicsWrapper {
private: 
    ADC _ph_sensor; 
    ADC _conductivity_sensor; 
    ADC _dissolved_o2_sensor; 
    DS18B20 _temperature_sensor; 

public: 
    /**
     * @brief Constructor for SlowDynamicsWrapper 
     * @param ph_cfg The configuration for the pH ADC sensor. 
     * @param conductivity_cfg The configuration for the pH ADC.
     * @param dissolved_o2_cfg The configuration for the conductivity ADC. 
     * @param temperature_cfg ModBus configutation for the temperature sensor.
     */
    SlowDynamicsWrapper(
        const ADC_Config_t& ph_cfg, 
        const ADC_Config_t& conductivity_cfg,
        const ADC_Config_t& dissolved_o2_cfg,
        const DS18B20_Config_t& temperature_cfg
    ); 

    /**
     * @brief Initializes all sensors managed by the wrapper
     * @return ESP_OK on success, an error code otherwise.
     */
    esp_err_t initialize();

    /**
     * @brief Reads water variables: pH, conductivity, dissolved O2 and temperature (One Wire).
     * @param data_k Pointer to the SlowDynamics struct to fill with data.
     * @return ESP_OK on successful read, an error code otherwise.
     */
    esp_err_t readSingle(SlowDynamics* data_k);
}; 

#endif // SLOW_DYNAMICS_H 