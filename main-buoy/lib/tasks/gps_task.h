#ifndef GPS_TASK_H
#define GPS_TASK_H

#include <Arduino.h>         
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"     
#include "freertos/semphr.h"   
#include "esp_err.h"
#include "uart_interface.h"    
#include "driver/uart.h"
#include "params_config.h"
#include "global_config.h"  
#include "global_vars.h"


/**
 * @brief Initializes the GPS task and serial communication.
 * 
 * @param gps_sensor_instance An instance of the `GPS` class that handles GPS data acquisition.
 * @param gps_task_handler A reference to a `TaskHandle_t` that will hold the handle of the created GPS task.
 */
esp_err_t gps_task_init(GPS& gps_sensor_instance, TaskHandle_t& gps_task_handler);

/**
 * @brief FreeRTOS task for continuous GPS data acquisition and processing.
 * This task continuously reads/encodes NMEA data from the GPS sensor and
 * updates the global `g_current_gps_data` structure.
 *
 * @param pvParameters A pointer to the initialized `GPS` class instance.
 */
extern "C" void gps_data_task(void *pvParameters);

#endif // GPS_TASK_H 