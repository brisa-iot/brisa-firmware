#ifndef _DYNAMICS_TASK_H
#define _DYNAMICS_TASK_H

#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "fast_dynamics.h"
#include "power_monitor.h"
#include "medium_dynamics.h"
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


// ------------------------ Struct for Wrapping Dynamic Tasks Resources ------------------------ //

typedef struct {
    PowerMonitor* power_monitor_ptr;           
    MediumDynamicsWrapper* med_dyn_wrapper_ptr; 
} DynamicsTaskWrappers_t;
// ------------------------ Struct for Wrapping Dynamic Tasks Resources ------------------------ //

/**
 * @brief Initializes the Dynamics system, including FreeRTOS queues,
 * the averaged data buffer, sensor wrapper, and creates the tasks.
 * 
 *  @param dynamics_task_wrapper A reference to the DynamicsTaskWrappers_t structure containing
 *                              pointers to the PowerMonitor and MediumDynamicsWrapper instances.
 *  @param dynamics_task_handler A reference to the TaskHandle_t that will be used to
 *                              manage the Dynamics task.
 */
esp_err_t _dynamics_task_init(DynamicsTaskWrappers_t& dynamics_task_wrapper, TaskHandle_t& dynamics_task_handler);

/**
 * @brief Task that acquires data from fast dynamics sensors, power monitor, and medium dynamics sensors.
 * This task reads raw data from the sensors, averages it over a defined number of samples,
 * and stores the averaged data in the respective buffers.
 */
extern "C" void _data_acquisition_tasks(void *pvParameters);

#endif // _DYNAMICS_TASK_H