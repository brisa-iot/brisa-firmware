#ifndef SLOW_DYNAMICS_TASK_H
#define SLOW_DYNAMICS_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"          
#include "slow_dynamics.h"
#include "uart_interface.h"    
#include "params_config.h"
#include "global_config.h"
#include "global_vars.h"


/**
 * @brief Initializes the Slow Dynamics system, including FreeRTOS queues,
 * the slow dynamics data buffer, and creates the task for data acquisition.
 */
esp_err_t slow_dynamics_task_init(SlowDynamicsWrapper& slow_dynamics_wrapper, TaskHandle_t& slow_dynamics_task_handler);

/**
 * @brief FreeRTOS task for continuous slow dynamics data acquisition and processing.
 * This task continuously reads data from the slow dynamics sensor and updates the
 * global `g_slow_dynamics_buffer` structure.
 * 
 * @param pvParameters A pointer to the initialized slow dynamics sensor instance.
 */
extern "C" void slow_data_acquisition_task(void *pvParameters);

#endif // SLOW_DYNAMICS_TASK_H