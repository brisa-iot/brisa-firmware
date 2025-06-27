#ifndef LORA_TASK_H
#define LORA_TASK_H

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

#include "_dynamics_task.h"
#include "gps_task.h"
#include "slow_dynamics_task.h"


/**
 * @brief Initializes the LoRa task and serial communication.
 * 
 * @param lora_module Reference to the `LoRaModule` class instance.
 * @param lora_task_handler Reference to the FreeRTOS task handle for the Lo
 */
esp_err_t lora_tx_task_init(LoRaModule& lora_module, TaskHandle_t& lora_task_handler);

/**
 * @brief FreeRTOS task for continuous LoRa data transmission.
 * This task continuously reads/encodes data from the LoRa module and
 * handles communication with the LoRa network.
 *
 * @param pvParameters A pointer to the initialized `LoRa` class instance.
 */
extern "C" void lora_tx_data_task(void *pvParameters);

#endif // LORA_TASK_H