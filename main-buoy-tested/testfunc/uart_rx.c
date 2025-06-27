#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UARTn        UART_NUM_1
#define UART_TX_PIN  18
#define UART_RX_PIN  5 

static const int NUM_BYTES_TX = 4;
static const int RX_BUF_SIZE = 1024;


void init_uart(QueueHandle_t *uart_queue) 
{
    uart_config_t conf = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    }; 
    ESP_ERROR_CHECK(uart_param_config(UARTn, &conf));
    ESP_ERROR_CHECK(uart_set_pin(UARTn, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UARTn, RX_BUF_SIZE * 2, 0, 10, uart_queue, 0));
}


void uart_rx_task(void *pvParameters)
{   
    QueueHandle_t uart_queue = *(QueueHandle_t *)pvParameters;
    uart_event_t uart_event;
    uint8_t *data = (uint8_t *) malloc(RX_BUF_SIZE + 1);
    
    while(1) {
        if (xQueueReceive(uart_queue, (void *)&uart_event, portMAX_DELAY)) {
            bzero(data, RX_BUF_SIZE);
            printf("UART Event: %d. UART Size: %d\n", uart_event.type, uart_event.size);
            switch (uart_event.type) {
                case UART_DATA:
                    if (uart_event.size > 0) {
                        int rxBytes = uart_read_bytes(UARTn, data, NUM_BYTES_TX, portMAX_DELAY);
                        printf("Received %d bytes. d1 = %d, d2 = %d, d3 = %d, d4 = %d\n", \
                            rxBytes, data[0], data[1], data[2], data[3]); 
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

void app_main() {

    QueueHandle_t uart_queue;
    init_uart(&uart_queue);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, (void *)&uart_queue, 1, NULL);
}