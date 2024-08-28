#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "test.h"

#include "libopencm3/stm32/rcc.h"

void taskTest(void *args __attribute__((unused))){
    // Definición de la cadena de prueba
    char string_1[] = "Probando UART 1\r\n";
    char string_2[] = "Probando UART 2\r\n";
    
    int len_string_1 = strlen(string_1);
    int len_string_2 = strlen(string_2);

    // Se encola la cadena de prueba en la cola de TX
    uint16_t nsent_1 = UART_puts(USART1, string_1);
    uint16_t nsent_2 = UART_puts(USART2, string_2);

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_1 != sizeof(string_1) - 1){
        UART_puts(USART3, "Error al enviar datos por UART1\r\n");
        vTaskDelete(NULL);
    }

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_2 != sizeof(string_2) - 1){
        UART_puts(USART3, "Error al enviar datos por UART2\r\n");
        vTaskDelete(NULL);
    }

    // Se espera un segundo
    vTaskDelay(pdMS_TO_TICKS(2000));

    int data;
    int i = 0;
    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while((data = UART_receive(USART1)) != -1){
        if(string_1[i] != (char)data || i == len_string_1) {
            UART_puts(USART3, "Error al recibir datos por UART1.\r\n");
            continue;
        }
        i++;
    }

    if(data == -1)
        UART_puts(USART3, "Prueba UART1 runned successfully\r\n");

    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while((data = UART_receive(USART2)) != -1){
        if(string_1[i] != (char)data || i == len_string_2) {
            UART_puts(USART3, "Error al recibir datos por UART2.\r\n");
            continue;
        }
        i++;
    }

    if(data == -1)
        UART_puts(USART3, "Prueba UART2 runned successfully\r\n");
    
    vTaskDelete(NULL);
}

void taskPrintBuffer(void *args __attribute__((unused))) {
    for (;;) {
        UART_print_buffer(USART1);
        UART_print_buffer(USART2);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

