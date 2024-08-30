#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "test.h"

#include "libopencm3/stm32/rcc.h"

void taskTest(void *args __attribute__((unused))){
    // Espero 100 ms para que se envíen los datos
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Definición de la cadena de prueba
    char string_1[] = "Testing UART 1\r\n";
    char string_2[] = "Testing UART 2\r\n";
    
    int len_string_1 = strlen(string_1);
    int len_string_2 = strlen(string_2);

    // Se encola la cadena de prueba en la cola de TX
    uint16_t nsent_1 = UART_puts(USART1, string_1, pdMS_TO_TICKS(100));
    uint16_t nsent_2 = UART_puts(USART2, string_2, pdMS_TO_TICKS(100));

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_1 != sizeof(string_1) - 1){
        UART_puts(USART3, "Error al enviar datos por UART1\r\n", pdMS_TO_TICKS(100));
    }

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_2 != sizeof(string_2) - 1){
        UART_puts(USART3, "Error al enviar datos por UART2\r\n", pdMS_TO_TICKS(100));
    }

    uint16_t data;
    int i = 0;
    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while(UART_receive(USART1, &data, pdMS_TO_TICKS(100))){
        if(string_1[i] != (char)data || i == len_string_1) {
            break;
        }
        i++;
    }

    if(i == len_string_1)
        UART_puts(USART3, "Test UART1 runned successfully\r\n", pdMS_TO_TICKS(100));
    else
        UART_puts(USART3, "Error al recibir datos por UART1.\r\n", pdMS_TO_TICKS(100));

    i = 0;
    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while(UART_receive(USART2, &data, pdMS_TO_TICKS(100))){
        if(string_2[i] != (char)data || i == len_string_2) {
            break;
        }
        i++;
    }

    if(i == len_string_2)
        UART_puts(USART3, "Test UART2 runned successfully\r\n", pdMS_TO_TICKS(100));
    else
        UART_puts(USART3, "Error al recibir datos por UART2.\r\n", pdMS_TO_TICKS(100));
    
    vTaskDelete(NULL);
}

void taskPrintBuffer(void *args __attribute__((unused))) {

    for (;;) {
        UART_print_buffer(USART1);
        UART_print_buffer(USART2);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void taskTestUART_Semaphore(void *args __attribute__((unused))) {
    // Se espera a que haya datos disponibles en la cola de recepción
    UART_semaphore_release(USART1);

    if(UART_semaphore_take(USART1, pdMS_TO_TICKS(100)) == pdFALSE){
        UART_puts(USART3, "Test Semaphore failed. No se pudo tomar.\r\n", pdMS_TO_TICKS(100));
        UART_semaphore_release(USART1);
        vTaskDelete(NULL);
        return;
    }
    
    if(UART_semaphore_take(USART1, pdMS_TO_TICKS(500)) == pdTRUE){
        UART_puts(USART3, "Test Semaphore failed. Tomado dos veces.\r\n", pdMS_TO_TICKS(100));
        UART_semaphore_release(USART1);
        vTaskDelete(NULL);
        return;
    }

    UART_puts(USART3, "Test Semaphore runned\r\n", pdMS_TO_TICKS(100));
    // Se libera el semáforo
    UART_semaphore_release(USART1);    
    vTaskDelete(NULL);
}
