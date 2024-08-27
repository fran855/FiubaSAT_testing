#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "test.h"

#include "uart1.h"
#include "uart2.h"
#include "uart3.h"

#include "libopencm3/stm32/rcc.h"

void taskTest(void *args __attribute__((unused))){
    // Definición de la cadena de prueba
    char string_1[] = "Testing UART 1\r\n";
    char string_2[] = "Testing UART 2\r\n";
    
    int len_string_1 = strlen(string_1);
    int len_string_2 = strlen(string_2);

    // Se encola la cadena de prueba en la cola de TX
    uint16_t nsent_1 = UART1_puts(string_1);
    uint16_t nsent_2 = UART2_puts(string_2);

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_1 != sizeof(string_1) - 1){
        UART3_puts("Error al enviar datos por UART1\r\n");
        vTaskDelete(NULL);
    }

    // Se verifica que la cantidad de caracteres enviados sea la correcta
    if(nsent_2 != sizeof(string_2) - 1){
        UART3_puts("Error al enviar datos por UART2\r\n");
        vTaskDelete(NULL);
    }

    // Se espera un segundo
    vTaskDelay(pdMS_TO_TICKS(1000));

    int data;
    int i = 0;
    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while((data = UART1_receive()) != -1){
        if(string_1[i] != (char)data || i == len_string_1) {
            UART3_puts("Error al recibir datos por UART1.\r\n");
        }
        i++;
    }

    if(data == -1)
        UART3_puts("Test UART1 runned successfully\r\n");

    // Recibe los datos de la cola de RX y los compara con la cadena de prueba
    while((data = UART1_receive()) != -1){
        if(string_1[i] != (char)data || i == len_string_2) {
            UART3_puts("Error al recibir datos por UART2.\r\n");
        }
        i++;
    }

    if(data == -1)
        UART3_puts("Test UART2 runned successfully\r\n");
    
    vTaskDelete(NULL);
}

void taskPrintBuffer(void *args __attribute__((unused))) {
    for (;;) {
        UART1_print_buffer();
        UART2_print_buffer();
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

