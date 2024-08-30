#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// Handle para la tarea del parpadeo
static TaskHandle_t blink_handle;
 
/* Acá estaría la tarea asignada al periférico conectado a la interfaz USART3 */
static void taskUART3_receive(uint32_t usart_id) {
    uint16_t data;
    for (;;) {
        // Esperar a que el semáforo indique que hay datos disponibles
        if (UART_semaphore_take(usart_id, portMAX_DELAY) == pdTRUE) {
            // Procesar todos los datos en la cola
            while (UART_receive(usart_id, &data, pdMS_TO_TICKS(100))) {
                // Aquí puedes manejar el dato recibido (por ejemplo, almacenarlo o procesarlo)
                UART_putchar(USART3, data, pdMS_TO_TICKS(100));
            }
            // Liberar el semáforo después de procesar los datos
            UART_semaphore_release(usart_id);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

/* Main loop donde arranca el programa */
int main(void) {
    // Setup main clock, using external 8MHz crystal 
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Inicialización del LED para el blink
    blink_setup();

    // Inicialización de UARTs con sus baudrates
    if(UART_setup(USART1, 115200) != pdPASS) return -1;
    if(UART_setup(USART2, 115200) != pdPASS) return -1;
    if(UART_setup(USART3, 115200) != pdPASS) return -1;

    // Crear tarea para parpadear el LED
    xTaskCreate(taskBlink, "LED", 100, NULL, 2, &blink_handle);  // Crear tarea para parpadear el LED

    // Creación de tareas genéricas para transmisión UART
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART1 TX", 128, (void *)USART1, 2, NULL);
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART2 TX", 128, (void *)USART2, 2, NULL);
    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART3 TX", 128, (void *)USART3, 2, NULL);

    // Creación de tareas genéricas para recepción UART
    xTaskCreate((TaskFunction_t)taskUART3_receive, "UART3 RX", 128, (void *)USART3, 2, NULL);

    // Crear tareas para Test
    xTaskCreate(taskTestUART_Semaphore, "Test_Semaphore", 100, NULL, 2, NULL);  // Crear tarea para Test
    xTaskCreate(taskTest, "Test", 100, NULL, 2, NULL);  // Crear tarea para Test
    xTaskCreate(taskPrintBuffer, "Print_buffer", 100, NULL, 2, NULL);  // Crear tarea para Test

    // Start RTOS Task scheduler
	vTaskStartScheduler();

    // The task scheduler is blocking, so we should never come here...
	for (;;);
    
	return 0;
}