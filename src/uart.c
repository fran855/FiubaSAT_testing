#include "FreeRTOS.h"
#include "uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SIZE_BUFFER 256  // Queues size

typedef struct {
    uint32_t usart;  // USART_ID 
    QueueHandle_t txq;  // Cola de transmisión
    QueueHandle_t rxq;  // Cola de recepción donde se bufferean los datos
    SemaphoreHandle_t mutex;  // Mutex para protección de acceso
    SemaphoreHandle_t semaphore; // Semáforo para señalizar datos de rxq
    int interrupciones;  // Contador de interrupciones
} uart_t;

// Definición de estructuras UART
static uart_t uart1;
static uart_t uart2;
static uart_t uart3;

// Prototipos de funciones
static BaseType_t uart_init(uart_t *uart, uint32_t usart);
static void usart_generic_isr(uint32_t usart_id);

// Manejadores de UARTs
static uart_t *get_uart(uint32_t usart_id) {
    switch (usart_id) {
        case USART1: return &uart1;
        case USART2: return &uart2;
        case USART3: return &uart3;
        default: return NULL;
    }
}

BaseType_t UART_setup(uint32_t usart, uint32_t baudrate) {
    uart_t *uart = get_uart(usart);
    if (uart == NULL) return pdFAIL;

    // Configuración del reloj y pines según el USART
    if (usart == USART1) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART1)
        rcc_periph_clock_enable(RCC_GPIOA); 
        // Habilitar el clock para USART1
        rcc_periph_clock_enable(RCC_USART1); 

        // Configurar los pines de UART1 (PA9 TX y PA10 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART1_TX, 
            GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART1_TX);

        gpio_set_mode(GPIO_BANK_USART1_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART1_RX);
        
        // Habilitar la interrupción de UART1 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART1_IRQ);
        if(uart_init(&uart1, USART1) != pdPASS) return pdFAIL;

    } else if (usart == USART2) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART2)
        rcc_periph_clock_enable(RCC_GPIOA);
        // Habilitar el clock para USART2
        rcc_periph_clock_enable(RCC_USART2);

        // Configurar los pines de UART2 (PA2 TX y PA3 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART2_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART2_TX);

        gpio_set_mode(GPIO_BANK_USART2_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART2_RX);

        // Habilitar la interrupción de UART2 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART2_IRQ);
        if(uart_init(&uart2, USART2) != pdPASS) return pdFAIL;

    } else if (usart == USART3) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART3)
        rcc_periph_clock_enable(RCC_GPIOB);
        // Habilitar el clock para USART3
        rcc_periph_clock_enable(RCC_USART3);

        // Configurar los pines de UART1 (PB10 TX y PB11 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART3_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART3_TX);

        gpio_set_mode(GPIO_BANK_USART3_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART3_RX);
        
        // Habilitar la interrupción de UART3 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART3_IRQ);
        if(uart_init(&uart3, USART3) != pdPASS) return pdFAIL;
    }

    // Configuración de USART
    usart_set_baudrate(usart, baudrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

    usart_enable(usart);
    // Dentro de las fuentes de interrupción de UART, habilitar la interrupción de recepción
    usart_enable_rx_interrupt(usart);

    return pdPASS;
}

// Inicialización de UART
static BaseType_t uart_init(uart_t *uart, uint32_t usart) {
    uart->usart = usart;  // Asigna el USART correspondiente
    uart->txq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de transmisión
    if (uart->txq == NULL) return pdFAIL;

    uart->rxq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de recepción
    if (uart->rxq == NULL) {
        vQueueDelete(uart->txq);
        return pdFAIL;
    }

    uart->mutex = xSemaphoreCreateMutex();
    if (uart->mutex == NULL) {
        vQueueDelete(uart->txq);
        vQueueDelete(uart->rxq);
        return pdFAIL;
    }
    xSemaphoreGive(uart->mutex);

    uart->semaphore = xSemaphoreCreateBinary();
    if (uart->semaphore == NULL) {
        vQueueDelete(uart->txq);
        vQueueDelete(uart->rxq);
        vSemaphoreDelete(uart->mutex);
        return pdFAIL;
    }

    uart->interrupciones = 0;
    return pdPASS;
}

void taskUART_transmit(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uint16_t ch;
    for (;;) {
        // Intentar adquirir el mutex antes de acceder a la cola de transmisión
        if (xSemaphoreTake(uart->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Recibir datos de la cola de transmisión
            while (xQueueReceive(uart->txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
                // Esperar hasta que el registro de transmisión esté vacío
                while (!usart_get_flag(uart->usart, USART_SR_TXE))
                    taskYIELD(); // Ceder la CPU hasta que esté listo
                // Enviar el byte a través de USART
                usart_send_blocking(uart->usart, ch);
            }
            // Liberar el mutex después de transmitir los datos
            xSemaphoreGive(uart->mutex);
        }
        // Esperar 50 ms antes de la siguiente iteración
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

BaseType_t UART_receive(uint32_t usart_id, uint16_t *data, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return -1;

    // Si hay datos devuelvo y sino espero xTicksToWait
    return xQueueReceive(uart->rxq, data, xTicksToWait);
}

// Reseteo buffer de recepcion de UART
BaseType_t UART_clear_rx_queue(uint32_t usart_id, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    // Aquí se usa un mutex para proteger el acceso a la cola
    if(xSemaphoreTake(uart->mutex, xTicksToWait) != pdTRUE) return pdFAIL;

    // Vaciar la cola de recepción
    if(xQueueReset(uart->rxq) != pdTRUE) {
        xSemaphoreGive(uart->mutex);
        return pdFAIL;
    }

    if(xSemaphoreGive(uart->mutex) != pdTRUE) return pdFAIL;
    
    return pdPASS;
}

// Envio cadena de caracteres a través de UART
uint16_t UART_puts(uint32_t usart_id, const char *s, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return 0;

    uint16_t nsent = 0;
    // Recorre el string s hasta encontrar el caracter nulo
    for ( ; *s; s++) {
        // Añade el caracter a la cola uart1_txq. Espera xTicksToWait (portMAX_DELAY espera indefnidamente)
        if(xQueueSend(uart->txq, s, xTicksToWait) != pdTRUE) {
            return nsent; // Queue full nsent < strlen(s)
        }
        nsent++;
    }
    return nsent;
}

// Envio de un caracter a través de UART
BaseType_t UART_putchar(uint32_t usart_id, uint16_t ch, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    return xQueueSend(uart->txq, &ch, xTicksToWait);
}

void usart1_isr(void) {
    usart_generic_isr(USART1);
}

void usart2_isr(void) {
    usart_generic_isr(USART2);
}

void usart3_isr(void) {
    usart_generic_isr(USART3);
}

// Rutina de interrupción genérica para USART
static void usart_generic_isr(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    // Incrementar el contador de interrupciones (auxiliar)
    uart->interrupciones++;

    // flag USART_SR_RXNE: Receive Data Register Not Empty
    while (usart_get_flag(uart->usart, USART_SR_RXNE)) {
        // Leer el byte de datos recibido del registro de datos del USART correspondiente
        uint16_t data = usart_recv_blocking(uart->usart);
        // Añade el byte de datos a la cola de recepción desde la rutina de interrupción
        if (xQueueSendToBackFromISR(uart->rxq, &data, NULL) == pdTRUE) { 
            // Dar el semáforo para indicar que hay datos disponibles en la cola
            xSemaphoreGiveFromISR(uart->semaphore, NULL);
        }
    }
}

// Imprimir los elementos en uart->rxq de UART (auxiliar)
void UART_print_buffer(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;
    
    // Calcular el número de elementos en la cola
    UBaseType_t items_in_queue = uxQueueMessagesWaiting(uart->rxq);
    
    if (items_in_queue == 0) {
        UART_puts(USART3, "La cola está vacía.\r\n", pdMS_TO_TICKS(500));
        return;
    }

    switch (usart_id)
    {
    case USART1:
        UART_puts(USART3, "RXQ USART 1: ", pdMS_TO_TICKS(500));
        break;
    
    case USART2:
        UART_puts(USART3, "RXQ USART 2: ", pdMS_TO_TICKS(500));
        break;

    case USART3:
        UART_puts(USART3, "RXQ USART 3: ", pdMS_TO_TICKS(500));
        break;
    
    default:
        break;
    }

    uint16_t data;
    for (UBaseType_t i = 0; i < items_in_queue; i++) {
        if (xQueuePeek(uart->rxq, &data, 0) == pdTRUE) {
            UART_putchar(USART3, data, pdMS_TO_TICKS(500));
        }
        // Sacar el siguiente elemento para avanzar en la cola
        xQueueReceive(uart->rxq, &data, 0);
        // Volver a poner el elemento para mantener la cola intacta
        xQueueSendToBack(uart->rxq, &data, 0);
    }
    UART_putchar(USART3, '\r', pdMS_TO_TICKS(500));
    UART_putchar(USART3, '\n', pdMS_TO_TICKS(500));
}

// Wrapper SemaphoreTake
BaseType_t UART_semaphore_take(uint32_t usart_id, TickType_t ticks_to_wait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    return xSemaphoreTake(uart->semaphore, ticks_to_wait);
}

// Wrapper SemaphoreGive
BaseType_t UART_semaphore_release(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;
    
    return xSemaphoreGive(uart->semaphore);
}