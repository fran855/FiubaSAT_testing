#include "FreeRTOS.h"
#include "uart1.h"
#include <stdint.h>
#include <stdbool.h>

#define SIZE_BUFFER 256  // Define el tamaño del buffer

static uint16_t buffer[SIZE_BUFFER];
static uint16_t head = 0;  // Índice de escritura
static uint16_t tail = 0;  // Índice de lectura
static bool buffer_full = false;  // Indica si el buffer está lleno

static QueueHandle_t uart1_txq; // TX queue for UART
static QueueHandle_t uart1_rxq; // RX queue for UART

SemaphoreHandle_t uart1_mutex;

static void UART1_process_data(uint16_t data);
static void buffer_write(uint16_t data);

void UART1_setup(void) {
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

    // Modo de TX-RX, sin paridad, 115200 baudios, 8 bits de datos, 1 bit de stop, sin control de flujo
    usart_set_mode(USART1,USART_MODE_TX_RX);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);

    // Habilitar la UART
    usart_enable(USART1);

    // Dentro de las fuentes de interrupción de UART, habilitar la interrupción de recepción
    usart_enable_rx_interrupt(USART1);
    // Habilitar la interrupción de UART1 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
    nvic_enable_irq(NVIC_USART1_IRQ);
    
    // Crear dos colas para los datos de TX y RX, de tamaño SIZE_BUFFER_USART * sizeof(uint16_t) bytes
    uart1_txq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));
    uart1_rxq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));

    // Crear un mutex binario para garantizar que solo una tarea a la vez acceda a la UART1
    uart1_mutex = xSemaphoreCreateBinary();
    if(uart1_mutex == NULL) {
        UART1_puts("Error al crear mutex\n");
    }
    // Inicializar el mutex en estado disponible
    xSemaphoreGive(uart1_mutex);
}

void taskUART1_transmit(void *args __attribute__((unused))) {
    uint16_t ch;
    for (;;) {
        // xQueueReceive: recibe un elemento de la cola uart1_txq: si existe, se almacena en ch y devuelve pdPASS; si no, devuelve pdFALSE. El tercer parámetro indica la cantidad máxima de tiempo que la tarea debe bloquear la espera de recibir un elemento si la cola está vacía en el momento de la llamada.
        while (xQueueReceive(uart1_txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
            // Verifica si el registro de transmisión está vacío: si no, se bloquea hasta que esté vacío
            while (!usart_get_flag(USART1,USART_SR_TXE) )
                taskYIELD(); // Yield until ready
            // Una vez vacío, se envía el byte (función bloqueante)
            usart_send_blocking(USART1,ch);
        }
        // Retardo de 50 ms antes de volver a ejecutarse
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void taskUART1_receive(void *args __attribute__((unused))) {
    int data;
    for(;;) {
        while((data = UART1_receive()) != -1) {
            UART1_process_data(data);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int UART1_receive() {
    int data;
    // Intenta recibir un dato de la cola uart1_rxq. Si no hay datos, se bloquea durante 500 ms; si hay, lo devuelve
    if (xQueueReceive(uart1_rxq, &data, pdMS_TO_TICKS(500)) == pdPASS) {
        return data;
    }
    return -1;
}

// UART1_PROCESS_DATA
// Envia un byte de datos a través de UART1 y lo almacena en el buffer
static void UART1_process_data(uint16_t data) {
    //UART3_putchar(data);
    buffer_write(data);
}

uint16_t *UART1_get_buffer(void) {
    return buffer;
}

uint16_t UART1_puts(const char *s) {
    uint16_t nsent = 0;
    // Recorre el string s hasta encontrar el caracter nulo
    for ( ; *s; s++) {
        // Añade el caracter a la cola uart1_txq. portMAX_DELAY indica que la tarea se bloqueará indefinidamente si la cola está llena
        if(xQueueSend(uart1_txq, s, portMAX_DELAY) != pdTRUE) {
            // Si falla, se resetea la cola y se devuelve la cantidad de caracteres enviados
            xQueueReset(uart1_txq);
            return nsent; // Queue full
        }
        nsent++;
    }
    return nsent;
}

void UART1_putchar(uint16_t ch) {
    xQueueSend(uart1_txq, &ch, portMAX_DELAY);
}

// USART1_ISR
// Se ejecuta automáticamente en respuesta a una interrupción generada por el hardware de USART1 cuando hay datos disponibles para ser leidos
void usart1_isr() {
    // flag USART_SR_RXNE: Receive Data Register Not Empty
    while (usart_get_flag(USART1, USART_SR_RXNE)) {
        // Leer el byte de datos recibido del registro de datos de USART1
        uint16_t data = usart_recv(USART1);
        // Añade el byte de datos a la cola de recepción desde la rutina de interrupción, sin prioridad de interrupción (NULL)
        if(xQueueSendToBackFromISR(uart1_rxq, &data, NULL) != pdTRUE) { 
            // Si falla, se resetea la cola
            xQueueReset(uart1_rxq);
        }
    }
}

bool UART1_buffer_read(uint16_t *data) {
    if (head == tail && !buffer_full) {
        // Buffer vacío
        return false;
    }

    *data = buffer[tail];
    tail = (tail + 1) % SIZE_BUFFER;
    buffer_full = false;  // Después de leer, el buffer ya no puede estar lleno
    return true;
}

static void buffer_write(uint16_t data) {
    if (buffer_full) {
        // Opcional: manejar el caso de buffer lleno, como sobrescribir el dato más antiguo.
        tail = (tail + 1) % SIZE_BUFFER;
    }

    buffer[head] = data;  // Escribe el dato en la posición de head
    head = (head + 1) % SIZE_BUFFER;

    // Verifica si el buffer está lleno
    if (head == tail) {
        buffer_full = true;
    } else {
        buffer_full = false;
    }
}

void UART1_print_buffer(void) {
    uint16_t i = tail;
    UART3_puts("Contenido del buffer 1:\r\n");

    // Si el buffer no está lleno, imprimir desde tail hasta head
    while (i != head || (i == head && buffer_full)) {
        UART3_putchar(buffer[i]);
        i = (i + 1) % SIZE_BUFFER;

        // Si el buffer estaba lleno, necesitamos asegurarnos de que
        // no imprima dos veces al dar una vuelta completa
        if (buffer_full && i == head) {
            buffer_full = false; // Resetear flag de buffer lleno
            break;
        }
    }
    UART3_putchar('\r');
    UART3_putchar('\n');
}