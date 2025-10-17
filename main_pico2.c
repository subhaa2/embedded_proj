//for pico 2

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// UART pins for LoRa communication
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0  // GP0
#define UART_RX_PIN 1  // GP1

// Mode control pins
#define M0_PIN 2
#define M1_PIN 3

// Buffer for received data
#define BUFFER_SIZE 256
char rx_buffer[BUFFER_SIZE];
int rx_count = 0;

int main() {
    // Initialize stdio for USB communication (for debugging)
    stdio_init_all();
    
    // Initialize UART for LoRa
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Set mode control pins (M0=0, M1=0 for transparent mode)
    gpio_init(M0_PIN);
    gpio_init(M1_PIN);
    gpio_set_dir(M0_PIN, GPIO_OUT);
    gpio_set_dir(M1_PIN, GPIO_OUT);
    gpio_put(M0_PIN, 0);  // LOW
    gpio_put(M1_PIN, 0);  // LOW
    
    printf("Pico #2 (Standalone) Started!\n");
    printf("LoRa Module in Transparent Mode\n");
    printf("Waiting for messages from Pico #1...\n\n");
    
    while (true) {
        // Check for data from LoRa
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            printf("Received from LoRa: %c\n", c);
            
            // Echo back a response
            if (c == 'H') {  // If received 'H' (start of "Hello")
                sleep_ms(100);  // Small delay
                uart_putc_raw(UART_ID, 'R');  // Send 'R' for "Received"
                printf("Sent response: R\n");
            }
        }
        
        sleep_ms(10);
    }
}