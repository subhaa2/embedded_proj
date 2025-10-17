//for pico 1

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
    // Initialize stdio for USB communication
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
    
    printf("Pico #1 (Laptop Bridge) Started!\n");
    printf("LoRa Module in Transparent Mode\n");
    printf("Send messages from laptop, they will be transmitted via LoRa\n");
    printf("Messages from LoRa will be displayed here\n\n");
    
    while (true) {
        // Check for data from laptop (USB)
        if (stdio_usb_connected()) {
            int c = getchar_timeout_us(1000); // 1ms timeout
            if (c != PICO_ERROR_TIMEOUT) {
                // Send character to LoRa
                uart_putc_raw(UART_ID, c);
                printf("Sent to LoRa: %c\n", c);
            }
        }
        
        // Check for data from LoRa
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            printf("Received from LoRa: %c\n", c);
        }
        
        sleep_ms(10);
    }
}