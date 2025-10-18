//for pico 1 - Laptop Bridge

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
    printf("Receiving sensor data from Pico #2...\n");
    printf("Type 'hello' and press Enter to send to Pico #2\n\n");
    
    // Send initial hello message after 3 seconds
    sleep_ms(3000);
    uart_puts(UART_ID, "Hello from Laptop!\n");
    printf(">>> Sent initial greeting to Pico #2\n\n");
    
    absolute_time_t last_hello = get_absolute_time();
    
    while (true) {
        // Check for data from laptop (USB) to send to Pico 2
        if (stdio_usb_connected()) {
            int c = getchar_timeout_us(1000);
            if (c != PICO_ERROR_TIMEOUT) {
                uart_putc_raw(UART_ID, c);
                printf("%c", c);  // Echo to console
            }
        }
        
        // Check for data from LoRa (sensor data from Pico 2)
        if (uart_is_readable(UART_ID)) {
            char c = uart_getc(UART_ID);
            printf("%c", c);  // Display sensor data on laptop
        }
        
        // Send periodic "hello" message every 5 seconds
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_hello, now) >= 5000000) {  // 5 seconds
            uart_puts(UART_ID, "Hello from Laptop!\n");
            printf(">>> Sent periodic greeting to Pico #2\n");
            last_hello = now;
        }
        
        sleep_ms(10);
    }
}