// for pico 1 - Laptop Bridge (DEBUG VERSION)

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// UART pins for LoRa communication
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 1 // GP1 → LoRa RX
#define UART_RX_PIN 0 // GP0 ← LoRa TX

// Mode control pins
#define M0_PIN 2
#define M1_PIN 3

// function to send movement state command to the sensor pico to move the spider robot
void SendSpiderCommand(int _command){
    char buffer[16];
    int len = snprintf(buffer, sizeof(buffer), "SC%d\n", _command);
    uart_write_blocking(UART_ID, (uint8_t *)buffer, len);
    printf("[SPIDER COMMAND SENT] SC%d\n", _command);
}

int main()
{
    // Initialize stdio for USB communication
    stdio_init_all();
    sleep_ms(2000); // Wait for USB to stabilize

    printf("\n=== Pico #1 (Laptop Bridge) DEBUG MODE ===\n");

    // Initialize UART for LoRa
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    printf("UART0 initialized at %d baud\n", BAUD_RATE);
    printf("TX Pin: GP%d → LoRa RX\n", UART_TX_PIN);
    printf("RX Pin: GP%d ← LoRa TX\n", UART_RX_PIN);

    // Set mode control pins (M0=0, M1=0 for transparent mode)
    gpio_init(M0_PIN);
    gpio_init(M1_PIN);
    gpio_set_dir(M0_PIN, GPIO_OUT);
    gpio_set_dir(M1_PIN, GPIO_OUT);
    gpio_put(M0_PIN, 0); // LOW
    gpio_put(M1_PIN, 0); // LOW
    
    printf("M0=LOW, M1=LOW (Transparent Mode)\n");
    printf("Waiting for data from Pico #2...\n\n");

    // Buffer for reassembling complete messages
    static char msg_buffer[256];
    static int msg_idx = 0;
    static uint32_t byte_count = 0;
    static uint32_t message_count = 0;  
    static absolute_time_t last_byte_time;  // Will be initialized when first byte arrives
    static bool has_received_byte = false;  // Track if we've received any bytes
    
    absolute_time_t last_status = get_absolute_time();

    while (true)
    {
        // Check for data from laptop (USB) to send to Pico 2
        if (stdio_usb_connected())
        {
            // int c = getchar_timeout_us(1000);
            // if (c != PICO_ERROR_TIMEOUT)
            // {
            //     uart_putc_raw(UART_ID, c);
            //     printf("[SENT: '%c']\n", c);
            // }
            SendSpiderCommand(1);
            sleep_ms(1000);
        }

        // Check for data from LoRa (sensor data from Pico 2)
        if (uart_is_readable(UART_ID))
        {
            char c = uart_getc(UART_ID);
            byte_count++;
            last_byte_time = get_absolute_time();
            has_received_byte = true;
            
            // // Show ALL bytes in hex for debugging
            // if (c >= 32 && c < 127) {
            //     printf("[0x%02X '%c']", c, c);
            // } else {
            //     printf("[0x%02X]", c);
            // }
            
            // Buffer characters until we get a newline
            if (c == '\n')
            {
                printf("\n>>> MESSAGE #%lu: ", ++message_count);
                msg_buffer[msg_idx] = '\0';
                if (msg_idx > 0)
                {
                    printf("%s", msg_buffer);
                }
                printf("\n\n");
                msg_idx = 0;
            }
            else if (c != '\r')
            {
                if (msg_idx < 255)
                {
                    msg_buffer[msg_idx++] = c;
                }
                else
                {
                    // Buffer overflow - reset and discard
                    printf("\n[ERROR] Message buffer overflow, resetting\n");
                    msg_idx = 0;
                }
            }
            
            fflush(stdout);
        }

        // Handle timeout: if no bytes for 100ms, consider message complete
        // This helps with fragmented messages
        absolute_time_t now = get_absolute_time();
        if (msg_idx > 0 && has_received_byte && 
            absolute_time_diff_us(last_byte_time, now) > 200000)  // 200ms timeout (increased from 100ms)
        {
            printf("\n>>> MESSAGE #%lu (TIMEOUT): ", ++message_count);
            msg_buffer[msg_idx] = '\0';
            printf("%s", msg_buffer);
            printf("\n\n");
            msg_idx = 0;
        }

        // Print status every 5 seconds
        if (absolute_time_diff_us(last_status, now) >= 5000000)
        {
            printf("\n[STATUS] Bytes received: %lu | Messages: %lu\n", byte_count, message_count);
            
            if (byte_count == 0) {
                printf("[WARNING] No data received from LoRa module!\n");
                printf("Check: 1) Pico 2 is running  2) LoRa wiring  3) Both modules on same frequency\n");
            } else if (byte_count < 10) {
                printf("[WARNING] Very few bytes received - check if Pico 2 is transmitting\n");
            }
            
            last_status = now;
            byte_count = 0;
        }

        sleep_ms(10);
    }
}