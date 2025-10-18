#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define RADAR_UART_ID uart1
#define RADAR_TX_PIN 4
#define RADAR_RX_PIN 5
#define RADAR_BAUD_RATE 115200

// HLK-LD2450 frame header
#define FRAME_HEADER1 0xFD
#define FRAME_HEADER2 0xFC

typedef struct {
    int16_t x;
    int16_t y;
    int16_t speed;
    float distance;
    float angle;
} Target;

void parse_target(uint8_t *data, Target *t) {
    t->x = (int16_t)(data[0] | (data[1] << 8));
    t->y = (int16_t)(data[2] | (data[3] << 8));
    t->speed = (int16_t)(data[4] | (data[5] << 8));

    // Calculate distance in mm and angle in degrees
    t->distance = sqrtf((float)(t->x * t->x + t->y * t->y));
    t->angle = atan2f((float)t->x, (float)t->y) * (180.0f / M_PI);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("--- HLK-LD2450 Human Detection ---\n");

    uart_init(RADAR_UART_ID, RADAR_BAUD_RATE);
    gpio_set_function(RADAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RADAR_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(RADAR_UART_ID, false, false);

    uint8_t buffer[64];
    int index = 0;
    bool in_frame = false;

    while (1) {
        if (uart_is_readable(RADAR_UART_ID)) {
            uint8_t byte = uart_getc(RADAR_UART_ID);

            // Detect start of frame
            if (!in_frame) {
                if (index == 0 && byte == FRAME_HEADER1) {
                    buffer[index++] = byte;
                } else if (index == 1 && byte == FRAME_HEADER2) {
                    buffer[index++] = byte;
                    in_frame = true;
                } else {
                    index = 0;
                }
                continue;
            }

            // Store data
            buffer[index++] = byte;

            // Typical frame is 47 bytes for LD2450
            if (index >= 47) {
                in_frame = false;
                index = 0;

                Target target1, target2, target3;

                // Parse targets (offsets known from datasheet)
                parse_target(&buffer[8], &target1);
                parse_target(&buffer[16], &target2);
                parse_target(&buffer[24], &target3);

                printf("1 dist: %.1f mm  Angle: %.1f deg  Speed: %d mm/s  X: %d mm  Y: %d mm\n",
                       target1.distance, target1.angle, target1.speed, target1.x, target1.y);
                printf("2 dist: %.1f mm  Angle: %.1f deg  Speed: %d mm/s  X: %d mm  Y: %d mm\n",
                       target2.distance, target2.angle, target2.speed, target2.x, target2.y);
                printf("3 dist: %.1f mm  Angle: %.1f deg  Speed: %d mm/s  X: %d mm  Y: %d mm\n\n",
                       target3.distance, target3.angle, target3.speed, target3.x, target3.y);
            }
        }
    }
}
