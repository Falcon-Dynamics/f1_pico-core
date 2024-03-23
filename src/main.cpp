#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "ADXL345_RP2040/ADXL345.h"


ADXL345 accelerometer;

void read_sensorsTask(void *pvParameters) {
    while (true) {
        printf("X: %d Y: %d Z: %d\n",
               accelerometer.getX(),
               accelerometer.getY(),
               accelerometer.getZ()
        );

        vTaskDelay(50);
    }
}

void led_task(void *pvParameters) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(100);
        gpio_put(LED_PIN, 0);
        vTaskDelay(100);
    }
}

void setup() {
    accelerometer.begin(ADXL345_DEFAULT_ADDRESS, i2c_default, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
    accelerometer.setRange(ADXL345_RANGE_16_G); // set 16 g range
}

int main() {
    stdio_init_all();
    setup();

    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1) {};
}