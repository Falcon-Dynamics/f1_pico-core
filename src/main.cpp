#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "ADXL345.h"

#include "f_util.h"
#include "ff.h"
#include "pico/stdlib.h"
#include "rtc.h"
#include "hw_config.h"


ADXL345 accelerometer;

void read_sensorsTask(void *pvParameters) {
    while (true) {
        printf("X: %d Y: %d Z: %d\n",
               accelerometer.getX(),
               accelerometer.getY(),
               accelerometer.getZ()
        );

        vTaskDelay(500);
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

void test_SD() {
    time_init();
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    FIL fil;
    const char *const filename = "filename.txt";
    fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
    if (f_printf(&fil, "Hello, world!\n") < 0) {
        printf("f_printf failed\n");
    }
    fr = f_close(&fil);
    if (FR_OK != fr) {
        printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }
    f_unmount("");
}

void setup() {
    accelerometer.begin(ADXL345_DEFAULT_ADDRESS, i2c_default, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
    accelerometer.setRange(ADXL345_RANGE_16_G); // set 16 g range
}

int main() {
    stdio_init_all();
    setup();

    test_SD(); // todo test the SD

    xTaskCreate(read_sensorsTask, "read_sensorsTask", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1) {};
}