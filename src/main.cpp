#include <iostream>
#include <sstream>

#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>
#include "pico/stdlib.h"

#include "ADXL345.h"

#include "f_util.h"
#include "ff.h"
#include "pico/stdlib.h"
#include "rtc.h"
#include "hw_config.h"

#include <RadioLib.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "simple.pb.h"

#include <MPL3115A2.h>

using namespace std;

ADXL345 accelerometer;
Adafruit_MPL3115A2 baro;

const TickType_t xpollRate = 100 / portTICK_PERIOD_MS;
const TickType_t xblinkRate = 100 / portTICK_PERIOD_MS;


void read_sensorsTask(void *pvParameters) {
    while (true) {
        printf("X: %f Y: %f Z: %f\n",
               accelerometer.getX() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD,
               accelerometer.getY() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD,
               accelerometer.getZ() * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD
        );

        vTaskDelay(xpollRate); //todo make absolute time delay
    }
}


void read_BarosensorsTask(void *pvParameters) {
    while (true) {
        printf("altitude: %f\n",
                baro.getAltitude()
        );

        vTaskDelay(xpollRate); //todo make absolute time delay
    }
}

void led_task(void *pvParameters) {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(xpollRate);
        gpio_put(LED_PIN, 0);
        vTaskDelay(xblinkRate);
    }
}

void test_SD() {
    sleep_ms(10000);
    printf("starting file write\n");
    time_init();
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    FIL fil;

    for(;;) {

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
        
        printf("file all good!!!\n");
    }

    f_unmount("");

    printf("file all good!!!\n");
}

void testPB(){
    uint8_t buffer[128];
    size_t message_length;
    bool status;

    SimpleMessage message = SimpleMessage_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    /* Fill in the lucky number */
    message.lucky_number = 13;

    /* Now we are ready to encode the message! */
    status = pb_encode(&stream, SimpleMessage_fields, &message);
    message_length = stream.bytes_written;

    std::stringstream ss;
    pb_ostream_t ostream = pb_ostream_from_buffer(reinterpret_cast<pb_byte_t *>(&ss),
                                                  reinterpret_cast<size_t>(pb_encode_string));
    if (!pb_encode_string(&ostream, buffer, message_length)) {
        std::cerr << "Encoding to string failed" << std::endl;
    }
    std::string encoded_string = ss.str();
    std::cout << "Encoded string: " << encoded_string << std::endl;
}

void setup() {
//    accelerometer.begin(ADXL345_DEFAULT_ADDRESS, i2c_default, PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN);
//    accelerometer.setRange(ADXL345_RANGE_2_G); // set 2 g range

//    sleep_ms(10000);
//    if(!baro.begin()) {
//        printf("Could not find sensor. Check wiring.");
//        while(1);
//    }
//
//
//    // use to set sea level pressure for current location
//    // this is needed for accurate altitude measurement
//    // STD SLP = 1013.26 hPa
//    baro.setSeaPressure(1022);
}

int main() {
    stdio_init_all();
    setup();

    test_SD(); // todo test the SD

//    xTaskCreate(read_sensorsTask, "read_sensorsTask", 256, NULL, 1, NULL);
//    xTaskCreate(read_BarosensorsTask, "read_BarosensorsTask", 256, NULL, 1, NULL);
    xTaskCreate(led_task, "led_task", 256, NULL, 2, NULL);
    vTaskStartScheduler();


    while (1) {};
}