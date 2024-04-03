// Adafruit_MCP9808.cpp
#include "MCP9808.h"

Adafruit_MCP9808::Adafruit_MCP9808() {}

bool Adafruit_MCP9808::begin() {
    // Initialize I2C interface
    i2c_init(i2c0, 100000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    return init();
}

bool Adafruit_MCP9808::init() {
    uint16_t manuf_id = read16(MCP9808_REG_MANUF_ID);
    uint16_t device_id = read16(MCP9808_REG_DEVICE_ID);
    if (manuf_id != 0x0054 || device_id != 0x0400)
        return false;

    write16(MCP9808_REG_CONFIG, 0x0);
    return true;
}

float Adafruit_MCP9808::readTempC() {
    float temp = NAN;
    uint16_t t = read16(MCP9808_REG_AMBIENT_TEMP);
    if (t != 0xFFFF) {
        temp = t & 0x0FFF;
        temp /= 16.0;
        if (t & 0x1000)
            temp -= 256;
    }
    return temp;
}

float Adafruit_MCP9808::readTempF() {
    float temp = readTempC();
    if (!isnan(temp)) {
        temp = temp * 9.0 / 5.0 + 32;
    }
    return temp;
}

uint8_t Adafruit_MCP9808::getResolution() {
    return read8(MCP9808_REG_RESOLUTION);
}

void Adafruit_MCP9808::setResolution(uint8_t value) {
    write8(MCP9808_REG_RESOLUTION, value & 0x03);
}

void Adafruit_MCP9808::shutdown_wake(bool sw) {
    uint16_t conf_shutdown;
    uint16_t conf_register = read16(MCP9808_REG_CONFIG);
    if (sw == true) {
        conf_shutdown = conf_register | MCP9808_REG_CONFIG_SHUTDOWN;
        write16(MCP9808_REG_CONFIG, conf_shutdown);
    }
    if (sw == false) {
        conf_shutdown = conf_register & ~MCP9808_REG_CONFIG_SHUTDOWN;
        write16(MCP9808_REG_CONFIG, conf_shutdown);
    }
}

void Adafruit_MCP9808::shutdown() {
    shutdown_wake(true);
}

void Adafruit_MCP9808::wake() {
    shutdown_wake(false);
    sleep_ms(260);
}

void Adafruit_MCP9808::write16(uint8_t reg, uint16_t val) {
    uint8_t data[3] = {reg, val >> 8, val & 0xFF};
    i2c_write_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 3, false);
}

uint16_t Adafruit_MCP9808::read16(uint8_t reg) {
    uint8_t data[2] = {reg};
    i2c_write_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 1, true);
    i2c_read_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 2, false);
    return (data[0] << 8) | data[1];
}

void Adafruit_MCP9808::write8(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    i2c_write_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 2, false);
}

uint8_t Adafruit_MCP9808::read8(uint8_t reg) {
    uint8_t data[1] = {reg};
    i2c_write_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 1, true);
    i2c_read_blocking(i2c0, MCP9808_I2CADDR_DEFAULT, data, 1, false);
    return data[0];
}
