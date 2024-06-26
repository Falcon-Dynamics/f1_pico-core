// Adafruit_MCP9808.h
#ifndef _ADAFRUIT_MCP9808_H
#define _ADAFRUIT_MCP9808_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MCP9808_I2CADDR_DEFAULT 0x18 ///< I2C address
#define MCP9808_REG_CONFIG 0x01      ///< MCP9808 config register

#define MCP9808_REG_CONFIG_SHUTDOWN 0x0100   ///< shutdown config
#define MCP9808_REG_CONFIG_CRITLOCKED 0x0080 ///< critical trip lock
#define MCP9808_REG_CONFIG_WINLOCKED 0x0040  ///< alarm window lock
#define MCP9808_REG_CONFIG_INTCLR 0x0020     ///< interrupt clear
#define MCP9808_REG_CONFIG_ALERTSTAT 0x0010  ///< alert output status
#define MCP9808_REG_CONFIG_ALERTCTRL 0x0008  ///< alert output control
#define MCP9808_REG_CONFIG_ALERTSEL 0x0004   ///< alert output select
#define MCP9808_REG_CONFIG_ALERTPOL 0x0002   ///< alert output polarity
#define MCP9808_REG_CONFIG_ALERTMODE 0x0001  ///< alert output mode

#define MCP9808_REG_UPPER_TEMP 0x02   ///< upper alert boundary
#define MCP9808_REG_LOWER_TEMP 0x03   ///< lower alert boundery
#define MCP9808_REG_CRIT_TEMP 0x04    ///< critical temperature
#define MCP9808_REG_AMBIENT_TEMP 0x05 ///< ambient temperature
#define MCP9808_REG_MANUF_ID 0x06     ///< manufacture ID
#define MCP9808_REG_DEVICE_ID 0x07    ///< device ID
#define MCP9808_REG_RESOLUTION 0x08   ///< resolutin

class Adafruit_MCP9808 {
public:
    Adafruit_MCP9808();
    bool begin();
    bool init();
    float readTempC();
    float readTempF();
    uint8_t getResolution();
    void setResolution(uint8_t value);
    void shutdown_wake(bool sw);
    void shutdown();
    void wake();
    void write16(uint8_t reg, uint16_t val);
    uint16_t read16(uint8_t reg);
    void write8(uint8_t reg, uint8_t val);
    uint8_t read8(uint8_t reg);

private:
    uint16_t _sensorID = 9808; ///< ID number for temperature
};

#endif //_ADAFRUIT_MCP9808_H
