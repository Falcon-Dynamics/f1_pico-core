#include "MPL3115A2.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/*!
    @brief  Instantiates a new MPL3115A2 class
*/
Adafruit_MPL3115A2::Adafruit_MPL3115A2() {}

/*!
 *   @brief  Setups the HW (reads coefficients values, etc.)
 *   @param  twoWire
 *           Optional TwoWire I2C object
 *   @return true on successful startup, false otherwise
 */
bool Adafruit_MPL3115A2::begin() {
    // Initialize I2C interface
    i2c_init(i2c0, 100000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);

    // Initialize I2C device
    if (!i2c_write_blocking(i2c0, MPL3115A2_ADDRESS, NULL, 0, true)) // Check if device is present
        return false;

    // Sanity check
    uint8_t whoami = read8(MPL3115A2_WHOAMI);
    if (whoami != 0xC4) {
        return false;
    }

    // Software reset
    write8(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
    while (read8(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST)
        sleep_ms(10);

    // Set oversampling and altitude mode
    currentMode = MPL3115A2_ALTIMETER;
    _ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
    write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

    // Enable data ready events for pressure/altitude and temperature
    write8(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
                                  MPL3115A2_PT_DATA_CFG_PDEFE |
                                  MPL3115A2_PT_DATA_CFG_DREM);

    return true;
}

/*!
 *  @brief  Get barometric pressure
 *  @return pressure reading as a floating point value in hPa
 */
float Adafruit_MPL3115A2::getPressure() {
    if (currentMode != MPL3115A2_BAROMETER)
        setMode(MPL3115A2_BAROMETER);
    startOneShot();
    while (!conversionComplete())
        sleep_ms(10);
    return getLastConversionResults(MPL3115A2_PRESSURE);
}

/*!
 *  @brief  Get altitude
 *  @return altitude reading as a floating-point value in meters
 */
float Adafruit_MPL3115A2::getAltitude() {
    if (currentMode != MPL3115A2_ALTIMETER)
        setMode(MPL3115A2_ALTIMETER);
    startOneShot();
    while (!conversionComplete())
        sleep_ms(10);
    return getLastConversionResults(MPL3115A2_ALTITUDE);
}

/*!
 *  @brief  Get the altitude offset
 *  @return Offset value in meters
 */
int8_t Adafruit_MPL3115A2::getAltitudeOffset(void) {
    return int8_t(read8(MPL3115A2_OFF_H));
}

/*!
 *  @brief  Set the altitude offset
 *  @param offset Offset value in meters, from -127 to 128
 */
void Adafruit_MPL3115A2::setAltitudeOffset(int8_t offset) {
    write8(MPL3115A2_OFF_H, uint8_t(offset));
}

/*!
 *  @brief  Set the local sea level pressure
 *  @param SLP sea level pressure in hPa
 */
void Adafruit_MPL3115A2::setSeaPressure(float SLP) {
    // Multiply by 100 to convert hPa to Pa
    // Divide by 2 to convert to 2 Pa per LSB
    // Convert to integer
    uint16_t bar = SLP * 50;

    // Write result to register
    uint8_t buffer[3];
    buffer[0] = MPL3115A2_BAR_IN_MSB;
    buffer[1] = bar >> 8;
    buffer[2] = bar & 0xFF;
    i2c_write_blocking(i2c0, MPL3115A2_ADDRESS, buffer, 3, false);
}

/*!
 *  @brief  Get temperature
 *  @return temperature reading as a floating-point value in degC
 */
float Adafruit_MPL3115A2::getTemperature() {
    startOneShot();
    while (!conversionComplete())
        sleep_ms(10);
    return getLastConversionResults(MPL3115A2_TEMPERATURE);
}

/*!
 *  @brief Set measurement mode.
 *  @param mode The measurement mode. Can be MPL3115A2_BAROMETER or
 * MPL3115A2_ALTIMETER.
 */
void Adafruit_MPL3115A2::setMode(mpl3115a2_mode_t mode) {
    // Assumes STANDBY mode
    _ctrl_reg1.reg = read8(MPL3115A2_CTRL_REG1);
    _ctrl_reg1.bit.ALT = mode;
    write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);
    currentMode = mode;
}

/*!
 *  @brief Initiate a one-shot measurement.
 */
void Adafruit_MPL3115A2::startOneShot(void) {
    // Wait for one-shot to clear before proceeding
    _ctrl_reg1.reg = read8(MPL3115A2_CTRL_REG1);
    while (_ctrl_reg1.bit.OST) {
        sleep_ms(10);
        _ctrl_reg1.reg = read8(MPL3115A2_CTRL_REG1);
    }
    // Initiate one-shot measurement
    _ctrl_reg1.bit.OST = 1;
    write8(MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);
}

/*!
 *  @brief Check for measurement conversion completion.
 *  @return true if conversion is complete, otherwise false.
 */
bool Adafruit_MPL3115A2::conversionComplete(void) {
    // PTDR bit works for either pressure or temperature
    // 0: No new set of data ready
    // 1: A new set of data is ready
    return ((read8(MPL3115A2_REGISTER_STATUS) & MPL3115A2_REGISTER_STATUS_PTDR) !=
            0);
}

/*!
 *  @brief Get results from last measurement.
 *  @param value Measurement value, can be MPL3115A2_PRESSURE,
 * MPL3115A2_ALTITUDE, or MPL3115A2_TEMPERATURE
 *  @return The measurement value.
 */

//todo missing half of code