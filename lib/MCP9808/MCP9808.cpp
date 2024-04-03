

Adafruit_MCP9808::Adafruit_MCP9808() {}

/*!
 *    @brief  Setups the HW
 *    @param  *theWire
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MCP9808::begin(TwoWire *theWire) {
    return begin(MCP9808_I2CADDR_DEFAULT, theWire);
}