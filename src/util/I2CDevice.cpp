#include <Wire.h>

#include "I2CDevice.h"

/* I2C Device Communication */
uint8_t readI2CReg(uint8_t addr, uint8_t reg) {

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false); //Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(addr, (uint8_t)1);
    uint8_t value = Wire.read();
    Wire.endTransmission();

    return value;
}

void readI2CRegs(uint8_t addr, uint8_t reg, uint8_t count, uint8_t dest[]) {
    uint8_t i = 0;

    Wire.beginTransmission(addr);   // Initialize the Tx buffer
    Wire.write(reg);                // Put slave register address in Tx buffer
    Wire.endTransmission(false);    // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(addr, count);  // Read bytes from slave register address

    while (Wire.available()) {
        // Put read results in the Rx buffer
        dest[i++] = Wire.read();   
    }
}

void writeI2CReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}