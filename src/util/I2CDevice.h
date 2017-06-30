#ifndef I2CDevice_h
#define I2CDevice_h

    //Internal register functions
    void writeI2CReg(uint8_t addr, uint8_t reg, uint8_t value);
    uint8_t readI2CReg(uint8_t addr, uint8_t reg);
    void readI2CRegs(uint8_t addr, uint8_t startReg, uint8_t count, uint8_t dest[]);

#endif