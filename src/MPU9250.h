#ifndef MPU9250_h
#define MPU9250_h

#include "util/SerialMessage.h"

/* Full scale range settings */
enum gyroFSRVal {
    GFSR_250DPS = 0,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accFSRVal {
    AFSR_2G = 0,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

/* Lowpass filter configuration */
enum lpfVal {
    LPF_250HZ = 0,
    LPF_184HZ,
    LPF_92HZ,
    LPF_41HZ,
    LPF_20HZ,
    LPF_10HZ,
    LPF_5HZ,
    LPF_3600HZ
};

/* x, y, z data structs */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SRAWDATA;

typedef struct {
    double x;
    double y;
    double z;
} SSCALEDDATA;

class MPU9250 {
private:

    uint8_t magI2CAddr;
    bool gyroEnabled, accEnabled, magEnabled;
    uint8_t gyroFSR, accFSR, magFSR, lpf;
    double gyroSensitivity, accSensitivity, magSensitivity;
    bool magBypass;
    bool useInterruptPin;
    int interruptPin;
    
    int8_t reset();
    int8_t resetMag();

    void setLPF(uint8_t setLPF);
    void setGyroFSR(uint8_t value);
    void setAccFSR(uint8_t value);
    void setSampleRateDivider(uint8_t div);

public:

    SRAWDATA rawGyro, rawAcc, rawMag;
    SSCALEDDATA scaledGyro, scaledAcc, scaledMag;
    SSCALEDDATA gyroBias, accOffset, accScale, magAdj, magOffset;

	MPU9250(bool setGyroEnabled = true, bool setAccEnabled = true, bool setMagEnabled = true);

	int8_t init(gyroFSRVal setGyroFSRValue = GFSR_250DPS, accFSRVal setAccFSRValue = AFSR_2G, lpfVal setLPFValue = LPF_250HZ);
    void calibrateGyro(int sampleCount = 1280);
    void calibrateAcc(int sampleCount = 10);
    void calibrateMag(int sampleCount = 1280);
    bool dataReady();
    bool magDataReady();
    
    void printRegister(const char *regName, uint8_t regVal);

    void enableInterruptPin(int pin);
    void setMagBypass(bool enable);
    
    void readGyro(bool scale = true);
    void readAcc(bool scale = true);
    void readAccGyro(bool scale = true);
    void readMag(bool scale = true);
};

#endif