#include <Arduino.h>
#include <Wire.h>
#include <float.h>
//#include <inttypes.h>

#include "util/I2CDevice.h"
#include "util/AccCalibration.h"
#include "MPU9250RegisterMap.h"

#include "MPU9250.h"

/*
 * Todo:
 * -Implement mag access via I2C master.
 * -Implement self-test functionality.
 * -Implement temperature readings.
 * -Implement disabling of internal gyro offset settings.
 */
MPU9250::MPU9250(bool setGyroEnabled, bool setAccEnabled, bool setMagEnabled) {
    gyroEnabled = setGyroEnabled;
    accEnabled = setAccEnabled;
    magEnabled = setMagEnabled;

    //Initialize member variables to default
    gyroFSR = GFSR_250DPS;
    accFSR = AFSR_2G;
    lpf = LPF_250HZ;

    //Disable interrupt pin by default
    useInterruptPin = false;
    interruptPin = 0;

    //Enable mag bypass by default
    magBypass = magEnabled;

    //Zero out the mag address - we'll determine it later
    magI2CAddr = 0;

    //Zero out accelerometer offsets and scales
    accOffset.x = 0;
    accOffset.y = 0;
    accOffset.z = 0;

    accScale.x = 0;
    accScale.y = 0;
    accScale.z = 0;

    magOffset.x = 0;
    magOffset.y = 0;
    magOffset.z = 0;
}

int8_t MPU9250::reset() {
    //Set the H_RESET bit
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x80);

    //Delay until reset has completed
    unsigned long resetTime = millis();
    unsigned long readTime = resetTime;
    uint8_t regVal = 0x00;
    do {
        //1 second reset timeout
        if (readTime - resetTime > 1000) {
            return failMessage("MPU9250 reset timed out.");
        }

        delay(100);
        readTime = millis();

        //Reset is complete when MSB is reset
        regVal = readI2CReg(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1);
    } while (regVal & 0x80);

    return 0;
}

int8_t MPU9250::resetMag() {
    //Set the H_RESET bit
    writeI2CReg(magI2CAddr, AK8963_CNTL2, 0x01);

    //Delay until reset has completed
    unsigned long resetTime = millis();
    unsigned long readTime = resetTime;
    uint8_t regVal = 0x00;
    do {
        //1 second reset timeout
        if (readTime - resetTime > 1000) {
            return failMessage("AK8963 reset timed out.");
        }

        delay(100);
        readTime = millis();

        //Reset is complete when MSB is reset
        regVal = readI2CReg(magI2CAddr, AK8963_CNTL2);
    } while (regVal & 0x01);

    return 0;
}

/* Applies only to gyro and acc if mag is disabled. */
bool MPU9250::dataReady() {
    return useInterruptPin ? digitalRead(interruptPin) : (readI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_STATUS) & 0x01);
}

/* Only use this if magBypass is enabled! */
bool MPU9250::magDataReady() {
    return magBypass ? readI2CReg(magI2CAddr, AK8963_ST1) : false;
}

void MPU9250::setLPF(uint8_t setLPF) {
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_CONFIG, setLPF);
}

void MPU9250::setMagBypass(bool enable) {
    uint8_t intPinCfg = readI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG);
    uint8_t userCtrl = readI2CReg(MPU9250_I2C_ADDR, MPU9250_USER_CTRL);

    if (enable) {

        //Turn off I2C master
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_I2C_MST_CTRL, 0x00);

        //Disable I2C Master
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_USER_CTRL, userCtrl & ~(0x20));
        delay(3);

        //Set bypass bit
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, intPinCfg | 0x02);

    } else {

        //Clear bypass bit
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, intPinCfg & ~(0x02));

        //Enable I2C Master
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_USER_CTRL, userCtrl & 0x20);
        delay(3);

        //Turn on I2C master
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_I2C_MST_CTRL, 0x80);
    }
}

void MPU9250::enableInterruptPin(int pin) {
    useInterruptPin = true;
    interruptPin = pin;

    //Active high, push-pull, latching, clear on read
    uint8_t intPinCfg = readI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG);
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, intPinCfg | 0x30);

    //Enable data ready interrupt
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_INT_ENABLE, 0x01);
}

void MPU9250::setGyroFSR(uint8_t value) {
    uint8_t regVal = readI2CReg(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG);

    //Clear the FSR bits [4:3] and then set them
    regVal &= ~(0x11 << 3);
    regVal |= value << 3;

    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250::setAccFSR(uint8_t value) {
    uint8_t regVal = readI2CReg(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG);

    //Clear the FSR bits [4:3] and then set them
    regVal &= ~(0x11 << 3);
    regVal |= value << 3;

    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250::setSampleRateDivider(uint8_t div) {
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_SMPLRT_DIV, div);
}

void MPU9250::printRegister(const char *regName, uint8_t regVal) {
    snprintf(printMsgBuf, PRINT_MAX_BUF, "%s: 0x%02X", regName, regVal);
    printMessage(printMsgBuf);
}

int8_t MPU9250::init(gyroFSRVal setGyroFSRValue, accFSRVal setAccFSRValue, lpfVal setLPFValue) {

    gyroFSR = setGyroFSRValue;
    accFSR = setAccFSRValue;
    lpf = setLPFValue;

    //Calculate gyro, acc, and mag sensitivities
    switch (gyroFSR) {
        case GFSR_250DPS:
            gyroSensitivity = 500.0 / 65536.0;
            break;
        case GFSR_500DPS:
            gyroSensitivity = 1000.0 / 65536.0;
            break;
        case GFSR_1000DPS:
            gyroSensitivity = 2000.0 / 65536.0;
            break;
        case GFSR_2000DPS:
            gyroSensitivity = 4000.0 / 65536.0;
            break;
        default:
            printMessage("Warning: Invalid gyroFSR. Setting to 250DPS.");
            gyroSensitivity = 500.0 / 65536.0;
    }

    switch (accFSR) {
        case AFSR_2G:
            accSensitivity = 4.0 / 65536.0;
            break;
        case AFSR_4G:
            accSensitivity = 8.0 / 65536.0;
            break;
        case AFSR_8G:
            accSensitivity = 16.0 / 65536.0;
            break;
        case AFSR_16G:
            accSensitivity = 32.0 / 65536.0;
            break;
        default:
            printMessage("Warning: Invalid accFSR. Setting to 2G.");
            accSensitivity = 4.0 / 65536.0;
    }

    //Only 1 mag sensitivity
    magSensitivity = 9824.0 / 65536.0;

    uint8_t whoAmI = readI2CReg(MPU9250_I2C_ADDR, MPU9250_WHOAMI);
    if (whoAmI != MPU9250_WHOAMI_VAL) {
        snprintf(printMsgBuf, PRINT_MAX_BUF, "MPU9250 WHOAMI validation failed! (0x%02X != 0x%02X)", whoAmI, MPU9250_WHOAMI_VAL);
        return failMessage(printMsgBuf);
    }

    //Reset the chip (and all register values)
    if (reset() < 0) {
        return -1;
    }

    //No sample rate division (1:1)
    setSampleRateDivider(0);

    //Turn off FIFO
    writeI2CReg(MPU9250_I2C_ADDR, MPU9250_FIFO_EN, 0x00);

    if (gyroEnabled) {
        setGyroFSR(gyroFSR);
        /*
        //Print gyro offsets
        readI2CRegs(MPU9250_I2C_ADDR, XG_OFFSET_H, 6, &rawDataBuf[0]);
        int16_t gyroOffsetX = (int16_t)(((int16_t)rawDataBuf[0] << 8) | rawDataBuf[1]);
        int16_t gyroOffsetY = (int16_t)(((int16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
        int16_t gyroOffsetZ = (int16_t)(((int16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

        snprintf(printMsgBuf, PRINT_MAX_BUF, "Gyro Offsets: %d, %d, %d", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
        printMessage(printMsgBuf);
        */
    } else {
        //Set PWR_MGMT_2 [0:2] to disable gyro
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_2, 0x07);
    }

    if (accEnabled) {
        setAccFSR(accFSR);
    } else {
        //Set PWR_MGMT_2 [3:5] to zero to disable acc
        uint8_t tmp = readI2CReg(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_2);
        writeI2CReg(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_2, tmp | 0x38);
    }

    /*
     *  Set the requested Gyro/Acc LPF cutoff frequency.
     *  Note: This also configures the Gyro ODR
     *  via page 13 of the register map document.
     */
    setLPF(lpf);

    if (magEnabled) {
        //Enable mag bypass
        setMagBypass(magBypass);

        if (magBypass) {
            //Find the mag's I2C address (depends on CAD0/1 pins)
            for (uint8_t addr = 0x0C; addr <= 0x0F; addr++) {
                //Verify mag device ID
                uint8_t magDevID = readI2CReg(addr, AK8963_WIA);
                if (magDevID == AK8963_DEVICE_ID) {
                    magI2CAddr = addr;
                    snprintf(printMsgBuf, PRINT_MAX_BUF, "AK8963 bypass enabled (I2C Address 0x%02X)", magI2CAddr);
                    printMessage(printMsgBuf);
                    break;
                }
            }

            //If the mag wasn't found, error out
            if (magI2CAddr == 0) {
                return failMessage("AK8963 bypass failed - address not found.");
            }

            resetMag();

            //Configure 16-bit access with Continuous Measurement mode 2 (100 HZ)
            writeI2CReg(magI2CAddr, AK8963_CNTL1, 0x16);

            //Get mag sensitivity adjustment values
            magAdj.x = (double)(((int8_t)readI2CReg(magI2CAddr, AK8963_ASAX) - 128) / 256.0) + 1.0;
            magAdj.y = (double)(((int8_t)readI2CReg(magI2CAddr, AK8963_ASAY) - 128) / 256.0) + 1.0;
            magAdj.z = (double)(((int8_t)readI2CReg(magI2CAddr, AK8963_ASAZ) - 128) / 256.0) + 1.0;
        } else {
            //Todo: I2C Master setup
        }
    }

    printMessage("MPU9250 initialization complete.");
    return 0;
}

void MPU9250::readAccGyro(bool scale) {
    uint8_t rawDataBuf[14];
    readI2CRegs(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, 14, &rawDataBuf[0]);

    rawAcc.x = (int16_t)(((int16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
    rawAcc.y = (int16_t)(((int16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawAcc.z = (int16_t)(((int16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    rawGyro.x = (int16_t)(((int16_t)rawDataBuf[8] << 8) | rawDataBuf[9]);
    rawGyro.y = (int16_t)(((int16_t)rawDataBuf[10] << 8) | rawDataBuf[11]);
    rawGyro.z = (int16_t)(((int16_t)rawDataBuf[12] << 8) | rawDataBuf[13]);
}

void MPU9250::readGyro(bool scale) {
    uint8_t rawDataBuf[6];
    readI2CRegs(MPU9250_I2C_ADDR, MPU9250_GYRO_XOUT_H, 6, &rawDataBuf[0]);

    //Read the raw data into signed, 16-bit values
    rawGyro.x = (int16_t)(((int16_t)rawDataBuf[0] << 8) | rawDataBuf[1]);
    rawGyro.y = (int16_t)(((int16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawGyro.z = (int16_t)(((int16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    if (scale) {
        //Scale all values to double precision
        scaledGyro.x = ((double)rawGyro.x * gyroSensitivity) - gyroBias.x;
        scaledGyro.y = ((double)rawGyro.y * gyroSensitivity) - gyroBias.y;
        scaledGyro.z = ((double)rawGyro.z * gyroSensitivity) - gyroBias.z;
    }
}

void MPU9250::readAcc(bool scale) {
    //Read the six raw data registers sequentially into data array
    uint8_t rawDataBuf[6];
    readI2CRegs(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, 6, &rawDataBuf[0]);

    //Convert the MSB and LSB into a signed 16-bit value
    rawAcc.x = (uint16_t)(((uint16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
    rawAcc.y = (uint16_t)(((uint16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
    rawAcc.z = (uint16_t)(((uint16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

    if (scale) {
        //Scale all values to double precision
        scaledAcc.x = ((double)rawAcc.x - accOffset.x) * accScale.x;
        scaledAcc.y = ((double)rawAcc.y - accOffset.y) * accScale.y;
        scaledAcc.z = ((double)rawAcc.z - accOffset.z) * accScale.z;
    }
}

void MPU9250::readMag(bool scale) {

    uint8_t rawDataBuf[7];

    if (magBypass) {
        readI2CRegs(magI2CAddr, AK8963_HXL, 7, &rawDataBuf[0]);

        //Ignore magnetic overflow
        if (!(rawDataBuf[6] & 0x08)) {
            rawMag.x = (int16_t)(((int16_t)rawDataBuf[1] << 8) | rawDataBuf[0]);
            rawMag.y = (int16_t)(((int16_t)rawDataBuf[3] << 8) | rawDataBuf[2]);
            rawMag.z = (int16_t)(((int16_t)rawDataBuf[5] << 8) | rawDataBuf[4]);

            //Scale all values to double precision
            scaledMag.x = ((double)rawMag.x * magAdj.x * magSensitivity) - magOffset.x;
            scaledMag.y = ((double)rawMag.y * magAdj.y * magSensitivity) - magOffset.y;
            scaledMag.z = ((double)rawMag.z * magAdj.z * magSensitivity) - magOffset.z;
        }
    } else {
        //Todo...
    }
}

void MPU9250::calibrateGyro(int sampleCount) {

    //FSR must be 250 DPS for calibration
    setGyroFSR(GFSR_250DPS);

    //Todo: wait until it's sitting still via acc

    long xSum = 0, ySum = 0, zSum = 0;
    for (int i = 0; i < sampleCount; i++) {
        //Read gyro sample without scaling
        do {
            delay(1);
        } while (!dataReady());
        readGyro(false);
        xSum += rawGyro.x;
        ySum += rawGyro.y;
        zSum += rawGyro.z;
    }

    //Compute average offsets
    int16_t xOffset = round((double)xSum / (double)sampleCount);
    int16_t yOffset = round((double)ySum / (double)sampleCount);
    int16_t zOffset = round((double)zSum / (double)sampleCount);

    snprintf(printMsgBuf, PRINT_MAX_BUF, "0x%X", xOffset);
    printMessage(printMsgBuf);

    /*
    //Write unsigned offset bytes to gyro offset registers
    writeI2CReg(MPU9250_I2C_ADDR, XG_OFFSET_H, (xOffset >> 8));
    writeI2CReg(MPU9250_I2C_ADDR, XG_OFFSET_L, (xOffset & 0x00FF));
    writeI2CReg(MPU9250_I2C_ADDR, YG_OFFSET_H, (yOffset >> 8));
    writeI2CReg(MPU9250_I2C_ADDR, YG_OFFSET_L, (yOffset & 0x00FF));
    writeI2CReg(MPU9250_I2C_ADDR, ZG_OFFSET_H, (zOffset >> 8));
    writeI2CReg(MPU9250_I2C_ADDR, ZG_OFFSET_L, (zOffset & 0x00FF));
    */

    gyroBias.x = (double)xOffset * (500.0 / 65536.0);
    gyroBias.y = (double)yOffset * (500.0 / 65536.0);
    gyroBias.z = (double)zOffset * (500.0 / 65536.0);

    printMessage("Gyro calibration complete.");

    //printRegister("XG_OFFSET_H", readI2CReg(MPU9250_I2C_ADDR, XG_OFFSET_H));
    //printRegister("XG_OFFSET_L", readI2CReg(MPU9250_I2C_ADDR, XG_OFFSET_L));

    //Return FSR to original value
    setGyroFSR(gyroFSR);
}

void MPU9250::calibrateAcc(int sampleCount) {
    printMessage("\nAccelerometer Calibration:\n");
    snprintf(printMsgBuf, PRINT_MAX_BUF, "Samples: %d\nSend a single message to take a sample.\nMake sure the accelerometer is not moving while each sample is being taken.", sampleCount);
    printMessage(printMsgBuf);

    //Calibration code
    //initialize
    samp_capacity = sampleCount;
    n_samp = 0;
    data = (long*)malloc(samp_capacity * 3 * sizeof(long));
    reset_calibration_matrices();

    //initialize beta to something reasonable
    beta[0] = beta[1] = beta[2] = 512.0;
    beta[3] = beta[4] = beta[5] = 0.0095;

    for (int sampleNum = 0; sampleNum < sampleCount; sampleNum++) {
        while (!SerialUSB.available());
        //Clear out input buffer
        do {
            SerialUSB.read();
        } while (SerialUSB.available());

        snprintf(printMsgBuf, PRINT_MAX_BUF, "\nTaking sample %d...\n", sampleNum+1);
        printMessage(printMsgBuf);

        delay(200);
        take_sample(data + 3 * (n_samp % samp_capacity));
        n_samp++;

        printMessage("Done!");
    }

    SerialUSB.println("Performing calibration calculations...");
    calibrate_model();

    //XYZ offsets and scales are stored in betas
    accOffset.x = beta[0];
    accOffset.y = beta[1];
    accOffset.z = beta[2];

    accScale.x = beta[3];
    accScale.y = beta[4];
    accScale.z = beta[5];

    //Print samples for 5 seconds for verification
    unsigned long startTime = millis();
    do {
        delay(1000);
        readAcc();
        scaledAcc.x = ((double)rawAcc.x - accOffset.x) * accScale.x;
        scaledAcc.y = ((double)rawAcc.y - accOffset.y) * accScale.y;
        scaledAcc.z = ((double)rawAcc.z - accOffset.z) * accScale.z;
        SerialUSB.println("Acc: " + String(scaledAcc.x) + ", " + String(scaledAcc.y) + ", " + String(scaledAcc.z) + " g");
    } while(millis() - startTime < 5000);
}

void MPU9250::calibrateMag(int sampleCount) {
    printMessage("Magnetometer Calibration:\n");
    printMessage("Wave the IMU in a 3-dimensional figure-8 pattern for several seconds.");
    for (int i = 3; i > 0; i--) {
        snprintf(printMsgBuf, PRINT_MAX_BUF, "%d", i);
        printMessage(printMsgBuf);
        delay(1000);
    }
    printMessage("Go!");

    //Read one sample for baseline readings
    while (!magDataReady()) {
        delay(10);
    };
    readMag();

    SSCALEDDATA magMin, magMax;
    magMin = magMax = scaledMag;
    
    int magSamples = 0;
    do {
        if (magDataReady()) {
            readMag();

            magMin.x = min(magMin.x, scaledMag.x);
            magMin.y = min(magMin.y, scaledMag.y);
            magMin.z = min(magMin.z, scaledMag.z);

            magMax.x = max(magMax.x, scaledMag.x);
            magMax.y = max(magMax.y, scaledMag.y);
            magMax.z = max(magMax.z, scaledMag.z);

            magSamples++;
        }
        delay(10);
    } while (magSamples < sampleCount);

    snprintf(printMsgBuf, PRINT_MAX_BUF, "Collected %d samples.", magSamples);
    snprintf(printMsgBuf, PRINT_MAX_BUF, "Calibration Complete.\nMin values: %f, %f, %f.\nMax values: %f, %f, %f\n", magMin.x, magMin.y, magMin.z, magMax.x, magMax.y, magMax.z);
    printMessage(printMsgBuf);

    magOffset.x = (magMin.x + magMax.x) / 2.0;
    magOffset.y = (magMin.y + magMax.y) / 2.0;
    magOffset.z = (magMin.z + magMax.z) / 2.0;

    snprintf(printMsgBuf, PRINT_MAX_BUF, "Averages: %f, %f, %f\n", magOffset.x, magOffset.y, magOffset.z);
    printMessage(printMsgBuf);
}