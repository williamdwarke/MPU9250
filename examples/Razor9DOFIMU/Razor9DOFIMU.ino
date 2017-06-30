#include <MPU9250.h>
#include <Wire.h>
#include <FlashStorage.h>

#include "AHRS.h"
#include "MPU9250RegisterMap.h"

//IMU interrupt pin
#define INT_PIN 4

//Constant(s)
#define SEC_PER_US 0.000001

FlashStorage(calibratedStorage, bool);
FlashStorage(accOffsetStorage, SSCALEDDATA);
FlashStorage(accScaleStorage, SSCALEDDATA);
FlashStorage(magOffsetStorage, SSCALEDDATA);

MPU9250 imu;
bool calibrated = false;
bool initialized = false;

unsigned long loopTime, accTime, magTime, printTime, ahrsTime;
double ahrsDT = 0;
bool newData = false;

SSCALEDDATA gyro, acc, mag;

void initTimers() {
    loopTime = accTime = magTime = printTime = ahrsTime = micros();
}

void setMARGVariables() {
    gyro.x = degToRad(imu.scaledGyro.y);
    gyro.y = degToRad(imu.scaledGyro.x);
    gyro.z = degToRad(-1.0*imu.scaledGyro.z);

    acc.x = -1.0 * imu.scaledAcc.y;
    acc.y = -1.0 * imu.scaledAcc.x;
    acc.z = imu.scaledAcc.z;

    mag.x = imu.scaledMag.x;
    mag.y = imu.scaledMag.y;
    mag.z = imu.scaledMag.z;
}

void getAccMagCalibration() {
    calibrated = calibratedStorage.read();
    if (calibrated) {
        imu.accOffset = accOffsetStorage.read();
        imu.accScale = accScaleStorage.read();
        imu.magOffset = magOffsetStorage.read();

        SerialUSB.println("IMU calibration loaded.");
        SerialUSB.println("Acc offsets: " + String(imu.accOffset.x, 10) + ", " + String(imu.accOffset.y, 10) + ", " + String(imu.accOffset.z, 10));
        SerialUSB.println("Acc scales: " + String(imu.accScale.x, 10) + ", " + String(imu.accScale.y, 10) + ", " + String(imu.accScale.z, 10));
        SerialUSB.println("Mag offsets: " + String(imu.magOffset.x, 10) + ", " + String(imu.magOffset.y, 10) + ", " + String(imu.magOffset.z, 10));
    } else {
        imu.calibrateAcc(10);
        accScaleStorage.write(imu.accScale);
        accOffsetStorage.write(imu.accOffset);
        SerialUSB.println("Saved Acc calibration to flash.\n");

        delay(1000);
        SerialUSB.println("Send any message to begin Mag calibration.");
        while (!SerialUSB.available());
        do {
            SerialUSB.read();
        } while (SerialUSB.available());

        imu.calibrateMag();
        magOffsetStorage.write(imu.magOffset);
        SerialUSB.println("Saved Mag calibration to flash.");

        calibrated = true;
        calibratedStorage.write(calibrated);

        SerialUSB.println("Calibration complete.\nSet the IMU still on a flat surface for gyro calibration.");
        SerialUSB.println("Send any message to continue.");
        while (!SerialUSB.available());
        do {
            SerialUSB.read();
        } while (SerialUSB.available());
    }
}

/***************************************************************/
void setup() {
    SerialUSB.begin(115200);

    //Delay to allow serial terminal to load
    delay(2000);
    SerialUSB.println("Initializing...");

    //Initialize I2C communication
    Wire.begin();
    Wire.setClock(400000L);

    if (imu.init(GFSR_250DPS, AFSR_2G, LPF_5HZ) < 0) {
        SerialUSB.println("Initialization error.");
        return;
    }
    imu.enableInterruptPin(INT_PIN);
    getAccMagCalibration();
    imu.calibrateGyro();

    //Get gyro, acc, and mag
    do {
        delay(100);
    } while (!imu.dataReady());
    imu.readAccGyro();

    while (!imu.magDataReady()) {
        delay(100);
    }
    imu.readMag();
    setMARGVariables();
    initializeQuaternion(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);

    SerialUSB.println("Initialization Complete");

    initTimers();
    initialized = true;
}

void loop() {
    if (!initialized) {
        SerialUSB.println("Initialization failed! Looping...");
        delay(5000);
        return;
    }

    loopTime = micros();

    if (imu.dataReady()) {
        //Gyro/Acc = 1kHz
        imu.readGyro();
        imu.readAcc();
        newData = true;
    }

    //Mag = 100Hz
    if (loopTime - magTime >= 10000) {
        if (imu.magDataReady()) {
            imu.readMag();
            magTime = loopTime;
            newData = true;
        }
    }

    if (newData) {
        setMARGVariables();
        loopTime = micros();
        ahrsDT = (double)((loopTime - ahrsTime) * SEC_PER_US);
        MadgwickAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, ahrsDT);
        //MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ahrsDT);
        ahrsTime = loopTime;
    }

    if (loopTime - printTime >= (1000000 / 100)) {
        //Get and print Euler angles
        quaternionToEuler();
        snprintf(printMsgBuf, PRINT_MAX_BUF, "%0.2f, %0.2f, %0.2f", degRoll, degPitch, degYaw);
        SerialUSB.println(printMsgBuf);

        //Uncomment for debugging data
        //SerialUSB.println("Gyro: " + String(imu.scaledGyro.x) + ", " + String(imu.scaledGyro.y) + ", " + String(imu.scaledGyro.z));

        printTime = loopTime;
    }
}