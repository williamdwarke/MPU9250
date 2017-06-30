#ifndef MPU925RegisterMap_h
#define MPU9250RegisterMap_h

#define MPU9250_I2C_ADDR        0x68
#define MPU9250_WHOAMI_VAL      0x71
#define AK8963_DEVICE_ID        0x48

//Note: Adding registers to this list as needed.
#define SELF_TEST_X_GYRO        0x00
#define SELF_TEST_Y_GYRO        0x01
#define SELF_TEST_Z_GYRO        0x02
#define SELF_TEST_X_ACCEL       0x0D
#define SELF_TEST_Y_ACCEL       0x0E
#define SELF_TEST_Z_ACCEL       0x0F

//Gyro offsets
#define XG_OFFSET_H             0x13
#define XG_OFFSET_L             0x14
#define YG_OFFSET_H             0x15
#define YG_OFFSET_L             0x16
#define ZG_OFFSET_H             0x17
#define ZG_OFFSET_L             0x18

#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_CONFIG_2  0x1D

#define MPU9250_FIFO_EN         0x23
#define MPU9250_I2C_MST_CTRL    0x24

#define MPU9250_I2C_SLV4_CTRL   0x34
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_STATUS      0x3A

#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_TEMP_OUT_H      0x41
#define MPU9250_GYRO_XOUT_H     0x43

#define MPU9250_USER_CTRL       0x6A
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C
#define MPU9250_WHOAMI          0x75

//Acc offsets
#define MPU9250_XA_OFFSET_H     0x77
#define MPU9250_XA_OFFSET_L     0x78
#define MPU9250_YA_OFFSET_H     0x7A
#define MPU9250_YA_OFFSET_L     0x7B
#define MPU9250_ZA_OFFSET_H     0x7D
#define MPU9250_ZA_OFFSET_L     0x7E

//AK8963 onboard magnetometer registers (bypass only)
#define AK8963_WIA              0x00
#define AK8963_INFO             0x01
#define AK8963_ST1              0x02
#define AK8963_HXL              0x03
#define AK8963_ST2              0x09
#define AK8963_CNTL1            0x0A
#define AK8963_CNTL2            0x0B
#define AK8963_ASTC             0x0C

//Sensitivity adjustment
#define AK8963_ASAX             0x10
#define AK8963_ASAY             0x11
#define AK8963_ASAZ             0x12

#endif