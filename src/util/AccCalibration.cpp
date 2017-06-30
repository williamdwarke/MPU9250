/*
# copyright 2011 by Rolfe Schmidt
# This work is licensed under the Creative Commons Attribution-ShareAlike 3.0 Unported License.
# To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/ or send a
# letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
#
# Available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
#
# Described at http://wp.me/p1CQVj-1k
*/
#include <Arduino.h>

//I know, this is really hacky
#include "I2CDevice.h"
#include "SerialMessage.h"
#include "../MPU9250RegisterMap.h"

#include "AccCalibration.h"

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} SRAWDATA;

//data collection structures
long *data;   //dynamically allocated array for data storage.  Don't let it get too big!
int samp_capacity = 0;             //the capacity of the sample data array
int n_samp = 0;                    // Number of samples used for calibration
int sample_size = 32;              // Number of measurements averaged to produce 1 sample

//parameters for model.  beta[0], beta[1], and beta[2] are the 0-G marks (about 512)
double beta[6];                        
// while beta[3], beta[4], and beta[5] are the scaling factors.  So, e.g., if xpin reads
// value x, number of G's in the x direction in beta[3]*(x - beta[0]).

//matrices for Gauss-Newton computations
double JS[6][6];
double dS[6];
double delta[6];

//pass in a length-3 array where the data will be written
void take_sample(long *sample_out) {
    int i = 0;
    int first_pass_size = 5;
    int success = 0;

    while (success == 0) {
        //First, run through 32 samples and accumulate the mean and variance.
        //Make all variables longs because we will do some aritmetic that
        // will overflow an int.
        long sum[] = {0, 0, 0};
        unsigned long sum_squares[] = {0, 0, 0};
        unsigned long variance[] = {0, 0, 0};
        long x, y, z;
        uint8_t rawDataBuf[6];
        SRAWDATA rawAcc;

        for (i = 0; i < (1 << first_pass_size); ++i) {
            //Take a reading
            readI2CRegs(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, 6, &rawDataBuf[0]);

            rawAcc.x = (uint16_t)(((uint16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
            rawAcc.y = (uint16_t)(((uint16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
            rawAcc.z = (uint16_t)(((uint16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

            //Convert the MSB and LSB into a signed 16-bit value
            x = (long)rawAcc.x; 
            y = (long)rawAcc.y;
            z = (long)rawAcc.z;

            delay(20); //sample at 50 Hz

            sum[0] += x;
            sum[1] += y;
            sum[2] += z;

            sum_squares[0] += (unsigned long)(abs(x)) * (unsigned long)(abs(x));
            sum_squares[1] += (unsigned long)(abs(y)) * (unsigned long)(abs(y));
            sum_squares[2] += (unsigned long)(abs(z)) * (unsigned long)(abs(z));
        }

        for (i = 0; i < 3; i++) {
            unsigned long sum_squared = abs(sum[i]) * abs(sum[i]);
            variance[i] = sum_squares[i] / (sample_size - 1) - sum_squared / (sample_size - 1);

            SerialUSB.print("Sum squares: ");
            SerialUSB.print(sum_squares[i]);
            SerialUSB.print(" Sum, squared: ");
            SerialUSB.print(sum_squared);
            SerialUSB.print(" Variance: ");
            SerialUSB.print(variance[i]);
            SerialUSB.println();
        }

        //with mean and variance in place, start collecting real samples but filter out outliers.
        //Track the success rate and start over if we get too many fails.
        unsigned int success_count = 0;
        unsigned int fail_count = 0;
        i = 0;
        sample_out[0] = sample_out[1] = sample_out[2] = 0;

        while (i < sample_size) {

            //take a reading
            readI2CRegs(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H, 6, &rawDataBuf[0]);

            rawAcc.x = (uint16_t)(((uint16_t)rawDataBuf[0] << 8) | rawDataBuf[1]); 
            rawAcc.y = (uint16_t)(((uint16_t)rawDataBuf[2] << 8) | rawDataBuf[3]);
            rawAcc.z = (uint16_t)(((uint16_t)rawDataBuf[4] << 8) | rawDataBuf[5]);

            //Convert the MSB and LSB into a signed 16-bit value
            x = (long)rawAcc.x; 
            y = (long)rawAcc.y;
            z = (long)rawAcc.z;

            delay(20); //sample at 50 Hz

            long dx = x * 32 - sum[0];
            long dy = y * 32 - sum[1];
            long dz = z * 32 - sum[2];

            //check to see if it is any good (within 3 std deviations)
            if ((dx * dx) < 9 * variance[0] && (dy * dy) < 9 * variance[1] && (dz * dz) < 9 * variance[2]) {
                success_count++;
                sample_out[0] += x;
                sample_out[1] += y;
                sample_out[2] += z;
                i++;
            } else {
                fail_count++;
            }

            if (fail_count > success_count && i > 10 || fail_count > sample_size) {
                //we're failing too much, start over!
                SerialUSB.println("#Sample fail");
                break;
            }

            if (i % 5 == 0) {
                SerialUSB.println(x);
                SerialUSB.println(y);
                SerialUSB.println(z);
            }
        }

        //if we got our samples, mark the success.  Otherwise we'll start over.
        if (i == sample_size) {
            success = 1;

            sample_out[0] /= sample_size;
            sample_out[1] /= sample_size;
            sample_out[2] /= sample_size;

            SerialUSB.print("# ");
            SerialUSB.print(sample_out[0]);
            SerialUSB.print(" ");
            SerialUSB.print(sample_out[1]);
            SerialUSB.print(" ");
            SerialUSB.print(sample_out[2]);
            SerialUSB.print(";\n");
        }
    }
}

//Gauss-Newton functions
void reset_calibration_matrices() {
    int j, k;
    for (j = 0; j < 6; ++j) {
        dS[j] = 0.0;
        for (k = 0; k < 6; ++k) {
            JS[j][k] = 0.0;
        }
    }
}

void update_calibration_matrices(const long *data) {
    int j, k;
    double dx, b;
    double residual = 1.0;
    double jacobian[6];

    for (j = 0; j < 3; ++j) {
        b = beta[3 + j];
        dx = ((float)data[j]) - beta[j];
        residual -= b * b * dx * dx;
        jacobian[j] = 2.0 * b * b * dx;
        jacobian[3 + j] = -2.0 * b * dx * dx;
    }

    for (j = 0; j < 6; ++j) {
        dS[j] += jacobian[j] * residual;
        for (k = 0; k < 6; ++k) {
            JS[j][k] += jacobian[j] * jacobian[k];
        }
    }
}

void compute_calibration_matrices() {
    int i, j, k;
    double dx, b;

    reset_calibration_matrices();
    int ub = n_samp < samp_capacity ? n_samp : samp_capacity;
    for (i = 0; i < ub; i++) {
        update_calibration_matrices(data + 3 * i);
    }
}

void find_delta() {
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int i, j, k;
    double mu;

    //make upper triangular
    for (i = 0; i < 6; ++i) {
        //eliminate all nonzero entries below JS[i][i]
        for (j = i + 1; j < 6; ++j) {
            mu = JS[i][j] / JS[i][i];
            if (mu != 0.0) {
                dS[j] -= mu * dS[i];
                for (k = j; k < 6; ++k) {
                    JS[k][j] -= mu * JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for (i = 5; i >= 0; --i) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0;
        for (j = 0; j < i; ++j) {
            mu = JS[i][j];
            dS[j] -= mu * dS[i];
            JS[i][j] = 0.0;
        }
    }

    for (i = 0; i < 6; ++i) {
        delta[i] = dS[i];
    }
}

void calibrate_model() {
    int i;
    double eps = 0.000000001;
    int num_iterations = 40;
    double change = 100.0;
    while (--num_iterations >= 0 && change > eps) {
        compute_calibration_matrices();
        find_delta();
        change = delta[0] * delta[0] + delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3] / (beta[3] * beta[3]) + delta[4] * delta[4] / (beta[4] * beta[4]) + delta[5] * delta[5] / (beta[5] * beta[5]);

        for (i = 0; i < 6; ++i) {
            beta[i] -= delta[i];
        }

        reset_calibration_matrices();
        /*
        SerialUSB.print( "Num iterations: ");
        SerialUSB.print(20 - num_iterations);
        SerialUSB.print( " change: ");
        SerialUSB.println( change, 10);
        */
    }

    SerialUSB.print("\n");
    for (i = 0; i < 6; ++i) {
        SerialUSB.print(beta[i], 10);
        SerialUSB.print(" ");
    }
    SerialUSB.println();
}
