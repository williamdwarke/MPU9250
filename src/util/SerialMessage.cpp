#include <Arduino.h>
#include <inttypes.h>
#include <stdio.h>

#include "SerialMessage.h"

void printMessage(const char *msg) {
#ifndef DISABLE_SERIAL
    SerialUSB.println(msg);
#endif
}

uint8_t failMessage(const char *msg) {
    snprintf(errMsgBuf, PRINT_MAX_BUF, "Error: %s", msg);
    printMessage(errMsgBuf);
    return -1;
}