/**
 * @file RWC-slave.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-01-31
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RWC_COMM_H
#define RWC_COMM_H

#include "Arduino.h"
#include "Wire.h"
#include "stdlib.h"
#include "vehicle.h"

#define BUFFER_LEN 128

void i2cCommReceive(int);
void i2cCommRequest();

class RWCComHandler
{
private:
    uint8_t _requestHandled;
    uint8_t _responseLengt;
    VehicleConfig *_vehicleConfig;

public:
    RWCComHandler(VehicleConfig *config);
    ~RWCComHandler();

    uint8_t requestLen;
    uint8_t requestBuffer[BUFFER_LEN];
    uint8_t responseBuffer[BUFFER_LEN];

    void handler();
    void newRequest(uint8_t *request, uint8_t len);
};

#endif