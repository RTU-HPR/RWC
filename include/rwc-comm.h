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
#include "Checksums.h"

#define BUFFER_LEN 128

void i2cCommReceive(int);
void i2cCommRequest();

class RWCComHandler
{
private:
    uint8_t _requestHandled;
    uint8_t _responseLen;
    uint8_t _requestLen;
    uint8_t _requestBuffer[BUFFER_LEN];
    uint8_t _responseBuffer[BUFFER_LEN];
    VehicleConfig *_vehicleConfig;

public:
    RWCComHandler(VehicleConfig *config);
    ~RWCComHandler();

    void handler();
    void newRequest(uint8_t *request, uint8_t len);
    void generateResponse(uint8_t *response, uint8_t *responseLen);
};

#endif