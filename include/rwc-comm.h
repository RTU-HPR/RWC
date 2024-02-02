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
#include "stdlib.h"
#include "vehicle.h"

#define BUFFER_LEN 128

class RWCComHandler
{
private:
    uint8_t requestHandled;
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