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

#include "rwc-comm.h"

RWCComHandler::RWCComHandler(VehicleConfig *config) : _vehicleConfig(config)
{
}

RWCComHandler::~RWCComHandler()
{
}

void RWCComHandler::handler()
{
    if (!requestHandled)
    {
        requestHandled = 1;
    }
}

void RWCComHandler::newRequest(uint8_t *request, uint8_t len)
{

    for (uint8_t i = 0; i < BUFFER_LEN; i++)
    {
        requestBuffer[i] = 0;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        requestBuffer[i] = requestBuffer[i];
    }

    requestHandled = 0;
}