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

extern RWCComHandler comm;

RWCComHandler::RWCComHandler(VehicleConfig *config) : _vehicleConfig(config)
{
}

RWCComHandler::~RWCComHandler()
{
}

void RWCComHandler::handler()
{
    if (!_requestHandled)
    {

        uint8_t opCode = _requestBuffer[0];
        uint8_t dataSize;

        switch (opCode)
        {
        case STATE:
            dataSize = STATE_SIZE;
            break;
        case KEEP_ALIVE:
            dataSize = KEEP_ALIVE_SIZE;
            break;
        case ORIENTATION_MODE:
            dataSize = ORIENTATION_MODE_SIZE;
            break;
        case ORIENTATION_SETPOINT:
            dataSize = ORIENTATION_SETPOINT_SIZE;
            break;
        case SPEED_SETPOINT:
            dataSize = SPEED_SETPOINT_SIZE;
            break;
        case ORIENTATION:
            dataSize = ORIENTATION_SIZE;
            break;
        case ANG_SPEED:
            dataSize = ANG_SPEED_SIZE;
            break;
        case MOTOR_SPEED:
            dataSize = MOTOR_SPEED_SIZE;
            break;
        case NEW_DATA:
            dataSize = NEW_DATA_SIZE;
            break;
        case ERROR:
            dataSize = ERROR_SIZE;
            break;
        }

        if (_requestLen == 3)
        { // if request is only 3 bytes long (opCode + CRC) - no new data arrived -> read request

        }
        else
        { // if request is more then 3 bytes long -> write request
        }

        _requestHandled = 1;
    }
}

void RWCComHandler::newRequest(uint8_t *request, uint8_t len)
{

    for (uint8_t i = 0; i < BUFFER_LEN; i++)
    {
        _requestBuffer[i] = 0;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        _requestBuffer[i] = request[i];
    }

    _requestLen = len;

    _requestHandled = 0;
}

void i2cCommReceive(int len)
{
    uint8_t *buffer = new uint8_t[len];

    for (uint16_t i = 0; i < len; i++)
    {
        buffer[i] = Wire1.read();
    }

    comm.newRequest(buffer, len);

    delete[] buffer;
}

void i2cCommRequest()
{
    uint8_t buffer[128];
    uint8_t len;
    comm.generateResponse(buffer, &len);
}