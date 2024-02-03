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

        for (uint8_t i = 0; i < BUFFER_LEN; i++)
        {
            _responseBuffer[i] = 0;
        }

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
            if (!Checksum_CCITT_16::verify(_requestBuffer, 3))
            {
                _vehicleConfig->error |= CRC_ERR;
                _requestHandled = 1;
                return;
            }

            _responseLen = dataSize;

            switch (opCode)
            {
            case STATE:
                _responseBuffer[0] = _vehicleConfig->state;
                break;
            case KEEP_ALIVE:
                _responseBuffer[0] = 0;
                break;
            case ORIENTATION_MODE:
                _responseBuffer[0] = _vehicleConfig->mode;
                break;
            case ORIENTATION_SETPOINT:
                memcpy(_responseBuffer, (const void *)&_vehicleConfig->orientationSetpoint, 4);
                break;
            case SPEED_SETPOINT:
                memcpy(_responseBuffer, (const void *)&_vehicleConfig->speedSetpoint, 4);
                break;
            case ORIENTATION:
                memcpy(_responseBuffer, (const void *)&_vehicleConfig->orientation, 4);
                break;
            case ANG_SPEED:
                memcpy(_responseBuffer, (const void *)&_vehicleConfig->angSpeed, 4);
                break;
            case MOTOR_SPEED:
                memcpy(_responseBuffer, (const void *)&_vehicleConfig->motorSpeed, 4);
                break;
            case NEW_DATA:
                _responseBuffer[0] = _vehicleConfig->newData;
                _vehicleConfig->newData = 0;
                break;
            case ERROR:
                _responseBuffer[0] = _vehicleConfig->error;
                _vehicleConfig->error = 0;
                break;
            }
        }
        else
        { // if request is more then 3 bytes long -> write request
            if (!Checksum_CCITT_16::verify(_requestBuffer, 1 + dataSize + 2))
            {
                _vehicleConfig->error |= CRC_ERR;
                _requestHandled = 1;
                return;
            }

            uint8_t payload[4];
            memcpy(payload, _requestBuffer + 1, dataSize);

            switch (opCode)
            {
            case STATE:
                _vehicleConfig->state = payload[0];
                break;
            case KEEP_ALIVE:
                _updateKeepalive();
                break;
            case ORIENTATION_MODE:
                _vehicleConfig->mode = payload[0];
                break;
            case ORIENTATION_SETPOINT:
                memcpy((void *)&_vehicleConfig->orientationSetpoint, payload, 4);
                break;
            case SPEED_SETPOINT:
                memcpy((void *)&_vehicleConfig->speedSetpoint, payload, 4);
                break;
            }
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

void RWCComHandler::generateResponse(uint8_t *response, uint8_t *responseLen)
{
    *responseLen = _responseLen + 2;
    uint16_t crc = Checksum_CCITT_16::calculate(_responseBuffer, _responseLen);
    memcpy(response, _responseBuffer, _responseLen);
    response[_responseLen] = crc >> 8;
    response[_responseLen + 1] = crc & 0xFF;
}

void RWCComHandler::_updateKeepalive()
{
    _vehicleConfig->lastKeepAlive = millis();
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

    Wire1.write(buffer, len);
}