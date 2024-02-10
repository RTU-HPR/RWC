/**
 * @file RWC-slave.cpp
 * @author Marko
 * @brief Reaction Wheel Controller communication with Payload Flight Computer class header.
 * @version 0.1
 * @date 2024-02-10
 * @copyright Copyright (c) 2024
 */

#ifndef RWC_COMM_H
#define RWC_COMM_H

#include "Arduino.h"
#include "Wire.h"
#include "stdlib.h"
#include "vehicle.h"
#include "Checksums.h"

#define BUFFER_LEN 128

/**
 * @brief Handles the I2C communication reception.
 * 
 * @param numBytes Number of bytes received.
 */
void i2cCommReceive(int numBytes);

/**
 * @brief Handles the I2C communication request.
 */
void i2cCommRequest();

/**
 * @brief Class for handling communication with the Reaction Wheel Controller.
 */
class RWCComHandler
{
private:
    uint8_t _requestHandled; /**< Flag indicating if the request has been handled. */
    uint8_t _responseLen;    /**< Length of the response. */
    uint8_t _requestLen;     /**< Length of the request. */
    uint8_t _requestBuffer[BUFFER_LEN];   /**< Buffer for storing the request data. */
    uint8_t _responseBuffer[BUFFER_LEN];  /**< Buffer for storing the response data. */
    VehicleConfig *_vehicleConfig;        /**< Pointer to the vehicle configuration. */

    /**
     * @brief Updates the keep-alive watchdog.
     */
    void _updateKeepalive();

public:
    /**
     * @brief Constructs a new RWCComHandler object.
     * 
     * @param config Pointer to the vehicle configuration.
     */
    RWCComHandler(VehicleConfig *config);

    /**
     * @brief Destroys the RWCComHandler object.
     */
    ~RWCComHandler();

    /**
     * @brief Handles the data received, prepares the data to be sent. All of the processing is done here (except CRC  calculation).
     */
    void handler();

    /**
     * @brief Handles a new request.
     * 
     * @param request Pointer to the request data.
     * @param len Length of the request.
     */
    void newRequest(uint8_t *request, uint8_t len);

    /**
     * @brief Generates a response.
     * 
     * @param response Pointer to the response data.
     * @param responseLen Pointer to the length of the response.
     */
    void generateResponse(uint8_t *response, uint8_t *responseLen);
};

#endif
