/***
 * @file vehicle.h
 * @brief Vehicle configuration structure and definitions.
 * @version 1.0
 * @date 2024-04-11
 * @author Marko
*/
#ifndef PLATFORM_H
#define PLATFORM_H

#include "stdlib.h"
#include "motor.h"

/**
 * @brief Vehicle configuration structure.
 * 
 */
struct VehicleConfig
{
    uint8_t state;
    uint64_t lastKeepAlive;
    uint8_t mode;
    uint8_t newData;
    uint8_t error;
    uint8_t calibration;

    Motor *motor;

    float orientationSetpoint;
    float speedSetpoint;
    float orientation;
    float angSpeed;
    float motorSpeed;
    float motorTemp;
    float batteryVoltage;
};

/**
 * @brief Vehicle I2C registers. Check main document fore more.
 * 
 */
enum RWCRegisters
{
    STATE,
    KEEP_ALIVE,
    ORIENTATION_MODE,
    ORIENTATION_SETPOINT,
    SPEED_SETPOINT,
    ORIENTATION,
    ANG_SPEED,
    MOTOR_SPEED,
    NEW_DATA,
    ERROR,
    CALIBRATION_STATUS,
    MOTOR_TEMP,
    BATTERY_VOLTAGE,
};

/**
 * @brief Vehicle I2C registers size. Check main document for more.
 * 
 */

enum RWCRegistersSize
{
    STATE_SIZE = 1,
    KEEP_ALIVE_SIZE = 1,
    ORIENTATION_MODE_SIZE = 1,
    ORIENTATION_SETPOINT_SIZE = 4,
    SPEED_SETPOINT_SIZE = 4,
    ORIENTATION_SIZE = 4,
    ANG_SPEED_SIZE = 4,
    MOTOR_SPEED_SIZE = 4,
    NEW_DATA_SIZE = 1,
    ERROR_SIZE = 1,
    CALIBRATION_STATUS_SIZE = 1,
    MOTOR_TEMP_SIZE = 4,
    BATTERY_VOLTAGE_SIZE = 4,
};

/**
 * @brief Vehicle state.
 * 
 */

enum RWCState
{
    IDLE,
    STAB
};

/**
 * @brief Vehicle orientation mode.
 * 
 */

enum RWCOrientationMode
{
    ORIENTATION_HOLD,
    SPEED_HOLD
};

/**
 * @brief Vehicle telemetry new data flags.
 * 
 */

enum RWCNewData
{
    DATA_CLEAR,
    NEW_ORIENTATION = 1 << 0,
    NEW_ANG_SPEED = 1 << 1,
    NEW_MOTOR_SPEED = 1 << 2,
};

/**
 * @brief Vehicle telemetry errors.
 * 
 */

enum RWCErrors
{
    ERR_CLEAR,
    CRC_ERR = 1 << 0,
    MOTOR_RUNAWAY = 1 << 1,
};

#endif