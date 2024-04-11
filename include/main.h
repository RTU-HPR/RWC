/**
 * @file main.h
 * @author Marko
 * @brief Main header file containing all the necessary includes and definitions.
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "filter.h"
#include "motor.h"
#include "pid.h"
#include "vehicle.h"
#include "rwc-comm.h"

#define BNO_TICK_FREQ 500 ///< BNO055 sensor tick frequency in Hz
#define BNO_TICK_PERIOD 1000000 / BNO_TICK_FREQ - 1
#define MOTOR_TICK_FREQ 200 ///< Motor pid loop tick frequency in Hz
#define MOTOR_TICK_PERIOD 1000000 / MOTOR_TICK_FREQ - 1
#define COMM_UPDATE_FREQ 10 ///< I2C communication telemetry daya update frequency in Hz
#define COMM_UPDATE_PERIOD 1000 / COMM_UPDATE_FREQ - 1

#define MOTOR_TEMP_CHCK_FREQ 500                                  ///< Motor temperature check frequency in mHz
#define MOTOR_TEMP_CHCK_PERIOD 1000000 / MOTOR_TEMP_CHCK_FREQ - 1 ///< NOTE: DS18B20 takes 750ms to read the temperature, if the period is less
                                                                    ///< temperature readings will block the main loop. DON'T set the frequency > 1.33 Hz
#if MOTOR_TEMP_CHCK_PERIOD < 750
    #error "MOTOR_TEMP_CHCK_PERIOD must be greater than 750ms!"
#endif

#define BATTERY_VOLTAGE_CHCK_FREQ 10 ///< Battery voltage check frequency in Hz
#define BATTERY_VOLTAGE_CHCK_PERIOD 1000 / BATTERY_VOLTAGE_CHCK_FREQ - 1

#define PID_SWITCH_TRESHOLD 12.0f ///< PID switch treshold in rad/s. If the speed is below this value, the orientation PID is switched to speed PID

#define KEEP_ALIVE_DEAD 5 * 1000 ///< Time without heartbeat after which the state is set to IDLE

#define I2C_SLAVE_SCL_PIN 15 ///< RWC-BFC I2C slave SCL pin
#define I2C_SLAVE_SDA_PIN 16 ///< RWC-BFC I2C slave SDA pin
#define I2C_SLAVE_ADDR 0x01 ///< RWC-BFC I2C slave address
#define I2C_SLAVE_FREQ 100000 ///< RWC-BFC I2C slave frequency

#define BNO_SDA 48 ///< BNO055 sensor SDA pin
#define BNO_SCL 47 ///< BNO055 sensor SCL pin

#define SPEED_PID_MAXIMUM_OUTPUT 250.0f ///< Maximum output of the speed PID controller
#define SPEED_PID_MINIMUM_OUTPUT -SPEED_PID_MAXIMUM_OUTPUT ///< Minimum output of the speed PID controller
#define MOTOR_RUNAWAY_TIME 30 * 1000 ///< Time in ms, after which the motor is considered runaway if the speed PID setpoint is at maximum or minimum
#define BNO_CALIBRATION_CHCK_FREQ 10 ///< BNO055 sensor calibration check frequency in Hz, is reported in the telemetry data as well as LEDs
#define BNO_CALIBRATION_CHCK_PERIOD 1000 / BNO_CALIBRATION_CHCK_FREQ - 1

#define WHITE_LED 5 ///< White LED pin
#define GREEN_LED 6 ///< Green LED pin
#define RED_LED 7 ///<  Red LED pin
#define BATTERY_VOLTAGE_PIN 14 ///< Battery voltage divider pin
#define DS18B20_PIN 2 ///< DS18B20 temperature sensor pin

#define BATTERY_VOLTAGE_R1 47000.0f ///< Battery voltage divider R1 value in Ohms
#define BATTERY_VOLTAGE_R2 8500.0f ///< Battery voltage divider R2 value in Ohms
#define BATTERY_VOLTAGE_DIV_K (BATTERY_VOLTAGE_R2 / (BATTERY_VOLTAGE_R1 + BATTERY_VOLTAGE_R2))


// Due to the non-linearity of ESP32 ADC, the following polynomial is used to correct the error
#define ADC_FIX_K1 0.0115f ///< ADC error correction polynomial coefficients
#define ADC_FIX_K2 -0.0663f
#define ADC_FIX_K3 0.1264f
#define ADC_FIX_K4 -0.0671f
#define ADC_FIX_K5 -0.0581f
#define ADC_FIX_K6 -0.0026f

#define ADC_REAL(x) (x - ADC_ERR(x)) ///< Corrected ADC value
#define ADC_ERR(x) (ADC_FIX_K1 * x * x * x * x * x + ADC_FIX_K2 * x * x * x * x + ADC_FIX_K3 * x * x * x + ADC_FIX_K4 * x * x + ADC_FIX_K5 * x + ADC_FIX_K6) ///< ADC error polynomial

#define ADC_OVERSAMPLE 20 ///< ADC oversampling factor
#define ADC_12BIT_CONV 3.3f / 4096.0f


const PROGMEM float filterk[] = { ///< IMU angular speed filter coefficients
    0.009082479966205859,
    0.015952216480505963,
    0.023234853965074423,
    0.030695728078538806,
    0.038084022811660879,
    0.045143296703246204,
    0.051622456922709739,
    0.057286661223692671,
    0.061927626244948367,
    0.065372845521188064,
    0.067493270732698937,
    0.068209082699060161,
    0.067493270732698937,
    0.065372845521188064,
    0.061927626244948367,
    0.057286661223692671,
    0.051622456922709739,
    0.045143296703246204,
    0.038084022811660879,
    0.030695728078538806,
    0.023234853965074423,
    0.015952216480505963,
    0.009082479966205859,
};
#endif // MAIN_H