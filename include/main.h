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

#define BNO_TICK_FREQ 500
#define BNO_TICK_PERIOD 1000000 / BNO_TICK_FREQ - 1
#define MOTOR_TICK_FREQ 200
#define MOTOR_TICK_PERIOD 1000000 / MOTOR_TICK_FREQ - 1
#define TELEM_FREQ 40
#define TELEM_TICK_PERIOD 1000 / TELEM_FREQ - 1
#define COMM_UPDATE_FREQ 10
#define COMM_UPDATE_PERIOD 1000 / COMM_UPDATE_FREQ - 1
#define PID_SWITCH_TRESHOLD 12.0f

#define MOTOR_TEMP_CHCK_FREQ 500                                  // Value in mHz
#define MOTOR_TEMP_CHCK_PERIOD 1000000 / MOTOR_TEMP_CHCK_FREQ - 1 // NOTE: DS18B20 takes 750ms to read the temperature, if the period is less
                                                                    // temperature readings will block the loop. DON'T set the frequency > 1.33 Hz
#if MOTOR_TEMP_CHCK_PERIOD < 750
    #error "MOTOR_TEMP_CHCK_PERIOD must be greater than 750ms!"
#endif

#define BATTERY_VOLTAGE_CHCK_FREQ 10
#define BATTERY_VOLTAGE_CHCK_PERIOD 1000 / BATTERY_VOLTAGE_CHCK_FREQ - 1

#define KEEP_ALIVE_DEAD 5 * 1000

#define I2C_SLAVE_SCL_PIN 15
#define I2C_SLAVE_SDA_PIN 16
#define I2C_SLAVE_ADDR 0x01
#define I2C_SLAVE_FREQ 100000

#define BNO_SDA 48
#define BNO_SCL 47

#define SPEED_PID_MAXIMUM_OUTPUT 250.0f
#define SPEED_PID_MINIMUM_OUTPUT -SPEED_PID_MAXIMUM_OUTPUT
#define MOTOR_RUNAWAY_TIME 30 * 1000
#define BNO_CALIBRATION_CHCK_FREQ 10
#define BNO_CALIBRATION_CHCK_PERIOD 1000 / BNO_CALIBRATION_CHCK_FREQ - 1

#define WHITE_LED 5
#define GREEN_LED 6
#define RED_LED 7
#define BATTERY_VOLTAGE_PIN 14
#define DS18B20_PIN 2

#define BATTERY_VOLTAGE_R1 47000.0f
#define BATTERY_VOLTAGE_R2 8500.0f
#define BATTERY_VOLTAGE_DIV_K (BATTERY_VOLTAGE_R2 / (BATTERY_VOLTAGE_R1 + BATTERY_VOLTAGE_R2))


//Section Below Fixes ESP32 ADC non-linearity
#define ADC_FIX_K1 0.0115f
#define ADC_FIX_K2 -0.0663f
#define ADC_FIX_K3 0.1264f
#define ADC_FIX_K4 -0.0671f
#define ADC_FIX_K5 -0.0581f
#define ADC_FIX_K6 -0.0026f

#define ADC_REAL(x) (x - ADC_ERR(x))
#define ADC_ERR(x) (ADC_FIX_K1 * x * x * x * x * x + ADC_FIX_K2 * x * x * x * x + ADC_FIX_K3 * x * x * x + ADC_FIX_K4 * x * x + ADC_FIX_K5 * x + ADC_FIX_K6)

#define ADC_OVERSAMPLE 20
#define ADC_12BIT_CONV 3.3f / 4096.0f

#endif // MAIN_H