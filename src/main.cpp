#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

#define KEEP_ALIVE_DEAD 5000

#define I2C_SLAVE_SCL_PIN 15
#define I2C_SLAVE_SDA_PIN 16
#define I2C_SLAVE_ADDR 0x01
#define I2C_SLAVE_FREQ 100000

#define BNO_SDA 48
#define BNO_SCL 47

#define SPEED_PID_MAXIMUM_OUTPUT 250.0f
#define SPEED_PID_MINIMUM_OUTPUT -SPEED_PID_MAXIMUM_OUTPUT
#define MOTOR_RUNAWAY_TIME 30 * 1000

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

PIDConfig orientationPidGains = {.p = 0.08, .i = 0.02, .d = 0.0002};
PIDConfig speedGains = {.p = 15, .i = 1, .d = 0.1};

PID orientationPid(orientationPidGains, -250.0, 250.0);
PID speedPid(speedGains, -250.0, 250.0);

LowPassFIR filter(23);

VehicleConfig rwc;
RWCComHandler comm(&rwc);

uint64_t motorTick, stabTick, commTick, motorRunawayDetectionTick;
int32_t motorMaxSpeedTime;

const PROGMEM float filterk[] = {
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

void setup()
{
    Wire.setPins(BNO_SDA, BNO_SCL);

    Wire1.onReceive(i2cCommReceive);
    Wire1.onRequest(i2cCommRequest);
    Wire1.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SLAVE_SDA_PIN, I2C_SLAVE_SCL_PIN, I2C_SLAVE_FREQ);
    if (!bno.begin())
    {
        while (1)
            ;
    }

    motor0.setInterruptHandler(motor0InterruptHandler);
    motor0.init();
    motor0.enable();

    filter.setCoefficients((float *)filterk);
}

void loop()
{

    if (rwc.state = STAB && millis() - rwc.lastKeepAlive < KEEP_ALIVE_DEAD)
    {
        if (micros() - motorTick > MOTOR_TICK_PERIOD)
        {
            motorTick = micros();
            motor0.tick(micros());
            rwc.motorSpeed = motor0.speed;
        }

        if (micros() - stabTick > BNO_TICK_PERIOD)
        {
            stabTick = micros();
            sensors_event_t rps;
            bno.getEvent(&rps, Adafruit_BNO055::VECTOR_GYROSCOPE);
            rwc.angSpeed = filter.performFiltering(rps.gyro.z);

            if (rwc.mode == SPEED_HOLD)
            {
                speedPid.setpoint = rwc.speedSetpoint;
                orientationPid.reset();
            }

            else if ((rwc.angSpeed > PID_SWITCH_TRESHOLD || rwc.angSpeed < -PID_SWITCH_TRESHOLD) & rwc.mode == ORIENTATION_HOLD)
            {
                speedPid.setpoint = 0.0f;
                orientationPid.reset();
            }

            else if (rwc.mode == ORIENTATION_HOLD)
            {
                imu::Quaternion quat = bno.getQuat();
                imu::Vector<3> euler = quat.toEuler();
                rwc.orientation = euler[0] * RAD_TO_DEG;
                rwc.orientation = 360.0f - ((rwc.orientation < 0.0f) ? rwc.orientation + 360.0f : rwc.orientation);
                float error = rwc.orientation - rwc.orientationSetpoint;
                error = !(error > 180 || error < -180) ? error : (error > 180 ? error - 360 : error + 360);
                speedPid.setpoint = orientationPid.tick(-error, micros());
            }

            motor0.pid.setpoint = speedPid.tick(rwc.angSpeed, micros());
        }
    }

    else
    {
        motor0.brake();
        orientationPid.reset();
        speedPid.reset();
    }

    if(motor0.pid.setpoint == SPEED_PID_MAXIMUM_OUTPUT || motor0.pid.setpoint == SPEED_PID_MINIMUM_OUTPUT){
        motorMaxSpeedTime += millis() - motorRunawayDetectionTick;
        motorRunawayDetectionTick = millis();
    }
    else{
        motorMaxSpeedTime -= millis() - motorRunawayDetectionTick;
        motorRunawayDetectionTick = millis();
    }

    if(motorMaxSpeedTime >= MOTOR_RUNAWAY_TIME){
        rwc.state = IDLE;
        rwc.error |= MOTOR_RUNAWAY;

        motor0.disable();
        orientationPid.reset();
        speedPid.reset();

        motorMaxSpeedTime = 0;
    }

    if (millis() - commTick > COMM_UPDATE_PERIOD)
    {
        commTick = millis();

        rwc.newData |= NEW_ORIENTATION | NEW_ANG_SPEED | NEW_MOTOR_SPEED;
    }

    comm.handler();
}