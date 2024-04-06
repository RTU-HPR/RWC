#include "main.h"

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

PIDConfig orientationPidGains = {.p = 0.08, .i = 0.01, .d = 0.0002};
PIDConfig speedGains = {.p = 15, .i = 1, .d = 0.1};

PID orientationPid(orientationPidGains, -250.0, 250.0);
PID speedPid(speedGains, -250.0, 250.0);

LowPassFIR filter(23);

OneWire oneWire(DS18B20_PIN);
DallasTemperature motorTemp(&oneWire);

VehicleConfig rwc;
RWCComHandler comm(&rwc);

uint64_t motorTick, stabTick, commTick, motorRunawayDetectionTick, calibrationTick, motorTempTick, batteryTick;
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

    motorTemp.begin();
    motorTemp.setWaitForConversion(false);
    motorTemp.setResolution(12);
    motorTemp.requestTemperatures();

    rwc.state = IDLE;
    rwc.mode = ORIENTATION_HOLD;

    pinMode(WHITE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
}

void loop()
{

    if (rwc.state == STAB && millis() - rwc.lastKeepAlive < KEEP_ALIVE_DEAD)
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

    if ((motor0.pid.setpoint == SPEED_PID_MAXIMUM_OUTPUT || motor0.pid.setpoint == SPEED_PID_MINIMUM_OUTPUT) && rwc.state == STAB)
    {
        motorMaxSpeedTime += millis() - motorRunawayDetectionTick;
        motorRunawayDetectionTick = millis();
    }
    else if (rwc.state == STAB)
    {
        motorMaxSpeedTime -= millis() - motorRunawayDetectionTick;
        if (motorMaxSpeedTime < 0)
        {
            motorMaxSpeedTime = 0;
        }
        motorRunawayDetectionTick = millis();
    }

    if (motorMaxSpeedTime >= MOTOR_RUNAWAY_TIME)
    {
        rwc.state = IDLE;
        rwc.error |= MOTOR_RUNAWAY;

        motor0.setPower(0, 0);
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

    if (millis() - calibrationTick > BNO_CALIBRATION_CHCK_PERIOD)
    {
        calibrationTick = millis();
        uint8_t calibration[4];
        bno.getCalibration(&calibration[0], &calibration[1], &calibration[2], &calibration[3]);

        rwc.calibration = 0;
        rwc.calibration |= calibration[0];
        rwc.calibration |= calibration[1] << 2;
        rwc.calibration |= calibration[2] << 4;
        rwc.calibration |= calibration[3] << 6;

        if (calibration[1] == 3)
        {
            digitalWrite(GREEN_LED, 1);
        }
        else
        {
            digitalWrite(GREEN_LED, 0);
        }

        if (calibration[3] == 3)
        {
            digitalWrite(RED_LED, 1);
        }
        else
        {
            digitalWrite(RED_LED, 0);
        }
    }

    if (millis() - motorTempTick > MOTOR_TEMP_CHCK_PERIOD)
    {
        rwc.motorTemp = motorTemp.getTempCByIndex(0);
        motorTemp.requestTemperatures();
        motorTempTick = millis();
    }

    if (millis() - batteryTick > BATTERY_VOLTAGE_CHCK_PERIOD)
    {
        batteryTick = millis();
        uint32_t totalMv = 0;
        for (uint8_t i = 0; i < 20; i++)
        { // Do several measurments to filter out the noise.
            totalMv += analogRead(BATTERY_VOLTAGE_PIN);
        }
        rwc.batteryVoltage = (((float)totalMv / (20.0f * 4095)) * 3.3) / BATTERY_VOLTAGE_DIV_K;
    }

    comm.handler();
}