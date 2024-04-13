# Reaction wheel controller (RWC) firmware repository.
![](/static/vehicle-render.png)
***
## Main requirements:
- Stabilize and control the vehicle according to the BFC commands.
- Make telemetry data about systems & sensors available through the
communication protocol.

## Design overview:
- RWC utilizes 3 PID loops: 2 that are directly used to control the speed and orientation of
the vehicle and 1 loop used to control the speed of the motor itself. Decicion to divide
control process into these loops was made to ensure stability of the platform as well as
make it easier to tune and test it.
- Two readings from the IMU (`BNO055`) are used: quaternion orientation data,
which is converter into euler angles to get the direction in which the platform is pointing and raw gyroscope (angular rates) readings. Orientation data is fused @ the sensor and
thus does not require additional filtering. Angular rates require to be filtered using
LPF filter and the fed into the PID loop.
- I2C is used to establish communication with the BFC. Current implementation utilizes
checksums to ensure data integrity. Heartbeat (keepalive) packet is also implemented to
prevent cases where the datalink is broken. I2C registers and their values are described
in the main documentation. Telemetry loop update frequency can be set in the `main.h` file.
- RWC does not implement any data logging due to the lack of hardware implementation. Instead all the data is sent through telemetry and then logged on BFC.
- 2 on-board LEDs are used to show the `BNO055` internal filter calibration status (green and red). The LEDs light up when the according calibration value reaches 3 (highest level).
Green LED is used for gyroscope calibration, red for magnetometer calibration.

## Safety algorithms:
- Battery readings can be used to decide whehter it is safe to enable the reaction wheel.
- Motor runaway timer is implemented to disable the system in case the motor reaches its maximum
speed and thus control over the system is lost.
- Motor temperature readings can be used to determine if the motor is in operational conditions.
- Watchdog timer that reboots the system if not reset after 5 seconds.

## Changes & fixes:
- The motor is driven without the boost converter, as it was creating strong magnetic field,
which interfered with magnetometer readings and made the fusion algorithm very unstable.
- DS18B20 temperature sensor was added to monitor the temperature of the motor.
- RWC <-> BFC protocol was changed to I2C, 2 pullup resistors were added.
- 2 Pullup resistors for the motor encoder were added.

## Usage:
1. Power on the vehicle.
2. Calibrate the magnetometer. To ensure calibration use either LEDs or telemetry data. __Note that after the calibration position will be relative to the North pole.__
3. Stabilization mode can be enabled after these procedures. (Further explained in [master driver requirements](#master-driver-requirements))

## Master driver requirements:
- Master driver (PFC / BFC in most of the cases) is used to control & operate the RWC. Commands are given through the I2C protocol on the hardware level and through RWC master SDK on the software level.
- Master driver is responsible for enabling / disabling the RWC as well as selecting the mode of orientation.
- Orientation data from the RWC and boards' sensors needs to be synchronized. The fact that
orientation is relative to the North pole shall be taken into account.