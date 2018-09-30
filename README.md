
# <p align="center">DIY flight controller from scratch</p>

<p align="center">
    <img src="/ReadmePictures/Drone.jpg">
    <img src="/ReadmePictures/v1_v2_boards.jpg">
    Pictures: First hardware version using Arduino Uno board and second hardware version using Arduino Nano board
</p>

---------------------------Table of contents----------------------------

[**1. Introduction**](#intro)

1.1. Purpose

1.2. Progress state

1.3. Videos

[**2. Required sensors**](#requiredSensors)

[**3. Accro mode explained (aka manual)**](#accroMode)

3.1. Angle feedback

3.2. Speed stabilization

[**4. Angle mode explained (aka automatic leveling mode)**](#angleMode)

4.1. Angles and angular speeds feedback

4.2. Stabilization

[**5. Hardware configuration**](#hardwareConfiguration)

5.1. Components list

5.2. Connexions

[**6. Software setup**](#softwareSetup)

6.1 Using Arduino IDE

6.2 Using PlatformIO

6.3. Using Docker

[**7. Software detailed design**](#softwareDetailedDesign)

7.1 Source Code overview

7.2 The state machine

7.3 CPPM Reception

7.4 Failsafe

[**8. Appendix**](#appendix)

8.1 My BenchTest

8.2 Exemple of First Person View configuration(FPV)

[**10. Bibliography**](#bibliography)

-------------------------------------------------------------------
## Warning

![Danger.jpg](/ReadmePictures/Danger.jpg "Danger")

Before starting this project, you must know that spinning propellers are very dangerous for your eyes and people around.
When using the benchtest indoor, you must wear large protective glasses, and set up a power limit.
For outside tests, choose a very large area, with nobody around.

![Protection_glasses.jpg](/ReadmePictures/Protection_glasses.jpg "Protection_glasses")

## 1. Introduction <a id="intro"></a>
### 1.1 Purpose
The aim of this project is to develop a very simple quadrirotor flight controller from scratch, using an Arduino and inertial sensors.
There is two benefits:
* to understand UAV flight stabilization
* to have our own system, with no limits for customization: you can add all the sensors you want.

In this project, the two main flight modes are addressed:
* Accrobatic mode (or manual mode): the simplest to code, but it requires flight skills
* Angle mode: more complex to implement, but easier to fly: UAV automatically goes back to
horizontal

I strongly advice to start by implementing the accrobatic mode, since a good accro mode will be a solid fundation
for angle mode, and it is easy and fast to implement.

### 1.2 Progress state
Releases have been successfully tested during flights tests: I have some nice flights in FPV on a
450mm frame, both in accro and angle modes.
BUT, this project is sometimes updated, mainly for code format, and unfortunately, I cannot realize flight tests for each submit.
So, I cannot garantee that the last commit will allow to flight without some corrections.

Note that you may also have to tune PID according to your configuration.

I advice you to use a large frame (450mm for exemple), because it is more stable, and I did not test the software on smaller frames.

### 1.3 Videos

<a href="https://www.youtube.com/watch?v=niiIYhLCFx0" target="_blank" rel="noopener noreferrer">Loop control indoor test</a>

<a href="https://www.youtube.com/watch?v=C8MZH-K4qus" target="_blank" rel="noopener noreferrer">Outdoor flight test</a>

## 2. Required sensors <a id="requiredSensors"></a>

An IMU (aka MPU) is the only sensor needed for flight.

An « Inertial Measurement Unit » is a MEMS, "Microelectromechanical system", composed by a 3 axis gyroscope sensor, and 3 axis accelerometer sensor.

| IMU sensor      | Function |Pros      | Cons |
| -------------- | -------------- | -------------- | -------------- |
| Gyroscope | Measure angular speeds around each axis (°/sec) | Fast | Drift along time |
| Accelerometer | Measure linear acceleration on each axis (g) | Slow |  Noizy and not usable when UAV is moving |

<p align="center">
<img src="/ReadmePictures/IMU.jpg" width="24%" />
</p>

## 3. Accro mode (aka manual) <a id="accroMode"></a>

Accrobatic mode implements minimal stabilization algorithms to make flight possible. UAV is not able to auto-leveling, and pilot skills are required.

This mode is the required base for the angle mode, and is easier to realize: do not try to code angle mode if accro mode is not working fine.

### 3.1 Compute angle speed feedback

Accrobatic mode algorithm only needs UAV angular speeds and angular speed command as entry data. Angular speeds are computed from raw gyroscope data integration.

Exemple of pitch angle speed computation using gyroscopes:
```
// currentPitch = previousPitch + rawSensorPitch*loopTime
_pos[0] = _pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time;
```
### 3.2 Speed stabilization  <a id="stabilization"></a>

A closed-loop system is needed to control angular spesoftwareDetailedDesigneds.

This system compares angular speed command to angular speed feedback, and computes a new motor power to apply.

![AsservissementAccro](/ReadmePictures/AsservissementAccro.jpg "AsservissementAccro")


## 4. Angle mode (aka automatic leveling mode) <a id="angleMode"></a>

It is a more sophisticated flight mode than the accrobatic mode: now, UAV is able to auto-level.

Auto-leveling mode entry data is UAV attitude: the angles from the horizontal (roll, pitch, yaw) and the angular speeds.

### 4.1 Compute angles and angular speeds feedback

Both angles speed and angles are required to compute reliable attitude angle feedback along time.

Both gyroscopes and accelerometers are involved.

#### 4.1.1 IMU gyroscopes

As seen previously, Angular speeds are computed from raw gyroscope data integration.

#### 4.1.1 IMU accelerometers

Attitude angles are computed from accelerations:

When UAV is not moving, or if it is moving at constant speed, accelerometers measure gravity, ie
vertical acceleration.
Angle between UAV and the gravity vector is computed by trigonometry.
Be carefull, this measure is faulty when UAV accelerates.

Exemple of pitch attitude angle computation using accelerometers:

```
_pos[0] = atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI);
```

#### 4.1.2 Data merging: the complementary filter

Complementary filter merges gyroscopes and accelerometers data, to get both sensors benefits, and to reduce theirs drawbacks:

* The gyroscope if fast but it drifts along time.

* Accelerometers are not fast enougth, they are noizy, and they are usable only when the UAV is not moving, and when UAV receive only earth acceleration.

The complementary filter mask their respective errors:

- A low-pass filter is applied on accelerometer data to filter the noize and the unwanted accelerations: these data are usefull on a long time period, fast changes have to be eliminated.
- A high-pass filter is applied on the gyrosope data: these data are usefull on short time period, but they drift and accumulate errors on long time periods.

![FiltreComplementaire](/ReadmePictures/FiltreComplementaire.jpg "FiltreComplementaire")

**Coding exemple**

  ```
 _pos[0] = HighPassFilterCoeff*(_pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time) +
 (LowPassFilterCoeff)*((atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI));
```

Note: LowPassFilterCoeff = 1 - HighPassFilterCoeff

**Coefficients computation**

HighPassFilterCoeff is computed from the Time constant.

Time constant is a compromise between UAV acceleration filtering, and gyroscopes drift:
* Too low, accelerometer noize are not eliminated
* Too high, gyroscopes drift is not compensated

```
timeConstant = (HighPassFilterCoeff*dt) / (1-HighPassFilterCoeff)
=> HighPassFilterCoeff = timeConstant / (dt + timeConstant)
```

Time constant in this project is set to 5 milliseconds. It implies a coefficient "HighPassFilterCoeff" of 0.9995 for a loop time of 2.49 ms.

**Note:** More efficient filters like the Kalman one are used in UAV, but they are more complicated to understand, and we want a very simple DIY software!

### 4.2 Stabilization

A double closed-loop system is needed to control attitude angles. It consists of a speed closed-loop [like the one in the accro mode], inside a position closed-loop.

This system compares angle command to angle feedback, and computes a new motor power to apply. Angular feedback is computed using a complementary filter to merge gyroscope dans accelerometer data.

The pilot controls each attitude angle, and if transmitter sticks are centered, command is 0°, and UAV automatically goes back to horizontal.

![AsservissementAngle](/ReadmePictures/AsservissementAngle.jpg "AsservissementAngle")

## 5. Hardware configuration <a id="hardwareConfiguration"></a>

### 5.1 Components list

As exemple, my hardware configuration for a 450mm quad:

| Component      | Reference      |
| -------------- | -------------- |
| **Microcontroller board** | Arduino Nano/Uno |
| **ESC** | Afro 20A-Simonk firmware 500Hz, BEC 0.5A 1060 to 1860 us pulse width |
| **Motors** | Multistar 2216-800Kv 14 Poles - 222W Current max: 20A Shaft: 3mm 2-4S|
| **Propellers** | 10x4.5 SF Props 2pc CW 2 pc CCW Rotation (Orange) |
| **Battery** | Zippy Flightmax 3000mAh 4S |
| **Receiver** | OrangeRx R617XL CPPM DSM2/DSMX 6 ch |
| **IMU** | MPU6050 (GY-86 breakout board)|
| **Compass** | HMC5883L (GY-86 breakout board) |
| **Buzzer** | Matek lost model beeper - Built-in MCU |
| **Frame** | Diatone Q450 Quad 450 V3. 450 mm wide frame choosen for better stability and higher autonomy (the lower the size, the lower the stability).|

### 5.2 Connexions
TODO: add receiver in schematic

**Full view:**

<img src="/ReadmePictures/schemaElectriqueDroneFull.jpg" width="80%"/>

**Zoomed view:**

<img src="/ReadmePictures/SchemaElectriqueDroneZoom.jpg" width="50%"/>

| Arduino pin      | Component      |
| -------------- | -------------- |
| PD2 | receiver |
| PD4 | ESC0 |
| PD5 | ESC1 |
| PD6 | ESC2 |
| PD7 | ESC3 |
| PC0 | potentiometer |
| PC4 | SDA MPU6050 |
| PC5 | SCL MPU6050 |

![flightConfiguration](/ReadmePictures/flightConfiguration.jpg "flightConfiguration")

## 6.Software setup <a id="softwareSetup"></a>

### 6.1 Using Arduino IDE
With minor modifications, project can be build using Arduino IDE:
* rename "main.cpp" to "CodeDroneDIY.ino"
* copy all source files from "CodeDroneDIY/src" to "CodeDroneDIY"
* launch and compile "CodeDroneDIY.ino" using Arduino IDE

### 6.2. Using PlatformIO
PlatformIO is an open source ecosystem for IoT development.

#### 6.2.1. PlatformIO installation
```sudo apt-get update
sudo apt-get install python-pip
sudo pip install --upgrade pip && sudo pip install -U platformio==3.5.2
platformio platform install atmelavr --with-package=framework-arduinoavr
platformio lib install MPU6050
pio lib install "I2Cdevlib-MPU6050"
```

Optional, for code format:

```sudo apt-get install -y clang-format```

#### 6.2.2. Build project
```platformio run```

#### 6.2.3. Flash target
```platformio upload --upload-port/ttyACM0 ```

### 6.3. Using Docker
The development tool "Docker" is a container platoform: it is a stand-alone, executable package
of a piece of software that includes everything needed to run it: code, runtime, system tools,
 system libraries, settings. It isolates software from its surroundings.
* Install Docker
* Move inside docker's folder: ```cd docker```
* Build docker image: ```make image```
* Format code: ```make format-all```
* build project: ```make build-codedronediy```

## 7. Software detailed design <a id="softwareDetailedDesign"></a>

### 7.1 Source code overview

![DiagrammeUML](/ReadmePictures/DiagrammeUML.jpg "DiagrammeUML")

| File      | Description      |
| -------------- | -------------- |
| main.cpp | Contains initialization function "setup()" and main loop "loop()" |
| Attitude.cpp/h | Attitude angles computation (roll, pitch, yaw angles) |
| Stabilization.cpp/h | Closed loop correction: computes new command from tx sticks and attitude |
| PID.cpp/h | Proportionnal, integral, derivative loop correction |
| Reception.h | Receiver CPPM signal acquisition using INT0 |
| ESC.cpp/h | Manage an ESC: pin, PWM to set |
| StateMachine.h | State machine |
| Time.h | To measure loop time and elapsed time |
| Math.h | Mathermatical functions: mean and delta max computations|
| checkIMU.cpp/h | To check IMU sensors |

### 7.2 The state machine

Due to security reasons, the UAV cannot start running with a flight mode enabled. Six states are defined to enable or disable flight mode safely. These states are ruled by a statemachine.

The six states StateMachine:

![MachineEtats](/ReadmePictures/MachineEtats.jpg "MachineEtats")

| State      | Description      |
| -------------- | -------------- |
| Initialization | UAV must on the ground, horizontal, and not moving. Sensors offsets are computed. Then, if receiver switch is disarmed, the system changes to the next state. |
| Starting | UAV is ready to be armed using tranceiver switch. When the receiver switch is set to a flight mode, the system changes to the corresponding state. |
| Angle/Accro | Throttle is enabled, flight can start. After 5 seconds of power idle, the system is set to "safety" state. |
| Safety | Throttle command is disabled and power cannot be set. Then, if receiver switch is disarmed, the system changes to the next state.|
| Disarmed | Throttle is disabled. When the receiver switch is set to a flight mode, the system changes to the corresponding state.|

### 7.3. Reception

A CPPM receiver is used to fly the UAV using a remote control.

CPPM (Pulse Position Modulation) reception  allows to receive all channels using only one entry pin. Each rising edge correpond to the end of the previous channel impulsion, and at the beginning of the next channel impulsion.
Elapsed time between two rising edge correspond to the pulse width of a given channel.

![CPPM](/ReadmePictures/CPPM.jpg "CPPM")

In this projet, each pulse width is measured using INT0, and then stored in the correponding channel of an array.

### 7.4 Failsafe

For security, you must set the failsafe to cut motors power when radio link is lost.

To set the failsafe:
1. Put transmitter sticks in the configuration wanted when reception is lost
2. Bind the receiver with the transmitter

Transmitter configuration used during the « bind » operation defines the « failsafe. »

## 8. Appendix <a id="appendix"></a>

### 8.1 The benchtest

<img src="/ReadmePictures/BenchTest01.jpg" width="40%"/>

### 8.2 FPV - First Person View  <a id="firstPersonView"></a>

My cheap configuration:

| Component      | Reference      |
| -------------- | -------------- |
| **Googgles** | Quanum DIY FPV Goggle V2 Pro |
| **Googgles battery** | 1000 mAh 3S |
| **Receiver** | Eachine RC832 Boscam FPV 5.8G 48CH Wireless AV Receiver  |
| **Receiver antenna** | DYS FPV 5.8G Antenna 4dBi Mushroom Antenna RHCP TX RX |
| **Camera** | Foxeer XAT600M HS1177 600TVL CCD 2.8MM IR Mini FPV Camera IR Blocked 5-22v |
| **Camera antenna** | Realacc 5.8G 5dBi 50W RHCP Omnidirectional 3 Leaf Clover FPV Antenna Red |
| **Video transmitter** | Upgrade Aomway Mini 5.8Ghz 200mW 32CH AV Wireless Transmitter Module |

## 10. Bibliography <a id="bibliography"></a>

* Arduino

https://www.arduino.cc/en/Reference/Libraries.html

https://www.arduino.cc/en/Reference/HomePage

* PWM

https://librepilot.atlassian.net/wiki/display/LPDOC/PWM,+PWMSync,+Oneshot+Output

http://forum.arduino.cc/index.php?topic=46487.0

* PPM

http://frskytaranis.forumactif.org/t4426-tuto-pwm-cppm-ccpm-ppm-s-bus-s-port-kesako

* PID

http://www.fpv-passion.fr/docteur-pid/

* Quadricopter

https://www.mondrone.net/fabriquer-quadricoptere-la-propulsion/

* Data processing

https://ericjformanteaching.wordpress.com/2013/10/08/smoothing-sensor-input/

* Data merging

http://www.mouser.fr/applications/sensor_solutions_mems/

http://www.pieter-jan.com/node/11

* Programme python

Andy BAKER

http://pythonfiddle.com/andy-baker-quadcopter/

* Vibrations

http://ardupilot.org/plane/docs/common-vibration-damping.html
