
# <p align="center">DIY flight controller from scratch</p>

<p align="center">
    <img src="/ReadmePictures/Drone.jpg">
    <img src="/ReadmePictures/v1_v2_boards.jpg">
    Pictures: First hardware version using Arduino Uno board and second hardware version using Arduino Nano board
</p>

---------------------------Table of contents----------------------------

[**1. Project introduction**](#projectIntro)

1.1. Purpose

1.2. Progress state

1.3. Videos

[**2. IMU**](#IMU)

[**3. Accro mode (aka manual)**](#accroMode)

3.1. Angle feedback

3.2. Speed stabilization

[**4. Angle mode (aka automatic leveling mode)**](#angleMode)

4.1. Angle feedback

4.2. Stabilization

[**5. State machine**](#stateMachine)

[**6. CPPM Reception**](#cppmReception)

[**7. Source code organization « CodeDroneDIY »**](#sourceCodeOrganization)

[**8. Hardware configuration**](#hardwareConfiguration)

8.1. Components list

8.2. Connexions

8.3. Failsafe

8.4. The benchtest

[**9. Project setup**](#projectSetup)

8.1 Using Arduino IDE

8.2 Using PlatformIO

8.2.1. PlatformIO installation

8.2.2. Build project

8.2.3. Flash target

8.3. Using Docker

8.2.3. Flash target

8.3. Using Docker

[**9. First Person View (FPV)**](#firstPersonView)

[**10. Bibliography**](#bibliography)

-------------------------------------------------------------------
## Warning
<p align="left">
![Danger.jpg](/ReadmePictures/Danger.jpg "Danger")
</p>

Before starting this project, you must know that spinning propellers are very dangerous for your eyes and the persons around.
When testing using the benchtest indoor, you must wear large protective glasses, and set up a power limit.
For outside tests, choose a very large area, with nobody around.

<p align="left">
![Protection_glasses.jpg](/ReadmePictures/Protection_glasses.jpg "Protection_glasses")
</p>

## 1. Project introduction <a id="projectIntro"></a>
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

## 2. IMU <a id="IMU"></a>

It is the main UAV sensor, required to make flight possible.

An IMU, « Inertial Measurement Unit » is a MEMS, "Microelectromechanical system", composed by 3 axis gyroscope and 3 axis accelerometer sensors.

| Sensor      | Function |Pros      | Cons |
| -------------- | -------------- | -------------- | -------------- |
| Gyroscope | Measure angular speed around each axis in degrees by second | Fast | Drift along time |
| Accelerometer | Measure linear acceleration on each axis in "g" | Slow |  Noizy and not usable when UAV is moving |

Attitude rotation speeds are given by gyroscopes, no computation is needed.

These two sensors are complementary to compute attitude angles: merging their data compensate their
drawbacks.

<p align="center">
<img src="/ReadmePictures/IMU.jpg" width="24%" />
</p>

This projet use an MPU6050 IMU sensor which communicates with the microcontroler using I2C protocol.

## 3. Accro mode (aka manual) <a id="accroMode"></a>

Accrobatic mode is the required base for the angle mode, and is easier to realize: do not try to code angle mode if accro mode is not working fine.

In this mode, the UAV is able to flight, but not to auto-leveling: pilot skills are required.

### 3.1 Compute angle speed feedback

Accro mode only need UAV angles speed.

Angles speed are computed from raw gyroscope data integration.

Exemple of pitch angle speed computation using gyroscopes:
```
// currentPitch = previousPitch + rawSensorPitch*loopTime
_pos[0] = _pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time;
```
### 3.2 Speed stabilization  <a id="stabilization"></a>

Using angles speed feedback, a speed control loop can be realized. The pilot controls rotation speeds around each axis.

![AsservissementAccro](/ReadmePictures/AsservissementAccro.jpg "AsservissementAccro")

Note: Only gyroscopics data are used: there is no need for a data merging filter.

## 4. Angle mode (aka automatic leveling mode) <a id="angleMode"></a>

The first step for auto-leveling mode is to compute UAV attitude: the angles from the horizontal (roll, pitch, yaw) and the rotation speeds.
UAV attitude is an entry for automatic stabilization, computed from IMU data.

### 4.1 Compute angle feedback

Both angles speed and angles are required to compute reliable attitude angle feedback along time.

Both gyroscopes and accelerometers are involved.

#### 4.1.1 IMU gyroscopes

Angles speed are computed from raw gyroscope data integration.

#### 4.1.1 IMU accelerometers

Attitude angles are computed from accelerations:

When UAV is not moving, or if it is moving at constant speed, accelerometers measure gravity, ie
vertical acceleration.
Angle between UAV and the gravity vector is computed by trigonometry.
Be carefull, this measure is faulty when UAV accelerates.

Exemple of pitch attitude angl ecomputation using accelerometers:
    _pos[0] = atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI);

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

**Coefficients comutation**

Filter time constant is a compromise between UAV acceleration filtering, and gyroscopes drift:
* Too low, accelerometer noize are not eliminated
* Too high, gyroscopes drift is not compensated

```
timeConstant = (HighPassFilterCoeff*dt) / (1-HighPassFilterCoeff)
=> HighPassFilterCoeff = timeConstant / (dt + timeConstant)
```


Time constant in this project is set to 5 milliseconds. It implies a coefficient "HighPassFilterCoeff" of 0.9995 for a loop time of 2.49 ms.

**Note:** More efficient filters like the Kalman one are used in UAV, but they are more complicated to understand, and we want a very simple DIY software!

### 4.2 Stabilization

It is a position control loop. The pilot controls each attitude angle. If transmitter sticks are
centered, command is 0°, and UAV automatically goes back to horizontal.

It consists in a speed control loop inside a position control loop.

Attitude is computed using a complementary filter, merging gyroscope dans accelerometer data.

![AsservissementAngle](/ReadmePictures/AsservissementAngle.jpg "AsservissementAngle")

## 5. State machine <a id="stateMachine"></a>

The system has 6 states:

![MachineEtats](/ReadmePictures/MachineEtats.jpg "MachineEtats")

After 5 secondes of power idle, the system is set into "security" state: throttle command is disabled and power cannot be set.

To arm again the system, pilot has to disarm it, and then he has to choose a flight mode "angle" or "accro".

## 6. CPPM reception <a id="cppmReception"></a>

CPPM (Pulse Position Modulation) reception  allows to receive all channels using only one entry pin. Each rising edge correpond to the end of the previous channel impulsion, and at the beginning of the next channel impulsion.
Elapsed time between two rising edge correspond to the pulse width of a given channel.

![CPPM](/ReadmePictures/CPPM.jpg "CPPM")

In this projet, each pulse width is measured using INT0, and then stored in the correponding channel of an array.

## 7. Source code organization « CodeDroneDIY » <a id="sourceCodeOrganization"></a>

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

## 8. Hardware configuration <a id="hardwareConfiguration"></a>

### 8.1 Components list

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

### 8.2 Connexions
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


### 8.3 Failsafe

For security, you must set the failsafe to cut motors power when radio link is lost.

To set the failsafe:
1. Put transmitter sticks in the configuration wanted when reception is lost
2. Bind the receiver with the transmitter

Transmitter configuration used during the « bind » operation defines the « failsafe. »

### 8.4 The benchtest

<img src="/ReadmePictures/BenchTest01.jpg" width="40%"/>

## 9.Project setup <a id="projectSetup"></a>

### 9.1 Using Arduino IDE
With minor modifications, project can be build using Arduino IDE:
* rename "main.cpp" to "CodeDroneDIY.ino"
* copy all source files from "CodeDroneDIY/src" to "CodeDroneDIY"
* Launch and compile "CodeDroneDIY.ino" using Arduino IDE

### 9.2. Using PlatformIO
PlatformIO is an open source ecosystem for IoT development.

#### 9.2.1. PlatformIO installation
```sudo apt-get update
sudo apt-get install python-pip
sudo pip install --upgrade pip && sudo pip install -U platformio==3.5.2
platformio platform install atmelavr --with-package=framework-arduinoavr
platformio lib install MPU6050
pio lib install "I2Cdevlib-MPU6050"
```

Optional, for code format:

```sudo apt-get install -y clang-format```

#### 9.2.2. Build project
```platformio run```

#### 9.2.3. Flash target
```platformio upload --upload-port/ttyACM0 ```

### 9.3. Using Docker
The development tool "Docker" is a container platoform: it is a stand-alone, executable package
of a piece of software that includes everything needed to run it: code, runtime, system tools,
 system libraries, settings. It isolates software from its surroundings.
* Install Docker
* Move inside docker's folder: ```cd docker```
* Build docker image: ```make image```
* Format code: ```make format-all```
* build project: ```make build-codedronediy```

## 10. FPV - First Person View  <a id="firstPersonView"></a>
| Component      | Reference      |
| -------------- | -------------- |
| **Googgles** | Quanum DIY FPV Goggle V2 Pro |
| **Googgles battery** | 1000 mAh 3S |
| **Receiver** | Eachine RC832 Boscam FPV 5.8G 48CH Wireless AV Receiver  |
| **Receiver antenna** | DYS FPV 5.8G Antenna 4dBi Mushroom Antenna RHCP TX RX |
| **Camera** | Foxeer XAT600M HS1177 600TVL CCD 2.8MM IR Mini FPV Camera IR Blocked 5-22v |
| **Camera antenna** | Realacc 5.8G 5dBi 50W RHCP Omnidirectional 3 Leaf Clover FPV Antenna Red |
| **Video transmitter** | Upgrade Aomway Mini 5.8Ghz 200mW 32CH AV Wireless Transmitter Module |

## 11. Bibliography <a id="bibliography"></a>

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
