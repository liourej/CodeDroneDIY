
# <p align="center">DIY flight controller from scratch</p>

<p align="center">
    <img src="/ReadmePictures/Drone.jpg">
    <img src="/ReadmePictures/v1_v2_boards.jpg">
    Pictures: First hardware version using Arduino Uno board and second hardware version using Arduino Nano board
</p>

---------------------------Table of contents----------------------------

**1. Project purpose**

**2. Attitude computation**

2.1. IMU

2.1.1. Gyroscopes

2.1.2. Accelerometers

2.2 .Data merging: the complementary filter

**3. Stabilization**

3.1. Accro mode (gyroscopes only)

3.2. Angle mode (gyroscopes and accelerometers)

3.3. Height stabilization

**4. State machine**

**5. CPPM Reception**

**6. Source code organization « CodeDroneDIY »**

**7. Hardware configuration**

7.1. Components list

7.2. Connexions

7.3. Failsafe

7.4. The benchtest

**8. Project setup**

8.1 Using Arduino IDE

8.2 Using PlatformIO

8.2.1. PlatformIO installation

8.2.2. Build project

8.2.3. Flash target

8.3. Using Docker

**9. First Person View (FPV)**

**10. Bibliography**

-------------------------------------------------------------------
## Warning
![Danger.jpg](/ReadmePictures/Danger.jpg "Danger")

Before starting this project, you must know that spinning propellers are very dangerous for your eyes and the persons around.
When testing using the benchtest indoor, you must wear large protective glasses, and set up a power limit.
For outside tests, choose a very large area, with nobody around.

![Protection_glasses.jpg](/ReadmePictures/Protection_glasses.jpg "Protection_glasses")

## 1. Project introduction
### 1.1 Purpose
The aim of this project is to develop a very simple quadrirotor flight controller from scratch, using an Arduino and inertial sensors.
There is two benefits:
* to understand UAV flight stabilization
* to have our own system, with no limits for customization: you can add all the sensors you want.

In this project, the two main flight mode are addressed:
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

## 2. Attitude computation <a id="Test"></a>

The first step to stabilize an UAV is to compute its attitude: the angles from the horizontal (roll,
    pitch, yaw) and the rotation speeds.
UAV attitude is an entry for automatic stabilization, computed from IMU data.

### 2.1 IMU

An IMU, « Inertial Measurement Unit » is a MEMS, "Microelectromechanical system", composed by 3 axis gyroscope and 3 axis accelerometer sensors.

| Sensor      | Function |Pros      | Cons |
| -------------- | -------------- | -------------- | -------------- |
| Gyroscope | Measure angular speed around each axis in degrees by second | Fast | Drift along time |
| Accelerometer | Measure linear acceleration on each axis in "g" | Slow |  Noizy and not usable when UAV is moving |

These 2 sensors are complementary to compute attitude angles: merging their data compensate their
defaults.
Attitude rotation speeds are given by gyroscopes, no computation is needed.

![IMU](/ReadmePictures/IMU.jpg "IMU")

This projet use an MPU6050 IMU sensor which communicates with the microcontroler using I2C protocol.

#### 2.1.1 Gyroscopes

**Flight modes involved:** Both

**Output data:** Attitude rotation speeds. They  are the only physical quantity needed for accro mode
stabilization.

**Computed data:** Attitude angles are computed from raw gyroscope data integration. These computed data are needed for
angle mode stabilization.

Exemple of pitch attitude angle computation using gyroscopes:
```
// currentPitch = previousPitch + rawSensorPitch*loopTime
_pos[0] = _pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time;
```

#### 2.1.2 Accelerometers

**Flight modes involved:** Only angle mode stabilization.

**Output data:** Acceleration on each axis

**Computed data:** Attitude angles are computed from accelerations:
when UAV is not moving, or if it is moving at constant speed, accelerometers measure gravity, ie
vertical acceleration.
Angle between UAV and the gravity vector is computed by trigonometry.
Be carefull, this measure is faulty when UAV accelerates.

Exemple of pitch attitude angl ecomputation using accelerometers:
    _pos[0] = atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI);

## 2.2 Data merging: the complementary filter

**Flight modes involved:** Data mergin is only needed for angle mode stabilization: it is use to compute attitude angles.

**Explanation:** The gyroscope sensor drift.
Accelerometers are not fast enougth, they are noizy, and they are usable only when the UAV is not moving, and when UAV receive only earth acceleration.

The complementary filter is a very simple filter which allows to merge data from gyroscopes and accelerometers, and mask their respective errors:

- A low-pass filter is applied on accelerometer data to filter the noize and the unwanted accelerations: these data are usefull on a long time period, fast changes have to be eliminated.
- A high-pass filter is applied on the gyrosope data: these data are usefull on short time period, but they drift and accumulate errors on long time periods.

![FiltreComplementaire](/ReadmePictures/FiltreComplementaire.jpg "FiltreComplementaire")

Complementary filter time constant is a compromise between UAV acceleration filtering, and gyroscopes drift:
* Too low, accelerometer noize are not eliminated
* Too high, gyroscopes drift is not compensated

**Computation:**

  ```
 _pos[0] = HighPassFilterCoeff*(_pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time) +
 (LowPassFilterCoeff)*((atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI));
```
Remark: LowPassFilterCoeff = 1 - HighPassFilterCoeff

Time constant in this project is set to 5 seconds. It implies a coefficient "HighPassFilterCoeff" of 0.9995 for a loop time of 2.49 ms.
Be carefull, the time constant depends on the control loop time:

timeCste = HighPassFilterCoeff*dt/(1-coeff)

HighPassFilterCoeff = timeCste/(dt + timeCste)

**Note:** More efficient filters like the Kalman one are used in UAV, but they are more complicated to understand, and we want a very simple DIY software!

## 3. Stabilization
### 3.1 Accro mode (gyroscopes only)
It is a speed control loop. The pilot controls rotation speeds around each axis.
Only gyroscopics data are used: there is no need for a complementary filter.

![AsservissementAccro](/ReadmePictures/AsservissementAccro.jpg "AsservissementAccro")

### 3.2 Angle mode (gyroscopes and accelerometers)

It is a position control loop. The pilot controls each attitude angle. If transmitter sticks are
centered, command is 0°, and UAV automatically goes back to horizontal.

It consists in a speed control loop inside a position control loop.

Attitude is computed using a complementary filter, merging gyroscope dans accelerometer data.

![AsservissementAngle](/ReadmePictures/AsservissementAngle.jpg "AsservissementAngle")

## 4. State machine

The system has 6 states:

![MachineEtats](/ReadmePictures/MachineEtats.jpg "MachineEtats")

After 5 secondes of power idle, the system is set into "security" state: throttle command is disabled and power cannot be set.

To arm again the system, pilot has to disarm it, and then he has to choose a flight mode "angle" or "accro".

## 5. CPPM reception

CPPM (Pulse Position Modulation) reception  allows to receive all channels using only one entry pin. Each rising edge correpond to the end of the previous channel impulsion, and at the beginning of the next channel impulsion.
Elapsed time between two rising edge correspond to the pulse width of a given channel.

![CPPM](/ReadmePictures/CPPM.JPG "CPPM")

In this projet, each pulse width is measured using INT0, and then stored in the correponding channel of an array.

## 6. Source code organization « CodeDroneDIY »

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
| checkIMU.cpp/h | To check IMU sensors |

## 7. Hardware configuration

### 7.1 Components list

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
| **Frame** | Diatone Q450 Quad 450 V3. Wide 450 mm frame choosen for better stability and higher autonomy. ![Chassis](/ReadmePictures/Chassis.jpg "Chassis")|

### 7.2 Connexions
TODO: add receiver in schematic
![schemaElectriqueDroneFull](/ReadmePictures/schemaElectriqueDroneFull.jpg "schemaElectriqueDroneFull")
<p align="center">Full view<p>

![SchemaElectriqueDroneZoom](/ReadmePictures/SchemaElectriqueDroneZoom.jpg "SchemaElectriqueDroneZoom")
<p align="center">Zoomed View<p>

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


### 7.3 Failsafe

For security, you must set the failsafe to cut motors power when radio link is lost.

To set the failsafe:
1. Put transmitter sticks in the configuration wanted when reception is lost
2. Bind the receiver with the transmitter

Transmitter configuration used during the « bind » operation defines the « failsafe. »

### 7.4 The benchtest

![BenchTest01](/ReadmePictures/BenchTest01.jpg "BenchTest01")

## 8.Project setup

### 8.1 Using Arduino IDE
With minor modifications, project can be build using Arduino IDE:
* rename "main.cpp" to "CodeDroneDIY.ino"
* copy all source files from "CodeDroneDIY/src" to "CodeDroneDIY"
* Launch and compile "CodeDroneDIY.ino" using Arduino IDE

### 8.2. Using PlatformIO
PlatformIO is an open source ecosystem for IoT development.

#### 8.2.1. PlatformIO installation
```sudo apt-get update
sudo apt-get install python-pip
sudo pip install --upgrade pip && sudo pip install -U platformio==3.5.2
platformio platform install atmelavr --with-package=framework-arduinoavr
platformio lib install MPU6050
pio lib install "I2Cdevlib-MPU6050"```

Optional, for code format:

```sudo apt-get install -y clang-format```

#### 8.2.2. Build project
```platformio run```

#### 8.2.3. Flash target
```platformio upload --upload-port/ttyACM0 ```

### 8.3. Using Docker
The development tool "Docker" is a container platoform: it is a stand-alone, executable package
of a piece of software that includes everything needed to run it: code, runtime, system tools,
 system libraries, settings. It isolates software from its surroundings.
* Install Docker
* Move inside docker's folder: ```cd docker```
* Build docker image: ```make image```
* Format code: ```make format-all```
* build project: ```make build-codedronediy```

## 9. FPV - First Person View
| Component      | Reference      |
| -------------- | -------------- |
| **Googgles** | Quanum DIY FPV Goggle V2 Pro |
| **Googgles battery** | 1000 mAh 3S |
| **Receiver** | Eachine RC832 Boscam FPV 5.8G 48CH Wireless AV Receiver  |
| **Receiver antenna** | DYS FPV 5.8G Antenna 4dBi Mushroom Antenna RHCP TX RX |
| **Camera** | Foxeer XAT600M HS1177 600TVL CCD 2.8MM IR Mini FPV Camera IR Blocked 5-22v |
| **Camera antenna** | Realacc 5.8G 5dBi 50W RHCP Omnidirectional 3 Leaf Clover FPV Antenna Red |
| **Video transmitter** | Upgrade Aomway Mini 5.8Ghz 200mW 32CH AV Wireless Transmitter Module |

## 10. Bibliography

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
