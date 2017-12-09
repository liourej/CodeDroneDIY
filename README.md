
# <p align="center">DIY flight controller from scratch</p>

<p align="center">
    <img src="/ReadmePictures/Drone.jpg">
    <img src="/ReadmePictures/v1_v2_boards.jpg">
    Pictures: First hardware version using Arduino Uno board and second hardware version using Arduino Nano board
</p>

---------------------------Table des matières----------------------------

**1.Project purpose**

**2.Attitude computation**

2.1.IMU

2.1.1.Gyroscopes

2.1.2.Accelerometers

2.2.Data merging: the complementary filter

**3.Stabilization**

3.1. Accro mode (gyroscopes only)

3.2.Angle mode (gyroscopes and accelerometers)

3.3.Height stabilization

**4.State machine**

**5.CPPM Reception**

**6. Source code organization « CodeDroneDIY »**

**7.Hardware configuration**

7.1.Components list

7.2.Connexions

7.3.Failsafe

7.4.The benchtest

**8.First Person View (FPV)**

**9.Appendix**

9.1.Flight modes

9.2.PID tuning

9.3.Arduino UNO rev3

9.4.PWM generation at 400Hz

**10.Bibliography**

-------------------------------------------------------------------
## Warning
![Danger.jpg](/ReadmePictures/Danger.jpg "Danger")

Before starting this project, you must know that spinning propellers are very dangerous for your eyes and the persons around.
When testing using the benchtest indoor, you must wear large protective glasses, and set up a power limit.
For outside tests, choose a very large area, with nobody around.

![Protection_glasses.jpg](/ReadmePictures/Protection_glasses.jpg "Protection_glasses")

## 1. Project purpose

The aim of this project is to develop a quadrirotor flight controller from scratch, using an Arduino and inertial sensors.
There is two benefits:
* to understand UAV flight stabilization
* to have our own system, with no limits for customization: you can add all the sensors you want.


## 2. Attitude computation <a id="Test"></a>

The UAV attitude correspond to the UAV angles from the horizontal: roll, pitch , yaw.
UAV attitude is an entry for automatic stabilization: when UAV receive no command from user, it automatically goes back to horizontal.

Attitude is computed from IMU data merging.

### 2.1 IMU

« Inertial Measurement Unit » is a MEMS, "Microelectromechanical system",
composed by a 3 axis gyroscope and a 3 axis accelerometer.

| Sensor      | Function |Pros      | Cons |
| -------------- | -------------- | -------------- | -------------- |
| Gyroscope | Measure angular speed around each axis in degrees by second | Fast | Drift along time |
| Accelerometer | Measure linear acceleration on each axis in "g" | Slow |  Noizy and not usable when UAV is moving |

These 2 sensors are complementary to compute attitude: merging their data compensate their defaults

![IMU](/ReadmePictures/IMU.jpg "IMU")

This projet use an MPU6050 IMU sensor which communicates with the microcontroler using I2C.

#### 2.1.1 Gyroscopes

Angles with the horizon are computed using raw gyroscope data.

    _pos[0] = _pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time;

#### 2.1.2 Accelerometers

When UAV is not moving, or it is moving at a constant speed, accelerometers measure gravity.
Angle between UAV and the horizon is computed by trigonometry.
Be carefull, this measure is faulty when UAV accelerates.

    _pos[0] = atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI);

## 2.2 Data merging: the complementary filter

The gyroscope sensor drift.
Accelerometers are not fast enougth, they are noizy, and they are usable only when the UAV is not moving, and when UAV receive only earth acceleration.

The complementary filter is a very simple filter which allows to merge data from gyroscopes and accelerometers, and mask their respective errors:

- A low-pass filter is applied on accelerometer data to filter the noize and the unwanted accelerations: these data are usefull on a long time period, fast changes have to be eliminated.
- A high-pass filter is applied on the gyrosope data: these data are usefull on short time period, but they drift and accumulate errors on long time periods.

![FiltreComplementaire](/ReadmePictures/FiltreComplementaire.jpg "FiltreComplementaire")

Complementary filter time constant is a compromise between UAV acceleration filtering, and gyroscopes drift:
* Too low, accelerometer noize are not eliminated
* Too high, gyroscopes drift is not compensated

Time constant in this project is set to 5 seconds. It implies a coefficient of 0.9995 for a loop time of 2.49 ms.

Be carefull, the time constant depends on the control loop time:

timeCste = coeff*dt/(1-coeff)
coeef = timeCste/(dt + timeCste)

    _pos[0] = HighPassFilterCoeff*(_pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time) + LowPassFilterCoeff*((atan(accGyroRaw[1]/accGyroRaw[2]))*57.2957795130823);

## 3. stabilization
### 3.1 Accro mode (gyroscopes only)
It is a speed control loop. The pilot controls rotation speeds around each axis.
Only gryroscopics data are used: there is no need for a complementary filter.

![AsservissementAccro](/ReadmePictures/AsservissementAccro.jpg "AsservissementAccro")

### 3.2 Angle mode (gyroscopes and accelerometers)

It is a position control loop. the pilot controls each attitude angle.

It consists in a speed control loop inside a position control loop.

Attitude is computed using a complementary filter, merging gyroscope dans accelerometer data.

![AsservissementAngle](/ReadmePictures/AsservissementAngle.jpg "AsservissementAngle")

## 3.3 Height stabilization

Measuring pressure using a barometer sensor allows to compute the UAV height.
This sensor is light and pressure sensitive: it needs to be isolated from propeller blow and air circulation due to UAV moves.

This project uses a MS5611 choosen for its accuracy. It uses I2C communication with the microcontroller.

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
| CodeDroneDIY.ino | Initialization fucntion and main loop |
| GetPosition.cpp | Gyro and accelero raw data intergration and merging, attitude angles computation |
| MPU6050.cpp | Library for MPU6050 raw data acquisition |
| PID.cpp | Proportionnal, integral, derivative loop correction |
| Reception.h | Receiver CPPM signal acquisition using INT0 |
| ESC.h | Manage an ESC: pin, PWM to set |
| SetPWM.h | Set PWM to ESC using timer 0 |
| Settings.h | All parameters: max power, PID settings... |
| StateMachine.h | State machine |
| Time.h | To measure loop time and elapsed time |
| checkIMU.cpp | IMU checking before using |

## 7. Hardware configuration

### 7.1 Components list

| Component      | Reference      |
| -------------- | -------------- |
| **Microcontroller board** | Arduino Nano |
| **ESC** | Afro 20A-Simonk firmware 500Hz, BEC 0.5A 1060 à 1860 us de largeur d'impulsion |
| **Motors** | Multistar 2216-800Kv 14 Poles - 222W Current max: 20A Shaft: 3mm 2-4S|
| **Propellers** | 10x4.5 SF Props 2pc CW 2 pc CCW Rotation (Orange) |
| **Battery** | Zippy Flightmax 3000mAh 4S |
| **Receiver** | OrangeRx R617XL CPPM DSM2/DSMX 6 ch |
| **IMU** | MPU6050 (GY-86 breakout board)|
| **Barometer** | MS5611 (GY-86 breakout board) |
| **Compass** | HMC5883L (GY-86 breakout board) |
| **Buzzer** | Matek lost model beeper - Built-in MCU |
| **Frame** | Diatone Q450 Quad 450 V3. Wide 450 mm frame choosen for better stability and higher autonomy. ![Chassis](/ReadmePictures/Chassis.jpg "Chassis")|

### 7.2 Connexions
| Arduino pin      | Component      |
| -------------- | -------------- |
| PD2 | receiver |
| PD4 | ESC0 |
| PD5 | ESC1 |
| PD6 | ESC2 |
| PD7 | ESC3 |
| PC0 | potentiometer |
| PC4 | SDA MPU6050 & MS5611 |
| PC5 | SCL MPU6050 & MS5611 |

![flightConfiguration](/ReadmePictures/flightConfiguration.jpg "flightConfiguration")


### 7.3 Failsafe

For security, you must set the failsafe to cut motors power when radio link is lost.

To set the failsafe:
1. Put transmitter commands in the configuration wanted when reception is lost
2. Bind the receiver with the transmitter

Trasnmitter configuration used during the « bind » operation defines the « failsafe. »

### 7.4 The benchtest

![BenchTest01](/ReadmePictures/BenchTest01.jpg "BenchTest01")

## 8. FPV - First Person View
| Component      | Reference      |
| -------------- | -------------- |
| **Googgles** | Quanum DIY FPV Goggle V2 Pro |
| **Googgles battery** | 1000 mAh 3S |
| **Receiver** | Eachine RC832 Boscam FPV 5.8G 48CH Wireless AV Receiver  |
| **Receiver antenna** | DYS FPV 5.8G Antenna 4dBi Mushroom Antenna RHCP TX RX |
| **Camera** | Foxeer XAT600M HS1177 600TVL CCD 2.8MM IR Mini FPV Camera IR Blocked 5-22v |
| **Camera antenna** | Realacc 5.8G 5dBi 50W RHCP Omnidirectional 3 Leaf Clover FPV Antenna Red |
| **Video transmitter** | Upgrade Aomway Mini 5.8Ghz 200mW 32CH AV Wireless Transmitter Module |

## 9. Appendix

### 9.1 Flight modes

| Mode      | Gyro      | Acce      | Baro      | Compass      | GPS      | Description      |
| -------------- | -------------- | -------------- | -------------- | -------------- | -------------- | -------------- |
| **ACRO** | X |||||Un mode généralement par défaut et son vol plus « acrobatique » (le quadrirotor ne peut faire de la mise à niveau automatique)|
| **ANGLE** | X |X||||Mode stable ; va essayer de maintenir le niveau du modèle par rapport au sol (mais pas à une position fixe).|
| **HORIZON** | X |||||Combine l’effet stable avec des commandes et des acrobaties RC lentes et avec des commandes RC rapides.|
| **BARO (Maintien de l’altitude)** | X | X | X |||Le baromètre est utilisé afin de conserver une certaine hauteur (fixée) lorsqu’aucune autre commande n’est reçue.|
| **MAG (Tenue du cap)** | X | X | | X | | Mode verrouillage de cap (direction à la boussole), pour essayer de maintenir son orientation en lacet. |
| **HEADFREE (CareFree/ orientation indépendante du déplacement)** | X | X | | X | | Maintient l’orientation (lacet) du quadrirotor et se déplace toujours dans la même direction 2D pour le même mouvement du manche en ROULIS/TANGAGE.|
| **GPS/ Retour à la base** | | X | | X | X | Utilise automatiquement une boussole et un GPS pour rentrer à la base, au point de départ GPS. |
| **GPS/ Points de passage** | | X | | X | X | Suit automatiquement les points de cheminement GPS pré-configurés de manière autonome. |
| **GPS/ Maintien de position** | | X | | X | X | Maintient la position actuelle en utilisant le GPS et le baromètre (si disponible). |

### 9.2 PID tuning

**Le P**

C’est le P qui va résoudre les problèmes de vibrations. Le I quant à lui, joue sur l’inertie de la machine et sur sa réactivité.
Montez le P jusqu’à obtenir une machine qui vibre / oscille. Baissez le I également, ça aide.
Descendez le P peu à peu jusqu’à ce que les vibrations disparaissent totalement, même à fond de gaz !

**Le I**

Il agît sur la dérive du multi est est lié au P. En acro, voltige ou FPV, il sera plus bas que pour une machine dédiée à la vidéo. Dans la vidéo en Français, vous apprendrez comment le régler à l’aide d’un truc connu : placer un poids sur un bras
Montez le I jusqu’à obtenir des oscillations faibles en montée et ou en descente. Vous devriez en avoir aussi à fond de gaz.
Si vous avez des oscillations en descente : montez le I
Si vous avez des oscillations en montée : baissez le I
Trouvez une valeur qui vous débarrasse des deux
Si des vibrations réapparaissent, c’est normal. Retouchez le P. Baissez le légèrement, c’est selon.

**Le D**

Ce paramètre est le plus  » personnel  » des trois. Il influence la réactivité de la machine.
Faites de grands mouvements de gauche à droite ou d’avant en arrière pour observer les réactions de la machine. Appréciez et réglez selon vos préférences.
Des vibrations peuvent revenir : corrigez le P.

Le Yaw
Si votre machine continue à dériver après un ordre ou si elle dérive seule sur l’axe du lacet, changez la valeur. Elle est très souvent sur 8.5 et est correcte ainsi.
tpa breakpoint
Ce paramètre joue sur le ratio des PID. En effet, la tension et le niveau de gaz sont des variables qui agissent sur le comportement. Le TPA va faire varier vos PID selon ces facteurs.Si vous n’en mettez pas, il se peut que vous ayez des vibrations lorsque vous êtes à fond de gaz avec une lipo chargée à bloc. Pour être précis, les TPA ( Throtlle PID Attenuation ) jouent sur le P. ( Merci XKin Ai pour la précision )

### 9.3 Arduino UNO rev3

ATmega328 microcontroller
8 bits RISC architecture
16Mhz => T = 0.0625us
1MIPS by MHz
Dimensions : 68.6 mm x 53.4 mm
Weight : 25 g

### 9.4 PWM generation at 400Hz

1. Classer les ESC par ordre croissant de largeur d’impulsion.
2. Utiliser le « Timer1 ».
3. Mettre toutes les sorties au niveau haut en une seule fois à l’aide de la commande « PORTB=0b00001111 ».
4. Mettre à au niveau bas les sortie au fur et à mesure.
>* For every 2.5 ms period the first 1.5 is unused and can be used for main program logic.
>* From 1.5 to 2.5 ms every interrupt will disturb timing, but without communications most servo controllers are uninteresting.
>* We can use a loop to wait for the next "set servo low" time or we can use a timer interrupt
>* Best precision is if all the servo output pins are on the same AVR port and we directly write to the PORTx register. Then servos with equal timing can be handled with one port write.
>* The situation with the most difficult timing is when two servos are separated by 2-8 uS, less than an interrupt period.
>* After every 2.5 ms servo period we wait 7*2.5 ms for the next servo control period. We can use these periods to address  other groups of servos if we use demuxes to distribute the servo pulses to the groups.

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

* Quadrirotor

https://www.mondrone.net/fabriquer-quadricoptere-la-propulsion/

* Traitement de données

https://ericjformanteaching.wordpress.com/2013/10/08/smoothing-sensor-input/

* Data merging

http://www.mouser.fr/applications/sensor_solutions_mems/

http://www.pieter-jan.com/node/11

* Programme python

Andy BAKER

http://pythonfiddle.com/andy-baker-quadcopter/

* Vibrations

http://ardupilot.org/plane/docs/common-vibration-damping.html
