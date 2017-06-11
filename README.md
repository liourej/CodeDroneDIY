# CodeDroneDIY
Very simple quadricopter flight controller from scratch.

27-May-2017 Plus config working in accro mode (PID only on speed, no autonomous flight)

Setup:  
pin PB0 connected to ESC0  
pin PB1 connected to ESC1   
pin PB2 connected to ESC2  
pin PB3 connected to ESC3  
pin PD2 connected to CPPm receiver  
pin PC2 connected to potentiometer  
pin PC4 connected to SDA MPU6050  
pin PC5 connected to SCL MPU6050  

ESC0 spin CCW 
ESC1 spin CW  
ESC2 spin CCW 
ESC3 spin CW  

Config working (tested in flight):

MAX_POWER:	1860
FLYING_MODE_ACCRO
/********* PID settings *********/
G: 0.01	Kp: 192.00	Kd: 0.00	Ki: 0.00
G: 0.01	Kp: 192.00	Kd: 0.00	Ki: 0.00
G: 0.01	Kp: 150.00	Kd: 0.00	Ki: 0.00
Yaw PID activation:	1
Mixing:	0.50
/********* Receiver settings *********/
Aile: 1488	Elev: 1492	Throt: 1092	Rudd: 1496
Switch1: 1096	Switch2: 1492
/********* MPU 6050 Configuration *********/
Gyroscope range:	2	Accelerometer range:	2

