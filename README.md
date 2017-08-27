
# <p align="center">Contrôleur de vol de quadrirotor DIY</p>

<p align="center">
    <img src="/ReadmePictures/Drone.jpg">
</p>

---------------------------Table des matières----------------------------

**1.Objectif du projet**

**2.Calcul de l’attitude**

2.1.IMU

2.1.1.Gyroscopes

2.1.2.Accéléromètres

2.2.Fusion de données: le filtre complémentaire

**3.Stabilisation**

3.1.Mode accro (gyroscopes seuls)

3.2.Mode “ANGLE” (gyroscopes et accéléromètres)

3.3.Stabilisation en hauteur

**4.Machine à états**

**5.Réception CPPM**

**6. Organisation du code « CodeDroneDIY »**

**7.Configuration matérielle**

7.1.Liste des composants

7.2.Connections

7.3.Failsafe

**8.FPV**

**9.Annexes**

9.1.Les modes de vol

9.2.Réglages PID

9.3.Arduino UNO rev3

9.4.Génération PWM à 400Hz

**10.Bibliographie**

-------------------------------------------------------------------
## 1. Objectif du projet

L'objectif de ce projet est de concevoir un contrôleur de vol de quadricoptère maison le plus simple possible.
L'intérêt est double:
* comprendre et maitriser le fonctionnement de la stabilisation d'un drone
* avoir un système évolutif totalement personnalisable: il est possible de rajouter les capteurs que l'on veut

## 2. Calcul de l’attitude <a id="Test"></a>

L'attitude correspond aux angles du drone par rapport à l'horizontale.
Connaître l'attitude permet de stabiliser le drone, et de le remettre automatiquement à plat lorsque les manches sont ramenés au neutre.
L'attitude est calculée par fusion des données fournies par une IMU.

### 2.1 IMU

L'« Inertial Measurement Unit » est un MEMS, "Microelectromechanical system", constitué d’un gyroscope 3 axes et d’un accéléromètre 3 axes.

| Capteur      | Fonction |Avantages      | Inconvénients |
| -------------- | -------------- | -------------- | -------------- |
| Gyroscope | Mesure la vitesse angulaire en degrés par seconde autour de chaque axe. | Rapide | Dérive dans le temps |
| Accéléromètre | Mesure l’accélération rectiligne en "g" sur chaque axe. | Lent | Bruité et utilisable quand le drone n'accélère pas |

Ces 2 capteurs sont complémentaires pour calculer l'attitude : les fusionner avec un filtre permet de compenser mutuellement leurs défauts.

![IMU](/ReadmePictures/IMU.jpg "IMU")

Le projet utilise un MPU6050 qui communique avec le microcontrôleur par protocole I2C.

#### 2.1.1 Gyroscopes

L'angle avec l'horizontale est calculé par intégration des données brutes des gyroscopes.

    _pos[0] = _pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time;

#### 2.1.2 Accéléromètres

Lorsque le quadrirotor est immobile ou à vitesse constante, les accéléromètres mesurent la gravité. L'angle avec l'horizontale est calculé par trigonométrie à partir de la mesure de l’accélération de la terre.
Attention, cette mesure est faussée quand le drone accélère.

    _pos[0] = atan(accGyroRaw[1]/accGyroRaw[2]))*(180/PI);

## 2.2 Fusion de données: le filtre complémentaire

Le gyroscope dérive.
L’accéléromètre n’est pas assez rapide, il est bruité et il n’est utilisable qu’au repos lorsqu’il ne subit que l’accélération de la terre.

Le filtre complémentaire fusionne les données des gyroscopes et des accéléromètres pour supprimer leurs défauts respectifs:
- Un filtre passe-bas sur l’accéléromètre filtre le bruit et les accélérations parasites: ces données sont exploitables sur la durée, il faut éliminer les variation brusques.
- Un filtre passe-haut est appliqué sur le gyroscope car ses données sont fiables sur le court terme mais prennent de l’erreur dans le temps à cause de sa dérive.

![FiltreComplementaire](/ReadmePictures/FiltreComplementaire.jpg "FiltreComplementaire")

La valeur de la constante de temps du filtre est un compromis entre l’élimination des accélérations dues aux mouvements du quadricoptère, et la compensation de la dérive des gyroscopes :
* trop faible, les parasites des accéléromètres ne sont pas filtrés
* trop haute, la mesure dérive à cause des gyroscopes insufisemment compensés

La constante de temps du projet est de 5 sec, soit un coeff de 0.9995 pour un tour de boucle de 2.49ms, pour éliminer les vibrations et les accélérations du quadrirotor.

Attention, la constante de temps est corrélée au temps de boucle du système:

timeCste = coeff*dt/(1-coeff)
coeef = timeCste/(dt + timeCste)

    _pos[0] = HighPassFilterCoeff*(_pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time) + LowPassFilterCoeff*((atan(accGyroRaw[1]/accGyroRaw[2]))*57.2957795130823);

## 3. Stabilisation
### 3.1 mode accro (gyroscopes seuls)
C'est un asservissement en vitesse. Le pilote agit sur la vitesse de rotation du drone autour de ses axes.
Seules les données des gyroscopes sont utilisées: le filtre complémentaire n'est pas utile.

![AsservissementAccro](/ReadmePictures/AsservissementAccro.jpg "AsservissementAccro")

### 3.2 Mode “ANGLE” (gyroscopes et accéléromètres)

C'est un asservissement en position. Le pilote agit sur la position angulaire du drone pour chacun de ses axes.

Il consiste en une boucle d'asservissement en vitesse imbriquée dans une boucle d'asservissement en position.
Le retour de l'attitude est calculée par fusion des données avec le filtre complémentaire.

![AsservissementAngle](/ReadmePictures/AsservissementAngle.jpg "AsservissementAngle")

## 3.3 Stabilisation en hauteur

La mesure de la pression par le baromètre permet de déterminer la hauteur à une dizaine de centimètres près.
Ce capteur est sensible à la lumière et aux variations de pression: il faut l'isoler du souffle des hélices et des déplacements  d'air entrainés par les mouvements du drone.

Le projet utilise un MS5611 choisi pour sa précision et qui communique avec le microcontrôleur par protocole I2C.

## 4. Machine à états

Le système possède 6 états:

![MachineEtats](/ReadmePictures/MachineEtats.jpg "MachineEtats")

Au bout de 5 secondes de puissance moteur à 0%, le système passe dans l'état "sécurité": la commande des moteurs est inhibée et la puissance ne peut plus être remise.

Pour réarmer le système, il faut d'abord le désarmer, puis choisir un mode de vol "Angle" ou "Accro".

## 5. Réception CPPM

La réception CPPM (Pulse Position Modulation) permet de recevoir toutes les voies sur une seule entrée: chaque front montant correspond à la fin de l'impulsion de la voie précédente et au début de l'impulsion de la voie suivante. Le temps écoulé entre deux fronts montants correspond à la largeur d'impulsion d'une voie donnée.

![CPPM](/ReadmePictures/CPPM.JPG "CPPM")

Dans le projet, la largeur en milliseconde de chaque impulsion du train d'impulsion est mesurée à l'aide du timer0, puis stockée dans la case correspondant à la voie dans un tableau.

## 6. Organisation du code « CodeDroneDIY »

| Fichier      | Description      |
| -------------- | -------------- |
| CodeDroneDIY.ino | Contient la fonction d'initialisation et la boucle principale |
| GetPosition.cpp | Intègre et fustionne les données brutes des gyro/accéléro et retourne les angles avec l'horizontale en degrés |
| MPU6050.cpp | Librairie pour acquérir les données brutes du MPU6050 |
| PID.cpp | Correcteur proportionnel, intégral, dérivée |
| Reception.h | Acquisition par interruption 0 des signaux du récepteur au format CPPM |
| ESC.h | Gère les caractérisques d'un ESC donné: pin attribuée, largeur d'impulsion à appliquer |
| SetPWM.h | Applique les PWM aux ESC en utiliant le timer0 |
| Settings.h | Paramètres des PID et de puissance maximale |
| StateMachine.h | Machine à états |
| Time.h | Mesure du temps de boucle et du temps écoulé depuis un instant t |
| checkIMU.cpp | Vérifie le fonctionnement de l'IMU avant une utilisation |

## 7. Configuration matérielle

### 7.1 Liste des composants

| Composant      | Référence      |
| -------------- | -------------- |
| **Contrôleur de vol ** | Arduino Nano |
| **ESC** | Afro 20A-Simonk firmware 500Hz, BEC 0.5A 1060 à 1860 us de largeur d'impulsion |
| **Moteurs** | Multistar 2216-800Kv 14 Poles - 222W Current max: 20A Shaft: 3mm 2-4S|
| **Hélices** | 10x4.5 SF Props 2pc CW 2 pc CCW Rotation (Orange) |
| **Batterie** | Zippy Flightmax 3000mAh 4S |
| **Récepteur** | OrangeRx R617XL CPPM DSM2/DSMX 6 ch |
| **Contrôleur** | Arduino UNO rev3 |
| **IMU** | MPU6050 (GY-86 breakout board)|
| **Baromètre** | MS5611 (GY-86 breakout board) |
| **Boussole** | HMC5883L (GY-86 breakout board) |
| **Buzzer** | Matek lost model beeper - Built-in MCU |
| **Chassis** | Diatone Q450 Quad 450 V3. Un grand châssis de 450mm a été choisi pour privilégier la stabilité et l'autonomie. ![Chassis](/ReadmePictures/Chassis.jpg "Chassis")|

### 7.2 Connections
| Borche Arduino      | Composant      |
| -------------- | -------------- |
| PD2 | receiver |
| PD4 | ESC0 |
| PD5 | ESC1 |
| PD6 | ESC2 |
| PD7 | ESC3 |
| PC0 | potentiomètre |
| PC4 | SDA MPU6050 & MS5611 |
| PC5 | SCL MPU6050 & MS5611 |

![flightConfiguration](/ReadmePictures/flightConfiguration.jpg "flightConfiguration")


### 7.3 Failsafe

Pour la sécurité, le « failsafe » doit être programmé pour couper les gaz en cas de perte de la liaison radio.

Pour programmer le « failsafe », mettre les commandes de la télécommande dans la configuration souhaitée lors de la perte de la réception radio, et « binder » la télécommande. La configuration utilisée pendant le « bind » défini le « failsafe. »

## 8. FPV - First Person View
| Composant      | Référence      |
| -------------- | -------------- |
| **Lunettes** | Quanum DIY FPV Goggle V2 Pro |
| **Batterie lunettes** | 1000 mAh 3S |
| **Récepteur** | Eachine RC832 Boscam FPV 5.8G 48CH Wireless AV Receiver  |
| **Antenne récepteur** | DYS FPV 5.8G Antenna 4dBi Mushroom Antenna RHCP TX RX |
| **Caméra** | Foxeer XAT600M HS1177 600TVL CCD 2.8MM IR Mini FPV Camera IR Blocked 5-22v |
| **Antenne caméra** | Realacc 5.8G 5dBi 50W RHCP Omnidirectional 3 Leaf Clover FPV Antenna Red |
| **Emetteur** | Upgrade Aomway Mini 5.8Ghz 200mW 32CH AV Wireless Transmitter Module |

## 9. Annexes

### 9.1 Les modes de vol

| Mode      | Gyro      | Accé      | Baro      | Bouss      | GPS      | Description      |
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

### 9.2 Réglages PID

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

Microcontrôleur ATmega328
Architecture 8 bits RISC
16Mhz => T = 0.0625us
1MIPS par MHz
Dimensions : 68.6 mm x 53.4 mm
Poids : 25 g

### 9.4 Génération PWM à 400Hz

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

## 10. Bibliographie

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

* Fusion de données

http://www.mouser.fr/applications/sensor_solutions_mems/

http://www.pieter-jan.com/node/11

* Programme python

Andy BAKER

http://pythonfiddle.com/andy-baker-quadcopter/

* Vibrations

http://ardupilot.org/plane/docs/common-vibration-damping.html
