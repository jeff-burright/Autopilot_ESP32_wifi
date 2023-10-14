

/* Jeff's Arduino Autopilot Version ESP32_Wheel1.5. 
Simple compass-only marine autopilot that functions like a Raymarine ST2000 or similar.
Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver, red button to toggle steering on/off, LSM303 9DOF compass, and 16x2 LCD. Rudder feedback potentiometer optional.
If using Wifi, pilot is operational with only ESP32, IMU compass, motor controller, and motor. 
WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing browser to 192.168.4.1.

Version Notes: 
--first attempt at ESP-NOW connection for TTGO smart watch


Issues: None at present. Code could be slimmer
Experimental aspects: 
wiper motor noise reduction via PWM frequency modulation seems to work. 

To do:
double check PID settings

Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.
Use this code at your own risk. 
*/

 #define Arduino 0   
 #define Teensy 1
 #define Board Arduino//  0 = Arduino or  1 = Teensy

  #include <Keypad.h>
 // #include <LiquidCrystal.h>
  #include <LiquidCrystal_I2C.h>
  #include <Wire.h>
#include <Bounce2.h> // use for red button toggle pilot on/off
  #include <L3G.h> // IMU compass
#include <BTS7960.h> // IBT-2 motor controller library
#include <IRremote.h> // infrared remote control
#include <IRremote.hpp>
 #include <LSM303.h> // IMU compass
 #include <WiFi.h> 
#include <AsyncTCP.h> 
#include <ESPAsyncWebServer.h> 
#include <AsyncElegantOTA.h> // over the air update of compiled binary via 192.168.4.1/update
#include <ArduinoJson.h> // Include ArduinoJson Library
//#include <SPIFFS.h> // for putting html and css in the flash
#include <esp_now.h>
#include <esp_wifi.h>


// REPLACE WITH THE MAC Address of your receiver 
//uint8_t peerAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x64};

// Set your new MAC Address for THIS DEVICE
//uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Replace with your WIFI network credentials. Point control device browser to 192.168.4.1
const char* ssid = "PlotDevice";
const char* password = "";

String jsonString; // place to store heading and rudder data for web socket

/******       USER INPUT       ********/

#define Compass 0 //  0 for Pololu, 1 for BNO055
#define IMU 2 // Used to select Pololu IMUs Versions and calibration set. 
    //Allowed values: 2 IMU9 V2; 93 (jacksIMU9V3); 103 (jacks IMU10V3; 51 (Jacks IMU9V5 #1); 52 (Jacks IMU9 V5 #2)
    //determines which set of calibration data is used these extra versions were added to code 7/11/17 J14.4
#define GPS_Used 0 // 1 to include GPS code, 0 to exclude it
#define Motor_Controller 3  // 1 for Pololu Qik dual controller, 2 for Pololu Trex dual controller, 3 for generic Controller
#define RUDDER_MODE 0 // 0 uses rudder position, 1 does not
#define RF24_Attached 0 // 0 if RF 24 radio modules are not attached, 1 if they are used
#define Wind_Input 0 // 1 to use NMEA wind data. 0 to not use wind data
#define RUDDER_OFFSET 1 // 1 uses rudder offset, 0 does not
#define BEARINGRATE_OFFSET 1 // 1 to use 0 to not use

 
 // Rudder direction for LCD. Value is set in PID tab. 
 String rudd_dir;

 // PID function
 
 float K_overall = 2;
 float K_heading = .4;
 float K_differential = 2;
 float K_integral = .0005;
 float PID_Ks[4] = {K_overall, K_heading, K_differential, K_integral};
 //float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
// float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output (PID Mode 1);
 #define PID_MODE 3 // See description PID tab.

 boolean just_started = 1; // to do a second setup so get a better gyro startup
 //float Kxte[3] = {0, 0, 0}; // used for XTE PID, use this to zero out XTE tracking
//float Kxte[3] = {.2, 0, 0}; // {.2, 4, .0004} baseline; {.05, .5, .0005}last used;  0 is proportional, 1 is differential, 2 is integral error, see GPS_Steer() in PID
                        // .36 will give 45 deg correction at 120 ft XTE to emulate my Garmin GPSMAP 740s see PID tab, voidActual_Gps_Steering()
 float K_course_avg = .999; //used to smooth gps course in PID/ void Actual_GPS_Steering().999 will smooth over 1000 points
 float Maximum_Rudder = 40; // rudder in degrees
 // User set motorspeedMIN around lines 359, 360, 371 for your controller type and rudder steering motor Use crtl-F to find motorspeedMIN 
 float Tack_Angle = 100;  // angle throug which boat will tack when tack L or R pressed (keys 4 and 6 in TACK mode(3)
 int Tack_rudder_MAX = 32;// limits rudder so it doesn't slow boat too much,  need to tune
 float Tack_rudder_speed = .5; // rudder speed during tack , value * full speed, will use min of tack speed and regular speed, user adjust
 float Rudder_Offset = 0; // see notes 10.21.16
 float bearingrate_Offset = 0;
 float MagVar_default = 14;// 18.4 Seattle   User should keep up to date for loaction.  Pgm will use GPS value if available + east, - west

 int print_level_max = 1;  //  0 to 4, used to set how much serial monitor detail to print
 // 0 = none, 1=PID Output, 2 Adds parsed GPS data, 3 adds raw GPS input, 4 adds checksum results
 int print_time =5;  // print interval in sec
 boolean Print_ETdata = 0; //prints GPS incoming data turn this off to see actual loop time
 boolean Print_ETtime = 0;  // prints Easy Transfer loop time 1 on 0 off
 boolean Print_heading  = 0 ; // diagnostic serial print heading, interval set in A_P_loop
 boolean Print_LCD_IMU9 = 0;  //prints Head, Pitch, Roll, Yaw on LCD conflicts with other LCD Prints
 boolean Print_LCD_AP = 1; // prints main A/P LCD output data and Menus, only do one of these at a time
 boolean Print_Gyro = 0; //  Prints LCD and Serial scaled X(roll), Y(pitch), Z(Yaw) deg/sec
 boolean Print_PID = 0;
 boolean Print_UTC = 0;
 boolean print_Nav_Data = 0; // Print_1 Tab
 boolean Print_Motor_Commands = 0;  // prints rudder commands in PID tab
 boolean Print_Anticpate_Turn = 0;  // prints data from void Actual_GPS_Steering to evaluate Anticipate turn function
 int print_level=print_level_max;
//  print modes for MinIMU9
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1
#define PRINT_DATA 0 // 1 to print serial data or run python display
#define PRINT_EULER 0  //Will print the Euler angles Roll, Pitch and Yaw, needed for python display
//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data


// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//LCD screen (16x2 LCD with I2C interface chip)
LiquidCrystal_I2C lcd(0x27,16,2); //Addr: 0x3F, 20 chars & 4 lines, for serial LCD
long lcdtimer=0;   //LCD Print timer

// Red on/off button settings
boolean toggle = false;     
const int BUTTON_PIN = 4;    // changed from #define
Bounce debouncer = Bounce(); // Instantiate a Bounce object

// IR Remote pin
const int IR_RECEIVE_PIN = 27;

 // Rudder position sensor
const int Rudder_Pin = 34;  

float MagVar; //Magnetic Variation E is plus, W is minus 
 float Magnetic_Variation; 
 float heading_error = 0;
 float differential_error = 0;
 float integral_error = 0; 
 float deadband = 2.0;  // steering deadband
 float rudder_position = 0;
 float rudder_command = 0;
 float rudder_error;
 int motorspeed;  // sets rudder motor speed in IBT-2 controller can be constant or variable
 int rudder_MAX; //allows rudder to be changed for various maneuvers like TACK
 boolean TACK_ON = false;  // goes on for a tack and off when within 10 deg final course
// float rudder_total_time;
 float bearingrate=0;
 float bearingrate_smoothed;
 float bearingrate2;
 unsigned long bearingrate2_time;
 float heading_old;
 float delta_heading;
 long delta_compass_time; // computed in compass tab, used in PID for integral error
 float PID_output = 0; 
// float GPS_PID = 0;
 boolean GPS_Steering = false;
 boolean Steering_Mode = 0;
 String Mode = "OFF";
 String Previous_Mode;
 boolean Steering = false;
 boolean sw1_turned_on = false;
 boolean sw1 = false;
 boolean sw2 = false;
 boolean sw1Status = 1;
 boolean DODGE_MODE = false;
 int Screen=0;
 boolean rudder_on;
 boolean rudder_was_off;
 unsigned long rudder_time_old;
 boolean lcdlight = 0; 

// ------------------ESP-NOW Parameters------------------------//

// Define variables to store outgoing readings
float incomingHeading;
float incomingHTS;
float APState;
float HTSout;

boolean requestHTS = 0; // need to setup on pilot code

// Define command payload sent from the watch
//const char *APbutton; 
uint8_t incomingCommand;
//const uint8_t* incomingData

// Variable to store if sending data was successful
String success;
 
//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int doit;
    float HTS;
    float STATE;
} struct_message;

// Create a struct_message called APCommand to hold command to send
struct_message APCommand;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *, esp_now_send_status_t);
void OnDataRecv(const uint8_t *, const uint8_t *, int);


 // ----------------- IBT-2 motor controller ----------------------

//assign IBT-2 Pins
//const uint8_t EN = 25;  // assigned but not used. EN pins on controller are connected to 5v voltage to controller to make them always active.
const uint8_t L_PWM = 32;
const uint8_t R_PWM = 33;


   int motorspeedMIN = 100; // was 555. this value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 255; // was 3200. jeff changed consistent with max PWM for IBT-2.
 
 // ------------------------END IBT-2 MOTOR CONTROLLER SETTINGS ---------------------


 
/***********  COMPASS  SET UP  **************************************/
float heading;
float heading_to_steer=0; //  see PID
float MAG_Heading_Degrees; //LCD_compass tab

//float system_cal; float gyro_cal;  float accel_cal; float mag_cal;
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
unsigned int counter=0;
unsigned int counter2=0;
unsigned long counter3=0; // used for 10 minute print interval in A_P_Loop
float roll;
float pitch;
float yaw;

#if Compass == 0
// setup data for MinIMU9 from Tab minIMU9AHRS of Pololu software
int SENSOR_SIGN[9] = {1,-1,-1, -1,1,1, 1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
// L3G4200D gyro: 2000 dps full scale  see tab I2C line 60 to set Full scale value
// 70 mdps/digit; 1 dps = 0.07 use .07 for FS 2000dps, use .00875 for FS 245 dps. see I2C about line 64 to set full scale
#define Gyro_Gain_X .00875 //X axis Gyro gain .07 for FS 2000 DPS, .00875 for full scale of 245 degrees per second
#define Gyro_Gain_Y .00875 //Y axis Gyro gain
#define Gyro_Gain_Z .00875 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board

// this is data for LSM303,
#if IMU == 2 // Jack's version 2 IMU calibration 

  #define M_X_MIN -663   
  #define M_Y_MIN -683
  #define M_Z_MIN -611   
  #define M_X_MAX 453
  #define M_Y_MAX 427
  #define M_Z_MAX 460 
#endif

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z; 
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
// end IMU-9 data
#endif // end if compass == 0
/************************************/

// -------------------------------WEB INTERFACE FOR WIFI CONTROL AT 192.168.4.1 --------------------


const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>
<title>Plot Device</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">

<style>

html {
  font-family: Arial, Helvetica, sans-serif;
  text-align: center;
}
h1 {
  font-size: 1.8rem;
  color: white;
}
h2{
  font-size: 1.5rem;
  font-weight: bold;
  color: #BABABA;
line-height: .1;
}

h3{
  font-size: 1rem;
  font-weight: bold;
  color: #828282;
line-height: .8
}

h4{
  font-size: .05rem;
  color:#150501;
  line-height: .1
}
.topnav {
  overflow: hidden;
  background-color: #2a0a03 ;
}
body {
  margin: 0;
background-color:#2a0a03;
}
.content {
  padding: 30px;
  max-width: 600px;
  margin: 0 auto;
}
.card {
  background-color: #150501;;
  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
  padding-top:10px;
  padding-bottom:20px;
color: #fff;
}
.button {
  padding: 10px 50px;
  font-size: 24px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 5px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button:hover {background-color: #0f8b8d}*/
 .button:active {
   background-color: #458a8a;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }

.button2 {
  padding: 5px 20px;
  font-size: 36px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 10px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button2:hover {background-color: #0f8b8d}*/
 .button2:active {
   background-color: #458a8a;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }


.button3 {
  padding: 10px 20px;
  font-size: 18px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 10px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button3:hover {background-color: #0f8b8d}*/
 .button3:active {
   background-color: #e4e000;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }


.button4 {
  padding: 5px 30px;
  font-size: 16px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 15px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button4:hover {background-color: #0f8b8d}*/
 .button4:active {
   background-color: #e4e000;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }

 .state {
   font-size: 1.5rem;
   color:#DFFFFF;
   font-weight: bold;
 }

.buttrow>.button2:not(:last-child) {
margin-right: 50px;
margin-bottom: 5px;
}




.gauge {
  position: relative;
  margin: auto;
  background: var(--gauge-bg);
  border: 0.05em solid #150501;
  border-radius: 50%;
  min-width: 250px;
  min-height: 125px;
  font-weight: bold;
  font-size: 1.5rem;
}

.gauge .ticks {
  position: relative;
  width: 100%;
  height: 100%;
  top: 0%;
  left: 0%;
}

.gauge .ticks .min {
  background: black;
  position: relative;
  left: -16%;
  top: -24%;
  width: 100%;
  height: 1%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(225deg);
}

.gauge .ticks .mid {
  background: black;
  position: absolute;
  left: 0%;
  top: -50%;
  width: 100%;
  height: 2%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(-90deg);
}

.gauge .ticks .max {
  background: black;
  position: relative;
  left: 16%;
  top: -24%;
  width: 100%;
  height: 1%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(-45deg);
}



.gauge .tick-circle {
  position: absolute;
  color: #BABABA;
  top: 15%;
  left: 25%;
  width: calc(50% - 0.1em);
  height: calc(70% - 0.1em);
  border-left: 0.1em solid transparent;
  border-top: 0.1em solid transparent;
  border-right: 0.1em solid transparent;
  border-bottom: 0.1em solid;
  border-radius: 50%;
}


:root{
  --gaugevalue: 0;
  --gaugedisplayvalue: 0;
}

.gauge .needle {
  /* Gauge value range 0-100 */
  transform: rotate(calc(45deg * calc(var(--gaugevalue, 0deg) / 45) - 90deg));
  background-color: black;
  position: absolute;
  left: 0%;
  top: 32%;
  width: 100%;
  height: 8%;
  margin-bottom: -4%;
  background: linear-gradient(90deg, rgba(2, 0, 36, 0) 0%, rgba(0, 0, 0, 0) 27%, rgba(15, 139, 141, 1) 27%, rgba(15, 139, 141, 1) 40%, rgba(0, 0, 0, 0) 50%);
}

.gauge .needle .needle-head {
  position: absolute;
  top: 10%;
  left: 26%;
  width: 2.7%;
  height: 80%;
  background-color: #0f8b8d;
  transform: rotate(-45deg);
}

.gauge .labels {
  position: absolute;
  width: 100%;
  height: 100%;
}

.gauge .labels .value-label {
color: #BABABA;
  position: relative;
  top: 10%;
  left: 50%;
  transform: translateX(-50%);
}

.gauge .labels .value-label::after {
  counter-reset: --gaugevalue var(--gaugedisplayvalue);
  content: counter(--gaugevalue);
}

.guide-x, .guide-y {
  background-color: orange;
  visibility: visible;
  position: absolute;
  left: 50%;
  top: 0;
  width: 1px;
  height: 100%;
}

.guide-y {
  left: 0;
  top: 50%;
  width: 100%;
  height: 1px;
}


</style>



</head>

<body>
  <div class="topnav">
    <h1>Plot Device</h1>
  </div>
  <div class="content">
    <div class="card">

<p></p>
      <p><h3>Status</h3></p>
<p><button id="button" class="button"><class="state"><span id="state">$STATE$</span></button></p>


 <div id="ruddGauge" class="gauge" style="
     --gauge-bg: #150501;

">

  <div class="ticks">
    <div class="tithe" style="--gauge-tithe-tick:1;"></div>
    <div class="tithe" style="--gauge-tithe-tick:2;"></div>
    <div class="tithe" style="--gauge-tithe-tick:3;"></div>
    <div class="tithe" style="--gauge-tithe-tick:4;"></div>
    <div class="tithe" style="--gauge-tithe-tick:6;"></div>
    <div class="tithe" style="--gauge-tithe-tick:7;"></div>
    <div class="tithe" style="--gauge-tithe-tick:8;"></div>
    <div class="tithe" style="--gauge-tithe-tick:9;"></div>
    <div class="min"></div>
    <div class="mid"></div>
    <div class="max"></div>
  </div>
  <div class="tick-circle"></div>
  <div class="needle">
    <div class="needle-head"></div>
  </div>
  <div class="labels">
    <div class="value-label"></div>
  </div>
</div>


      <h3>Heading</h3>
      <h2><p><span id="heading">$HEAD$</p></span></h2>

      <h3>Course to Steer</h3>
      <h2><p><span id="heading_to_steer">$HTS$</span></p></h2>

 <p>     
<div class="buttrow"> 
<button id="sub10" class="button2">-10</button> 
<button id="add10" class="button2">+10</button>
</div>
</p>
<p>
<div class="buttrow"> 
<button id="sub1" class="button2">-1</button> 
<button id="add1" class="button2">+1</button>
</div>
</p>
<p>
<div class="buttrow"> 
<button id="sub90" class="button2">-90</button>   
<button id="add90" class="button2">+90</button>
</div>
</p>

<div class="buttrow"> 
<button id="dodgeleft" class="button2"><-</button>   
<button id="dodgeright" class="button2">-></button>
</div>
</p>

<p><button id="LCDbacklight" class="button3">LCD light</button></p>

    </div>
<p><h3><i>Wherever you go, there you are</i></h3></p>
  </div>
<p></p>

<div class="card">
<h2>Pilot Settings</h2>
<h3><p><i>- resets to default on start -</i></p></h3>


<div class="buttrow"> 
<p><span id="R_deadband">Rudder Deadband = $DEADBAND$  <p></p>
<button id="Rdown" class="button4">-</button>   <button id="Rup" class="button4">+</button></span></p>
</div>
<p></p>

<br>
<h3><p>PID Values</p></h3>


<div class="buttrow"> 
<p><span id="K_overall">Gain = $KOVERALL$ <p></p>
<button id="Gdown" class="button4">-</button>   <button id="Gup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_heading">P = $KHEAD$ <p></p>
<button id="Pdown" class="button4">-</button>   <button id="Pup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_integral">I = $KINTEGRAL$ <p></p>
<button id="Idown" class="button4">-</button>   <button id="Iup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_diff">D = $KDIFF$ <p></p>
<button id="Ddown" class="button4">-</button>   <button id="Dup" class="button4">+</button></span></p>
</div>

<br><br>
<div class="buttrow"> 
<p><span id="MAGVAR">Magnetic Variation = $MAGVAR$ deg.<p></p>
<button id="magvardown" class="button4">-</button>   <button id="magvarup" class="button4">+</button></span></p>
</div>

</div>

<h4>Rudder</h4>
<h4><p><span id="rudder_position">$RUDD$</span></p></h4>

<script>


setInterval(function() {
    // Call a function repetatively with 5 Second interval
    getData();
  }, 1000); //1 Seconds update rate
  
  function getData() {
   websocket.send('readheading');
  }
  
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    window.addEventListener('load', onLoad);
    function initWebSocket() {
      console.log('Trying to open a WebSocket connection...');
      websocket = new WebSocket(gateway);
      websocket.onopen    = onOpen;
      websocket.onclose   = onClose;
      websocket.onmessage = onMessage; // <-- add this line
    }
    function onOpen(event) {
      console.log('Connection opened');
   
    }
    function onClose(event) {
      console.log('Connection closed');
      setTimeout(initWebSocket, 2000);
    }
  
    function onMessage(event) {
      var obj = JSON.parse(event.data);
      document.getElementById('heading').innerHTML = obj.heading;
      document.getElementById('rudder_position').innerHTML = obj.rudder_position;


     //document.getElementById('gaugevalue').innerHTML = obj.rudder_position;
      //console.log(obj);
      
     // var jgaugevalue = obj.rudder_position;
     // gaugevalue = jgaugevalue;
     // gaugedisplayvalue = jgaugevalue;

      let root = document.documentElement;
      root.style.setProperty('--gaugevalue', obj.rudder_position);
      root.style.setProperty('--gaugedisplayvalue', obj.rudder_position);
      
};
    
    
  
  
    function onLoad(event) {
      initWebSocket();
      initButton();
  initsub10();
  initadd10();
  initsub1();
  initadd1();
  initsub90();
  initadd90();
  initLCDbacklight();
  initRdown();
  initRup();
  initGdown();
  initGup();
  initPdown();
  initPup();
  initIdown();
  initIup();
  initDdown();
  initDup();
  initmagvardown();
  initmagvarup();
  initdodgeleft();
  initdodgeright();
    }
  
    function initButton() {
      document.getElementById('button').addEventListener('click', toggle);
    }
    function toggle(){
      websocket.send('toggleAP');
    window.location.reload();
    }
  
    function initsub10() {
      document.getElementById('sub10').addEventListener('click', sub10);
    }
    function sub10(){
      websocket.send('subten');
      window.location.reload();
    }
  
    function initadd10() {
      document.getElementById('add10').addEventListener('click', add10);
    }
    function add10(){
      websocket.send('addten');
      window.location.reload();
    }
  
    function initsub1() {
      document.getElementById('sub1').addEventListener('click', sub1);
    }
    function sub1(){
      websocket.send('sub1');
      window.location.reload();
    }
  
    function initadd1() {
      document.getElementById('add1').addEventListener('click', add1);
    }
    function add1(){
      websocket.send('add1');
      window.location.reload();
    }
  
    function initsub90() {
      document.getElementById('sub90').addEventListener('click', sub90);
    }
    function sub90(){
      websocket.send('subninety');
      window.location.reload();
    }
  
    function initadd90() {
      document.getElementById('add90').addEventListener('click', add90);
    }
    function add90(){
      websocket.send('addninety');
      window.location.reload();
    }
  
    function initdodgeleft() {
      document.getElementById('dodgeleft').addEventListener('click', dodgeleft);
    }
    function dodgeleft(){
      websocket.send('dodgeleft');
    }
  
    function initdodgeright() {
      document.getElementById('dodgeright').addEventListener('click', dodgeright);
    }
    function dodgeright(){
      websocket.send('dodgeright');
  
    }
    function initLCDbacklight() {
      document.getElementById('LCDbacklight').addEventListener('click', LCDbacklight);
    }
    function LCDbacklight(){
      websocket.send('lcdlightswitch');
    }
  
    function initRdown() {
      document.getElementById('Rdown').addEventListener('click', Rdown);
    }
    function Rdown(){
      websocket.send('Rdown');
      window.location.reload();
    }
  
    function initRup() {
      document.getElementById('Rup').addEventListener('click', Rup);
    }
    function Rup(){
      websocket.send('Rup');
      window.location.reload();
    }
  
    function initGdown() {
      document.getElementById('Gdown').addEventListener('click', Gdown);
    }
    function Gdown(){
      websocket.send('Gdown');
      window.location.reload();
    }
  
    function initGup() {
      document.getElementById('Gup').addEventListener('click', Gup);
    }
    function Gup(){
      websocket.send('Gup');
      window.location.reload();
    }
  
    function initPdown() {
      document.getElementById('Pdown').addEventListener('click', Pdown);
    }
    function Pdown(){
      websocket.send('Pdown');
      window.location.reload();
    }
  
    function initPup() {
      document.getElementById('Pup').addEventListener('click', Pup);
    }
    function Pup(){
      websocket.send('Pup');
      window.location.reload();
    }
  
    function initIdown() {
      document.getElementById('Idown').addEventListener('click', Idown);
    }
    function Idown(){
      websocket.send('Idown');
      window.location.reload();
    }
  
    function initIup() {
      document.getElementById('Iup').addEventListener('click', Iup);
    }
    function Iup(){
      websocket.send('Iup');
      window.location.reload();
    }
  
    function initDdown() {
      document.getElementById('Ddown').addEventListener('click', Ddown);
    }
    function Ddown(){
      websocket.send('Ddown');
      window.location.reload();
    }
  
    function initDup() {
      document.getElementById('Dup').addEventListener('click', Dup);
    }
    function Dup(){
      websocket.send('Dup');
      window.location.reload();
    }
  
    function initmagvarup() {
      document.getElementById('magvarup').addEventListener('click', magvarup);
    }
    function magvarup(){
      websocket.send('magvarup');
      window.location.reload();
    }
  
    function initmagvardown() {
      document.getElementById('magvardown').addEventListener('click', magvardown);
    }
    function magvardown(){
      websocket.send('magvardown');
      window.location.reload();
    }

</script>

</body>
</html>

)rawliteral";


// ----------END WEB INTERFACE FOR WIFI CONTROL ---------------



 //****  SETUP    SETUP   SETUP   ***** 

 void setup() {

     Serial.begin(115200); // Serial conection to Serial Monitor
     delay(1000); // give chip some warmup on powering up   
     Serial.println("Let's go somewhere");

// Red Button setup     
debouncer.attach(BUTTON_PIN,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
debouncer.interval(25); // Use a debounce interval of 25 milliseconds

//IR Reciever startup
IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

//LCD startup
 lcd.init(); 
 lcd.setCursor(0,0);

//ledcSetup for IBT-2 motor controller (PWMChannel, PWMFreq, PWMResolution); //sets frequency (24kHz) and resolution (8bit) for motor controller PWM
//purpose is to change frequency and reduce motor noise
ledcSetup(7,24000,8);
ledcSetup(8,24000,8);
ledcAttachPin(L_PWM, 7); //assigns PWM channel for frequency shift (motor noise fix)
ledcAttachPin(R_PWM, 8); //assigns PWM channel for frequency shift (motor noise fix)


// ------------------------------    WIFI setup   ------------------------------------------
 
/*   // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
 */

 // Connect to Wi-Fi
  WiFi.mode(WIFI_AP_STA);

    // ESP32 Board add-on after version > 1.0.5
 // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  WiFi.softAP(ssid, password, 1, 0);  //changed from WiFi.begin to WiFi.softAP to create access point for phone to connect (no internet passthrough).
  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(1000);


esp_wifi_set_promiscuous(true);
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
esp_wifi_set_promiscuous(false);

//Serial.println("Wifi Channel ");
//Serial.println(WiFi.channel);
    Serial.println("Starting WiFi access point..");

// finding IP address of Access Point
    IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);          // default is 192.168.4.1
  
  initWebSocket();   

// initialize esp-now (see WIFI tab)
  espnowsetup();

 /* // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);  
  });*/

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);  
  });

  // Start server
  AsyncElegantOTA.begin(&server); // OTA updates. Go to 192.168.4.1/update
  server.begin();

  //------------------- end WIFI setup -----------------------

   
 #if Compass == 0
 //SETUP FOR MinIMU9 
 lcd.print("Starting Compass");
 Serial.println("Starting Compass");
     I2C_Init();
   //Serial.println("Pololu MinIMU-9 + Arduino AHRS");
   // digitalWrite(STATUS_LED,LOW);
  delay(1500);
  Accel_Init();
  Compass_Init();
  Gyro_Init();  
  delay(20);  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;   
    AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
   // Serial.println(AN_OFFSET[y]);
 
  delay(2000);
 // digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
  // End setup data for MinIMU9
 #endif
 
 Key0_Pressed();         // make doubly-sure AP steering deactivated as intial condition
 Serial.println("Setup Complete");

 }  // end setup
  

/****************************   BEGIN LOOP   ********************************/
 

void loop()
{

 #if Compass == 0
  if(just_started)
  { // setup();  //this runs setup a second time because I (JE) find gyros zero state does not initialize when powered up, runs once
     just_started = 0;
  }
 #endif 
 
 ws.cleanupClients();  // controls websocket traffic. probably doesn't need to run every loop. could put on a timer if necessary
redbutton();  // tells the steering activate/deactivate toggle button to listen for presses
IRREMOTE();   // tells the IR remote to listen

//espnowhandler();

A_P_Loop(); // Autopilot Loop
//esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &incomingReadings, sizeof(incomingReadings));

//esp_now_send(broadcastAddress, (uint8_t *) &incomingReadings, sizeof(incomingReadings));

 }    

