/* Jeff's Arduino Autopilot Version ESP32_Wheel1.3. 
Simple compass-only marine autopilot that functions like a Raymarine ST2000 or similar.
Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver, red button to toggle steering on/off, LSM303 9DOF compass, and 16x2 LCD. Rudder feedback potentiometer optional.
If using Wifi, pilot is operational with only ESP32, IMU compass, motor controller, and motor. 
WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing browser to 192.168.4.1.

Version Notes: 
--created rudder position gauge that loads in the HTML interface and updates in realtime
--adjusted motorspeedMIN from 30 to 100. 
--tweaked dodge functions to reactivate steering after dodging. 
--changed LCD output. 


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
  #include <LiquidCrystal.h>
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
#include <SPIFFS.h> // for putting html and css in the flash


// Replace with your WIFI network credentials. Point control device browser to 192.168.4.1
const char* ssid = "PlotDevice";
const char* password = "thereyouare";

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
 
   // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
 
 // Connect to Wi-Fi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);  //changed from WiFi.begin to WiFi.softAP to create access point for phone to connect (no internet passthrough).
  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(1000);
    Serial.println("Starting WiFi access point..");

// finding IP address of Access Point
    IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);          // default is 192.168.4.1
  

  initWebSocket();   
 //     initWebServer();        //not sure if needed

 /* // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);  
  });*/

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
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

A_P_Loop(); // Autopilot Loop


 }    

