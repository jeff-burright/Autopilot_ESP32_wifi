/* Jeff's Arduino Autopilot Version ESP32_2.0. 
Simple compass-only marine autopilot that functions like a Raymarine ST2000 or similar.
Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver, red button to toggle steering on/off, LSM303 9DOF compass, and 16x2 LCD. Rudder feedback potentiometer optional.
If using Wifi, pilot is operational with only ESP32, IMU compass, motor controller, and motor. 
WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing browser to 192.168.4.1.

Version Notes: 
-- updated for ESP32 Core 3 from Core 2.1.2
-- updated libraries
-- code slimming to remove unused components
-- added IMU install correction factor to correct compass readings based on orientation of installation
-- added low pass filter for rudder sensor to correct jittery readings
-- removed separate code for tiller vs wheel. No longer needed. If necessary to reverse steering for a tiller, you can simply swap the input pins on the motor controller. 
-- added compass calibration readings into the HTML as a toggle button to display values. Calibration values still need to be hard coded in the main .ino file to change heading readings. 
-- pulled the HTML code out of the main .INO file into two separate HTML.h files. One contains the rudder position gauge and one does not. The HTML page loaded is defined by the RUDDER_MODE user input variable.
-- added rudder sensor toggle in the HTML that loads when RUDDER_MODE=0. This allows a pilot using rudder feedback to deactivate the rudder sensor if it glitches or gets damaged and revert to steering without rudder feedback.


Issues: None at present. 

To do:
add more buttons for a physical interface?


Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.
Use this code at your own risk. 
*/


#include <index_html.h>  // HTML interface without rudder position gauge
#include <index_rudder_html.h>  // HTML interface with rudder position gauge

#include <LiquidCrystal_I2C.h> // used for 16x2 LCD screen
#include <Wire.h>
#include <Bounce2.h> // use for red button toggle pilot on/off. could be used to add more buttons
#include <L3G.h> // IMU compass
#include <BTS7960.h> // IBT-2 motor controller library
#include <IRremote.h> // infrared remote control
#include <IRremote.hpp>
#include <LSM303.h> // IMU compass
 LSM303 compass;
#include <WiFi.h>  // necessary for WiFi control
#include <AsyncTCP.h> // necessary for WiFi control
#include <ESPAsyncWebServer.h>  // necessary for WiFi control
#include <ElegantOTA.h>  // over the air update of compiled binary via 192.168.4.1/update
#include <ArduinoJson.h> // Include ArduinoJson Library. Necessary for updating heading data and rudder position gauge in the HTML
#include <esp_now.h>  //for smartwatch UI
#include <esp_wifi.h> // for smartwatch UI
#include <Keypad.h> // turns button presses in interface with pilot commands


// Replace with your WIFI network credentials. Point control device browser to 192.168.4.1
const char* ssid = "PlotDevice";
const char* password = "";


/******       USER CUSTOMIZATION INPUT       ********/
#define Compass 0 //  0 for LSM303, 1 for BNO055
#define LSMLib 0 //  1 uses LSM library for compass tilt compensation, 0 uses original Compass_Heading() function.
#define IMU 2 //  1 for home prototype, 2 for IMU on wheel pilot, 3 for IMU on tiller pilot (Jeff's)    //determines which set of calibration data is used these extra versions were added to code 7/11/17 J14.4
#define Motor_Controller 3  // 1 for Pololu Qik dual controller, 2 for Pololu Trex dual controller, 3 for generic Controller (I use IBT-2)
#define RUDDER_MODE 0 // 0 uses rudder position (requires rudder position indicator to be installed), 1 does not
#define RUDDER_OFFSET 1 // 1 uses rudder offset, 0 does not (requires rudder position indicator)
#define BEARINGRATE_OFFSET 1 // 1 to use 0 to not use
#define REVERSEYAW 0 // some installations have reversed compass heading values. Set to 1 to correct. 
#define PHYSICALBUTTONS 0 // 1 to use, 0 to not use
#define PIDvals 1 // select starting PID values from different options (see below in this .ino). 
#define PID_MODE 3 // See description PID tab.
#define smartwatch 1 // activates smartwatch UI functions
#define RUDDER_SENSOR 0 // 0 uses potentiometer; 1 uses hall sensor for rudder position indicator. The difference is the calibration in the PID tab.

float Magnetic_Variation = 14; // sets default magnetic variation based on your boat's latitude
float IMU_Install_Correction = 88; // use this to correct the compass heading reading based on the orientation of your IMU once installed. This will add to the raw compass reading in the Subs tab
boolean compass_calib = false;
boolean rudd_sensor_off = false;
// NOTE: search for "motorspeedMIN" to set minimum power to motor. Currently set at 100 for faster response. 

///////////////////////////////////////////////////
//////////////// END USER INPUT ///////////////////
//////////////////////////////////////////////////


  String rudd_dir; // Rudder direction for LCD. Value is set in LCD tab. Options are L or R
  String jsonString; // place to store heading and rudder data for web socket (HTML display)


  // PID starting settings. These can be adjusted from within the user interface but will reset each time you cycle the power. 
 #if PIDvals == 1  // Jack's values minus rudder indicator
 float K_overall = 1;
 float K_heading = .6;
 float K_differential = 2;
 float K_integral = 0.000;
 float PID_Ks[4] = {K_overall, K_heading, K_differential, K_integral};
 //float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
// float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output (PID Mode 1);
 #endif

  #if PIDvals == 2 // Experimental
 float K_overall = .75;
 float K_heading = .4;
 float K_differential = .01;
 float K_integral = 0.000;
 float PID_Ks[4] = {K_overall, K_heading, K_differential, K_integral};
 //float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
// float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output (PID Mode 2);
 #endif


  #if PIDvals == 3  //pypilot starting gains. Note that in Keypad tab, PIDvals 3 has a different scale of +/-
 float K_overall = 1;
 float K_heading = .003;
 float K_differential = .01;
 float K_integral = 0.000;
 float PID_Ks[4] = {K_overall, K_heading, K_differential, K_integral};
 //float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
// float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output (PID Mode 1);
 #endif


 boolean just_started = 0; // to do a second setup so get a better gyro startup
 float K_course_avg = .999; //used to smooth gps course in PID/ void Actual_GPS_Steering().999 will smooth over 1000 points
 float Maximum_Rudder = 40; // rudder stop angle in degrees (requires rudder position sensor) 
 float Tack_Angle = 100;  // angle throug which boat will tack when tack L or R pressed (keys 4 and 6 in TACK mode(3) -- not used in Jeff's version.
 int Tack_rudder_MAX = 32;// limits rudder so it doesn't slow boat too much,  need to tune -- not used in Jeff's.
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


//  print modes for IMU
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

// Rudder sensor low pass filter
const float alpha = 0.10f;
const int rudddeadband = 6;
float filt = 0.0f;
int stableOut = 0;

float MagVar; //Magnetic Variation E is plus, W is minus 
 
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
 float counts = 0;

// ------------------ESP-NOW Parameters (Smartwatch)------------------------//
#if smartwatch == 1
// REPLACE WITH THE MAC Address of your receiver 
//uint8_t peerAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x64};

// Set your new MAC Address for THIS DEVICE
//uint8_t newMACAddress[] = {0x32, 0xAE, 0xA4, 0x07, 0x0D, 0x66};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcasts to any listening device.

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
//Must match the receiver structure (see separate watch code on github)
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

#endif


 // ----------------- IBT-2 motor controller ----------------------

//assign IBT-2 Pins
//const uint8_t EN = 25;  // assigned but not used. EN pins on controller are connected to 5v voltage to controller to make them always active.
const uint8_t L_PWM = 32;
const uint8_t R_PWM = 33;

   float motorspeedMIN = 75; // This value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 255; // Max PWM value for IBT-2 (full power).
 
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
// this is data for LSM303 IMU compass. Values are derived from running a calibration sketch and recording the min/max after waving the compass around. 
// the below values are not used if 
//#if LSMLib == 0
#if IMU == 1  // home prototype
//these values are for the home prototype
  #define M_X_MIN -579   
  #define M_Y_MIN -732
  #define M_Z_MIN -392   
  #define M_X_MAX 603
  #define M_Y_MAX 414
  #define M_Z_MAX 395 
#endif

#if IMU == 2 // Wheel boat (Macguffin)
  #define M_X_MIN -663   
  #define M_Y_MIN -683
  #define M_Z_MIN -611   
  #define M_X_MAX 453
  #define M_Y_MAX 427
  #define M_Z_MAX 460 
#endif

#if IMU == 3 // Tiller (Puffin)
  #define M_X_MIN -574   
  #define M_Y_MIN -759
  #define M_Z_MIN -331   
  #define M_X_MAX 604
  #define M_Y_MAX 406
  #define M_Z_MAX 456
#endif


//compass calibration from the HTML interface (see compcalib in the "Subs" tab and CALREP in the wifi tab)
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
char report[80];

char pidreport[80];  //added for pid debugging on the phone
//

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
float compassheading;  // experiment

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



// -------------------------------------------------------------------------------------------------
// -------------------------------WEB INTERFACE FOR WIFI CONTROL AT 192.168.4.1 --------------------
// -------------------------------------------------------------------------------------------------












// ----------END WEB INTERFACE FOR WIFI CONTROL ---------------



//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
 //****  SETUP    SETUP   SETUP   ***** 
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////


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
ledcAttach (L_PWM, 24000, 8);
ledcAttach (R_PWM, 24000, 8);


// Rudder low pass filter
int raw = analogRead(Rudder_Pin);
filt = raw;
stableOut = raw;


// ------------------------------    WIFI setup   ------------------------------------------
 

 // Connect to Wi-Fi
  WiFi.mode(WIFI_MODE_APSTA);

// Set wifi mode to BGNLR (currently broken. SSID won't broadcast if these are active)
  //esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B| WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
  // esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B| WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR);
  
  //esp_wifi_set_ps(WIFI_PS_NONE);  // experimental. works but probably not needed

    // ESP32 Board add-on after version > 1.0.5
 // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  //WiFi.softAP(ssid, password);
 WiFi.softAP(ssid, password, 1, 0);  //changed from WiFi.begin to WiFi.softAP to create access point for phone to connect (no internet passthrough).
  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(1000);

esp_wifi_set_promiscuous(true);
esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); //this facilitates the watch interface with ESPNOW and WiFi running simultaneously
esp_wifi_set_promiscuous(false);

    Serial.println("Starting WiFi access point..");

// finding IP address of Access Point
    IPAddress IP = WiFi.softAPIP();
Serial.print("AP IP address: ");
Serial.println(IP);          // default is 192.168.4.1
  
  initWebSocket();   

#if smartwatch == 1
// initialize esp-now for watch control (see WIFI tab)
  espnowsetup();
#endif


  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  
  #if RUDDER_MODE == 0  
  request->send_P(200, "text/html", index_rudder_html, processor); 
  #endif 

  #if RUDDER_MODE == 1  
  request->send_P(200, "text/html", index_html, processor); 
  #endif 

  });

  // Start server
  ElegantOTA.begin(&server); // OTA updates. Go to 192.168.4.1/update
  server.begin();

  //------------------- end WIFI setup -----------------------

   
 #if Compass == 0
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
  // End setup data for compass IMU
 #endif
 
 Key0_Pressed();         // make doubly-sure AP steering deactivated as intial condition
 Serial.println("Setup Complete");

 }  // end setup
  

/****************************   BEGIN LOOP   ********************************/
 

void loop()
{

 #if Compass == 0
  if(just_started)
  {  setup();  //this runs setup a second time because I (JE) find gyros zero state does not initialize when powered up, runs once
     just_started = 0;
  }
 #endif 
 
 ws.cleanupClients();  // controls websocket traffic. probably doesn't need to run every loop. could put on a timer if necessary
#if PHYSICALBUTTONS == 1 
redbutton();  // tells the steering activate/deactivate toggle button to listen for presses
IRREMOTE();   // tells the IR remote to listen
#endif

if (compass_calib){
compcalib(); // turns on compass calibration readings included in the html interface. this doesn't need to stay past the initial installation and breaking in phase.
}

ElegantOTA.loop();
A_P_Loop(); // Autopilot Loop

 }    
