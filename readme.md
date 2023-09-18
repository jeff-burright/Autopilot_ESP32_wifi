/*
Jeff's Arduino Autopilot Version ESP32_Wheel1.3.
 
Simple compass-only marine autopilot that functions like a Raymarine ST2000 or similar.

Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver, red button to toggle steering on/off, LSM303 9DOF compass, and 16x2 LCD. Rudder feedback potentiometer optional.
If using Wifi, pilot is operational with only ESP32, IMU compass, motor controller, and motor. 
WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing browser to 192.168.4.1.

Version Notes: 
--created rudder position gauge that loads in the HTML interface and updates in realtime
--adjusted motorspeedMIN from 30 to 100. 
--tweaked dodge functions to reactivate steering after dodging. 
--changed LCD output layout. 


Issues: None at present. Code has lots of old artifacts from Jack's version
Experimental aspects: 
wiper motor noise reduction via PWM frequency modulation seems to work. 

To do:
double check PID settings

Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.

Use this code at your own risk. 
*/


INSTALLATION NOTES
The build folder has a binary that can be directly uploaded to an ESP32, but this build DOES NOT CONTAIN THE HTML FILE. The HTML file was originally embedded directly in the sketch, but it got so long that it was bugging during uploads and randomly not including code sections. As a result, the HTML file was moved onto the SPI Flash of the ESP32 via the SPIFFS function. The file system uploader for ESP32 is currently not working in the Arduino IDE, so unfortunately you will need to install VS.Code and PlatformIO in order to upload the index.html file into the device. (tutorial here: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/)

Check out the parts list document file for build instructions, wiring pinouts, and photos of my two installations on wheel and tiller. 

Use at your own risk and good luck! 




