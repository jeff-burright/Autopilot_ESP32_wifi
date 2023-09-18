# Autopilot_ESP32_wifi
Arduino autopilot for ESP32. WiFi control with HTML interface (ESP32 is access point). Also has IR remote and toggle button control. Based on the arduino autopilot built by Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot).

**Jeff's Arduino Autopilot Version ESP32_Wheel1.3.**
 
Simple compass-only marine autopilot that functions like a Raymarine ST2000 or similar. This version is designed for a wheel. For the tiller version, see the Tiller branch of this repository. 

Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver, red button to toggle steering on/off, LSM303 9DOF compass, and 16x2 LCD. Rudder feedback potentiometer optional.
If using Wifi, pilot is operational with only ESP32, IMU compass, motor controller, and motor. 
WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing browser to 192.168.4.1.
The WIFI interface allows adjustment of PID settings and magnetic variation. 

**Version Notes:**
--fixed tilt-compensation of compass
--created rudder position gauge that loads in the HTML interface and updates in realtime
--adjusted motorspeedMIN from 30 to 100. 
--tweaked dodge functions to reactivate steering after dodging. 
--changed LCD output layout


**Issues:**
None at present. Pilot has been tested under motor and under sail on two boats. Code has lots of old artifacts from Jack's version
Experimental aspects: 
wiper motor noise reduction via PWM frequency modulation seems to work. 

**To do:**
double check PID settings

**Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.**



**INSTALLATION NOTES**
The build folder has a binary that can be directly uploaded to an ESP32, but this build DOES NOT CONTAIN THE HTML FILE. The HTML file was originally embedded directly in the sketch, but it got so long that it was bugging during uploads and randomly not including code sections. As a result, the HTML file was moved onto the SPI Flash of the ESP32 via the SPIFFS function. The file system uploader for ESP32 is currently not working in the Arduino IDE, so unfortunately you will need to install VS.Code and PlatformIO in order to upload the index.html file into the device. (tutorial here: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/)

Check out the parts list document file for build instructions, wiring pinouts, and photos of my two installations on wheel and tiller. 

Use at your own risk and good luck! 




