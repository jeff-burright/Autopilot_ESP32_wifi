**The Plot Device. 
ESP32 Wheel/Tiller Autopilot, Version 1.4  (with parts list and build info!).**

Arduino autopilot for ESP32 with wired compass. Fast WiFi control and auto-updating heading/rudder readings via an HTML interface from any device with a browser (no app or internet needed). The code also supports a physical control box with an on/off toggle button and an LCD screen, plus a small infrared remote for line of sight control to a sensor. 

This is a simple, inexpensive (~$150 total project cost including motor linkage), compass-only marine autopilot that functions like a Raymarine ST2000 or similar. The main code version is designed for control of a wheel, similar to the CPT autopilot. For the tiller version, see the Tiller branch of this repository. The main differences are the removal of the rudder position indicator for the tiller version and the reversal of motor commands. 

Stable on ESP32. Software support for IBT-2 motor controller, IR remote receiver (optional), single button to toggle steering on/off (optional), LSM303 9DOF compass, 16x2 LCD (optional), and rudder feedback potentiometer (optional). The rudder feedback sensor controls the wheel stop function when it reaches a preset angle, so it is recommended for the wheel version especially. The parts list describes how you can build one for about $20.  

If using a phone or tablet as your only interface, the pilot is operational with only an ESP32, IMU compass, motor controller, and motor. 

WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing the browser to 192.168.4.1. The WIFI interface allows realtime adjustment of course, PID settings, and magnetic variation. 

Over the air firmware updates may be uploaded by connecting to the pilot's SSID and going to 192.168.4.1/update.

**Version Notes:**
--fixed tilt-compensation of compass.
--created rudder position gauge that loads in the HTML interface and updates once per second.
--adjusted motorspeedMIN from 30 to 100 for a little faster wheel response (max is 255). 
--tweaked dodge functions to reactivate steering at new course a few seconds after dodging. Still experimenting with this. 
--changed LCD output layout

**Issues:**
None at present. Pilot has been tested under motor and under sail on two boats. Code has lots of old unused artifacts from Jack's version that I could clean up.

**Experimental aspects:** 
Wiper motor noise reduction via PWM frequency modulation seems to be working. 

**To do:**
double check PID settings and set optimal defaults and adjustment increments.

**Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.**


**INSTALLATION NOTES**
Check out the parts list document file within this repo for part links/costs, general build instructions, wiring pinouts, and photos of my two installations on wheel and tiller. Note also the list of libraries that need to be installed within the Arduino IDE software before you will be able to successfully compile the code. 

The build folder in this repo has a ~900kb binary file that can be directly uploaded to an ESP32 without having to install all the libraries. You will need to have the Arduino IDE or other ESP32 uploading software on your computer to upload the binary the first time via a USB cable (remember to hold down the boot button on the ESP while it's uploading or else it will fail). If you want to change the code for your own purposes and try to recompile, first you will need to find all the libraries. Even with all the right libraries and a successful compile/upload, the UI webpage will not load without one additional tweak. The HTML file was bugging out because the ESPAsyncWebServer library uses % to mark placholder text (like for settings and heading information), but the css stylesheet for the rudder gauge also uses % for its normal meaning. If you want to edit the code for your own purposes rather than just upload my binary, you will need to follow the instructions on this page for changing all the % placeholders to $ within the WebResponseImpl.h file in your version of the ESPAsyncWebServer library: https://stackoverflow.com/questions/74649351/espasyncwebserver-request-send-p-problem. 

As an alternative to tweaking the library, the HTML and CSS code can be moved out of the main .ino file onto the SPI Flash of the ESP32 via the SPIFFS function as separate files. The file system uploader for ESP32 is currently not working in the Arduino IDE, so unfortunately you would need to install VS.Code and PlatformIO in order to upload the index.html file into the device. (tutorial here: https://randomnerdtutorials.com/esp32-vs-code-platformio-spiffs/). I did this before discovering and fixing the % library bug for Version 1.4.



Use at your own risk and good luck! 




