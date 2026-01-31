**The Plot Device. 
ESP32 Wheel/Tiller Autopilot, Version 1.7  (with parts list and build info!).**

This is a simple, inexpensive (~$150 total project cost including motor linkage), compass-only marine autopilot for wheel or tiller that functions like a Raymarine ST2000 or a CPT. Version 1.7 integrates the tiller and wheel versions of the code and allows the user to switch between the two versions inside the USER INPUT section of the main .INO file. 

Check out the parts list document file within this repo for part links/costs, general build instructions, wiring pinouts, and photos of my two installations on wheel and tiller. Note also the list of libraries that need to be installed within the Arduino IDE software before you will be able to successfully compile the code. You can google the library names, download the zip files from their respective repositories, then within the Arduino IDE add the libraries from zip file. 

**Features:** Instantaneous pilot control and auto-updating heading/rudder readings via an HTML interface from any device with a browser (the pilot broadcasts its own access point, no app or internet needed). The code also supports a physical control box with an on/off toggle button and an LCD screen, plus a small infrared remote for line of sight control to a sensor. Control is also possible via a Lilygo smart watch (~$40) using the ESP-NOW protocol. See my other repository for the watch code. Video of the control options is here: https://www.youtube.com/watch?v=iN40WbeqQ1k

Stable on ESP32 devkit. Software support for IBT-2 motor controller, LSM303 9DOF compass, IR remote receiver (optional), single button to toggle steering on/off (optional),  16x2 LCD (optional), and rudder feedback potentiometer (optional). The rudder feedback sensor controls the wheel stop function when it reaches a preset angle, and it improves overall pilot efficiency, so it is recommended for the wheel version especially. The parts list describes how you can build one for about $20.  

If using a phone or tablet as your only interface, the pilot is operational with only an ESP32, IMU compass, motor controller, and motor. 

WIFI control of the pilot accessible by connecting a device to the pilot's SSID and directing the browser to 192.168.4.1. The WIFI interface allows realtime adjustment of course, PID settings, and magnetic variation. 

Over the air firmware updates may be uploaded by connecting to the pilot's SSID and going to 192.168.4.1/update.

**Thanks to the work of Jack Edwards (https://github.com/CoyoteWaits/Jack_Edwards_Autopilot) from which this project grew.**

Videos of the pilot in use: <br>
https://www.youtube.com/watch?v=wBYxuOjN69g <br>
https://www.youtube.com/shorts/7anuzTAU0g8 <br>
https://www.youtube.com/shorts/sYkNy174X3A <br>
https://www.youtube.com/watch?v=WxV1NFOIDxg <br>
https://www.youtube.com/shorts/Dm93frkyZY0 <br>

**INSTALLATION NOTES**
-- In the main Autopilot.INO tab, you can set your own SSID and password for the pilot, as well as set user-defined variables in the code. It is recommended to check these variables against your own setup. 

-- The user setting #define LSMLib allows you to select the method of tilt compensation. I prefer "0" and think it's more stable at heel, but in order to work properly in this configuration the IMU must be installed face down. You will also need to run the Calibrate.INO example sketch in the LSM303 library and collect the Min/Max XYZ values for your IMU. These values must be input into the main .INO tab starting around line 320 of the code. 

--Even with all the right libraries and a successful compile/upload, the UI webpage will not load without one additional tweak. The HTML file was bugging out because the ESPAsyncWebServer library uses % to mark placholder text (like for settings and heading information), but the css stylesheet for the rudder gauge also uses % for its normal meaning. If you want to edit the code for your own purposes rather than just upload my binary, you will need to follow the instructions on this page for changing all the % placeholders to $ within the WebResponseImpl.h file in your version of the ESPAsyncWebServer library: https://stackoverflow.com/questions/74649351/espasyncwebserver-request-send-p-problem. 



Use at your own risk and good luck! 


![jeff-burright/Autopilot_ESP32_wifi/](https://github.com/jeff-burright/Autopilot_ESP32_wifi/blob/main/AP-screenshot_v1_3.png)

