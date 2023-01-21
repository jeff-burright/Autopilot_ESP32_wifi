


// IR Remote start. ESP32 needs an IR Receiver Module with sense wire connected to IR_Receive_Pin. Pinout is changeable.

void IRREMOTE()  { 
  
  if (IrReceiver.decode()){   

 //Serial.println(IrReceiver.decodedIRData.address); 
  //Serial.println(IrReceiver.decodedIRData.command);

IRDECODE();
delay(300);
 IrReceiver.resume();
}

}

void IRDECODE() {
 
       switch(IrReceiver.decodedIRData.command){     
         case 69: //  Remote 'ON'    
      Key1_Pressed();
      break;

         case 71: //  Remote 'OFF'    
      Key0_Pressed();
      break;

         case 67: //  Remote 'Light ON'    
lcdlightswitch();
      break;

         case 90: //  Remote 'Light OFF'    
lcd.noBacklight();
      break;


         case 7: //  Remote '-10'    

remotesub10();
              
      break;

         case 9: //  Remote '+10'    
remoteadd10();
      break;


         case 22: //  Remote '-1`'    
     remotesub1();
      break;

         case 13: //  Remote '+1'    
remoteadd1();
      break;

         case 12: //  Remote '-90'    
remotesub90();
      break;

               case 94: //  Remote '+90'    
remoteadd90();
      break;

case 0xFFFFFF: // repeat
break;

default: break;

         } 

}


 /****************************************************/
// End void KeyPressed

/************************************************************************/


/***********************************************/

        void Key0_Pressed()   // Deactivate steering
        {
                   Steering_Mode = 0;
          Mode = "OFF";
          Steering = false;
         // GPS_Was_Available = false;
          Screen = 0;  
          //toggle = false; // resets key 3 to tack mode instead of wind mode


          lcd.begin(16,2);

        #if BEARINGRATE_OFFSET == 1
          bearingrate_Offset = 0; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                  // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
        #endif  
          LCD();
          lcd.setCursor(0,0);
          lcd.print("   Autopilot OFF");
          //lcd.setCursor(0,3);
          // lcd.print(Mode);  

          
     
        }  // end Key0 pressed

/*******************************************************/
void Key1_Pressed()    // Activate steering and hold course
{
          Steering_Mode = 1;
          Steering = true;
          Mode = "COMP";
          heading_to_steer = heading;  // tells the pilot to stay on the course at time of activation.
          integral_error = 0; // reset integral error
          #if RUDDER_OFFSET == 1
            Rudder_Offset = rudder_position; // placed before toggle this works for TACK and WIND see notes 10.21.16 
            #if PID_MODE == 3
              integral_error = Rudder_Offset/PID_Ks[0] + bearingrate_Offset/PID_Ks[0]; // sets initial  
                      //integral error = rudder position at time steering engaged divided by 
                      // overall PID gain because multiplied by overall gain in PID equation
            #endif
           #endif
           #if BEARINGRATE_OFFSET == 1
             bearingrate_Offset = - bearingrate; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                      // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
           #endif  
            
             lcd.setCursor(0,0);
          lcd.print("    Autopilot ON");

        //  lcd.setCursor(0,3);
          //lcd.print("          ");
          //lcd.setCursor(0,3);
          //lcd.print(Mode);

        //  lcd.setCursor(6, 0);
        //  lcd.print('Autopilot ON');

          //lcd.setCursor(12, 2);
          //lcd.print(heading_to_steer,1);
     
}  // end key1 pressed


 void redbutton(){    // Toggle between steering activate/deactivate using physical button at Button_Pin
   debouncer.update(); // Update the Bounce instance
   if ( debouncer.fell() ) {  
//if (Steering_Mode==0){Key1_Pressed();}
//else {Key0_Pressed();}
toggleAP(); 
}
 }

void toggleAP(){  // Toggle between steering activate/deactivate
if (Steering_Mode==0){Key1_Pressed();}
else {Key0_Pressed();}
 Serial.println("toggle on/off");
 Serial.println(heading_to_steer);
                notifyClients();   // send notification to web interface to update values
}

void remotesub10(){ 

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 10;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                 Serial.println("minus 10 deg");  
                             notifyClients();   // send notification to web interface to update values
            
}

void remoteadd10(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 10;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 10 deg");
                             notifyClients();   // send notification to web interface to update values
}

void remotesub1(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 1;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("minus 1 deg");
               notifyClients();   // send notification to web interface to update values
}


void remoteadd1(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 1;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 1 deg");
               notifyClients();   // send notification to web interface to update values
}


void remotesub90(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 90;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("minus 90 deg");
               notifyClients();   // send notification to web interface to update values
}


void remoteadd90(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 90;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 90 deg");
               notifyClients();   // send notification to web interface to update values
}


void lcdlightswitch(){

if (lcdlight==0){lcd.backlight();}
else {lcd.noBacklight();}
lcdlight = !lcdlight;
 Serial.println("LCD lightswitch");
                notifyClients();   // send notification to web interface to update values

}

