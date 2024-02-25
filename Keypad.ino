
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
 lcdlight = 1;
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


 

///////////////////////////////////////////////////
/***********************************************/

        void Key0_Pressed()   // Deactivate steering
        {
                   Steering_Mode = 0;
          Mode = "OFF";
          Steering = false;
          Rudder_Stop();
         // GPS_Was_Available = false;
          Screen = 0;  
          //toggle = false; // resets key 3 to tack mode instead of wind mode

          //resets display
          lcd.begin(16,2);


        #if BEARINGRATE_OFFSET == 1
          bearingrate_Offset = 0; // bearingrate_Offset applied in Tab Subs void Bearing_Rate()
                                  // set with keys 1, 2, 3 maybe 22 reset to 0 in key zero
        #endif  
          LCD();
          lcd.setCursor(0,0);
          lcd.print("STANDBY ");       
     
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
          lcd.print("STEERING");
     
}  // end key1 pressed


 void redbutton(){    // Toggle between steering activate/deactivate using physical button at Button_Pin
   debouncer.update(); // Update the Bounce instance
   if ( debouncer.fell() ) {  
            toggleAP(); 
          }
 }

void toggleAP(){  // Toggle between steering activate/deactivate
if (Steering_Mode==0){Key1_Pressed();}
else {Key0_Pressed();}
 Serial.println("toggle on/off");
 Serial.println(heading_to_steer);
                //notifyClients();   // send notification to web interface to update values
}
               

void remotesub10(){ 

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 10;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                 Serial.println("minus 10 deg");  
                            // notifyClients();   // send notification to web interface to update values        
}

void remoteadd10(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 10;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 10 deg");
                            // notifyClients();   // send notification to web interface to update values
                           
}

void remotesub1(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 1;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("minus 1 deg");
             //  notifyClients();   // send notification to web interface to update value          
}

void remoteadd1(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 1;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 1 deg");
              // notifyClients();   // send notification to web interface to update values           
}


void remotesub90(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer - 90;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("minus 90 deg");
              // notifyClients();   // send notification to web interface to update values
}


void remoteadd90(){

if(Steering_Mode==1) heading_to_steer = heading_to_steer + 90;
              if (heading_to_steer < 0) heading_to_steer = heading_to_steer +360;
              if (heading_to_steer > 360) heading_to_steer = heading_to_steer -360; 
              lcd.setCursor(13, 1);
              lcd.print(heading_to_steer,1);
                   Serial.println("plus 90 deg");
              // notifyClients();   // send notification to web interface to update values
}


void dodgeleft(){
        //  if(Steering_Mode == 0 || Steering_Mode ==5) break;  
         //   DODGE_MODE = true;
         //   Previous_Mode = Steering_Mode;
          //  Steering_Mode ==5;
          Key0_Pressed();
           motorspeed = motorspeedMAX;
            Left_Rudder();
            delay(500);

  //        if(Steering_Mode == 0 || Steering_Mode ==5) return; 
          Rudder_Stop();
            delay(2000);
           Key1_Pressed();
         //  DODGE_MODE = false;
         // Steering_Mode = Previous_Mode;
}

void dodgeright(){

           // DODGE_MODE = true;
           // Previous_Mode = Steering_Mode;
           //  Steering_Mode ==5;
          Key0_Pressed();
           motorspeed = motorspeedMAX;
            Right_Rudder();
            delay(500);
           Rudder_Stop(); 
           delay(2000);
           Key1_Pressed();
         // DODGE_MODE = false;
         // Steering_Mode = Previous_Mode;
          }


void lcdlightswitch(){

if (lcdlight==0){lcd.backlight();}
else {lcd.noBacklight();}
lcdlight = !lcdlight;
 Serial.println("LCD lightswitch");
               // notifyClients();   // send notification to web interface to update values

}


void Rup(){
  deadband = deadband + 0.5;
}

void Rdown(){
  if (deadband !=0){
  deadband = deadband - 0.5;
}
}

void Gup(){
  K_overall = K_overall + 0.5;
}

void Gdown(){
  if (K_overall != 0){
    K_overall = K_overall - 0.5;
}
}

void Pup(){
  K_heading = K_heading + 0.1;
}

void Pdown(){
  if (K_heading != 0){
    K_heading = K_heading - 0.1;
}
}

void Iup(){
  K_integral = K_integral + 0.0001;
}

void Idown(){
  if (K_integral != 0){
    K_integral = K_integral - 0.0001;
}
}

void Dup(){
  K_differential = K_differential + 0.5;
}

void Ddown(){
  if (K_differential != 0){
    K_differential = K_differential - 0.1;
}
}

void magvarup(){
  MagVar_default = MagVar_default + 1;
}

void magvardown(){
  MagVar_default = MagVar_default - 1;
}



