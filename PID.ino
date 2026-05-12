

/*******************************

PID TAB is the PID calculator and it computes and sends rudder control signals

*******************************/
/******************************
PID_MODE
 MODE 0: rudder_command = PID_output with no integral_error term
 MODE 1: rudder_command = rudder_command + PID_output.  This is a form of integral control. Was used successfully summer 2012
 MODE 3: rudder_command = PID_output and includes integral error.

RUDDER_MODE 
  Rudder mode determines which mode of PID control and rudder control are used.
  MODE 0: Rudder controlled to specified rudder_position
  MODE 1: Rudder keeps moving until rudder_command < deadband


***********************************/


        void Steer_PID()
        {  

#if RUDDER_MODE == 0
         //RUDDER_POSITION(); // 9.22.17 added to update rudder position and stop rudder in Dodge Mode but good for all modes in V14.7
         if(abs(rudder_position) > Maximum_Rudder)  {
           Rudder_Stop();
         }
#endif

        if(!DODGE_MODE) // if anything other than dodge mode, i.e., if steering
        { 

        heading_error = heading_to_steer - heading;  // This is the main PID proportional term for compass based steering           
        
        
           if (abs(heading_error) > 180) // this limits error to < 180 and makes turn short way on compass + right, - left
            {
               if(heading_to_steer > heading)  heading_error = heading_error - 360;
               if(heading_to_steer < heading) heading_error = 360 + heading_error;
            }

          /*
          lcd.setCursor(0,1);
          lcd.print("BRT       ");
          lcd.setCursor(5,1);
          lcd.print(bearingrate);
          */
          //if(abs(bearingrate) < 0.5 ) bearingrate = 0; // try to cut out bearing rate noise
      
           
          #if Compass == 0
           differential_error = bearingrate;
          #endif
          
          // Serial.print(heading_error);
          // Serial.print("  ");
          // Serial.println(differential_error);          
                        
          #if PID_MODE == 1          
            PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error); 
            rudder_command = PID_output + Rudder_Offset; // rudder offset set when key 1 or 3 pressed captures rudderoffset only used when User input RUDDER_OFFSET == 1
          #endif
          
          #if PID_MODE == 2   
           PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error); 
           rudder_command = rudder_command + PID_output + Rudder_Offset;  // this is a form of integral control was used summer of 2012 with estimated rudder position
          #endif
          
          #if PID_MODE == 3
           if(abs(heading_error)> deadband) // if heading error < deadband integral error stops accumulating
           {integral_error = integral_error + PID_Ks[3] * heading_error; // integral error used to control droop
           }
           if (!Steering) integral_error = 0;
           /*
           When sailing to windward a non zero rudder is needed to keep head into the wind with no integral term this requires a course error signal
           this is called droop and integral term will steer correct heading with non-zero rudder angles
           */
           integral_error = constrain(integral_error,-10,10);  // constrain intergral error to 10 degrees correction
           PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error + integral_error); // see void Key1_Pressed() for rudder offset
           rudder_command = PID_output;
          /* // Use this for debugging Integral error steering 
          lcd.setCursor(0,0);
          lcd.print("CMD ");
          lcd.print(rudder_command,1);// diagnostic
          lcd.setCursor(11,0);
          lcd.print("IE ");
          lcd.print(integral_error *PID_Ks[0],1);// diagnostic note THis is effective integral error because IE multiplied by PID_Ks[0] above.
          */ 

          #endif

          rudder_MAX = Maximum_Rudder;
         
        }   // end if  not DODGE_MODE
        

        if(!Steering)
           {
             rudder_command = 0;
             heading_to_steer = 0;
           }
    
         Rudder_Control(); // call Rudder for actual turning rudder
         
         if(Print_PID)
         {
          // Serial.print("course : "); Serial.println(course,1);
           //Serial.print("Heading Average: "); Serial.println(Cavg,1);
           Serial.print("heading to steer: "); Serial.println(heading_to_steer,1);
           Serial.print("Heading Error: "); Serial.println(heading_error,1);
         //  Serial.print("compass delta T in sec: "); Serial.println(compass_delta_T);
          // Serial.print("delta heading: "); Serial.println(delta_heading);
           Serial.print("Bearing Rate: ");Serial.println(bearingrate);
           Serial.print("Integral error: "); Serial.println(integral_error);  
           Serial.print("PID Output: "); Serial.println(PID_output);
          // Serial.print("Rudder: "); Serial.println(rudder_change);
           Serial.println("***********************"); 
         }   // end if Print PID       
       }  // End Void Steer_PID()  


/************************ Rudder Control **********************************************/
        void Rudder_Control()
        {
         float Rudder_Power_coeff = 0.5;  //  Set to 0 to not use. Use .5 for default starting point.applies more motor speed proportional to rudder position to have more force to increase rudder at
             // bigger rudder angles to counter weather helm. At bigger rudder positions it takes more force to increase rudder
          
          #if RUDDER_MODE == 0    
          if (!rudd_sensor_off){
          RUDDER_POSITION(); // update rudder position 
          }
          else {  
          rudder_position = 0;
          Rudder_Offset = rudder_position;
          }
         
          #endif

          if (RUDDER_MODE == 1) rudder_position = 0; // rudder feed back RFB not availabl  
          


          rudder_error = rudder_command - rudder_position;
          
          if(Steering_Mode == 0 || !sw1 || !sw2)  // sw1 and sw2 need to be on for automated steering
         {
            Steering = false;
         }
        
 
       if(Steering_Mode >0) Steering = true;
       // if(DODGE_MODE) Steering = false; //  if keypad LEFT or RIGHT RUDDER skip PID rudder control
            
        if(!DODGE_MODE) // do not steer if in dodge mode
        {
          #if RUDDER_MODE == 0
         // if(rudder_on)  RUDDER_POSITION(); //if rudder on up date position   
                   if (!rudd_sensor_off){
          RUDDER_POSITION(); // update rudder position 
          }
          else {  
          rudder_position = 0;
          Rudder_Offset = rudder_position;
          }

          #endif   

          if(Steering)
          {   

      //motorspeed variable tells motor controller PWM how high to be 0-255.
        motorspeed = motorspeedMAX/ 30 *rudder_error; // where at 30 degree rudder error speed = MAX. 
        motorspeed = abs(motorspeed);  // make it a positive integer
                if (motorspeed < motorspeedMIN) motorspeed = motorspeedMIN;

         #if Rudder_Power_coeff > 0    //see main page for description of rudder power coefficient
          if(abs(rudder_command) > abs(rudder_position))
          {
            motorspeed = motorspeedMIN + float(motorspeedMAX-motorspeedMIN) * ((abs(rudder_position)/Maximum_Rudder) * Rudder_Power_coeff);  //  User define in this void 
          }
         #endif
        motorspeed = constrain(motorspeed, motorspeedMIN, motorspeedMAX);
       // Serial.print("motor speed ");Serial.println(motorspeed);     
        


                       if(abs(rudder_error) < deadband) 
                         {
                          Rudder_Stop();
                         }                    
                   
                      if(rudder_error > deadband)   
                         {
                           Right_Rudder();
                         }
                         
                      if(rudder_error < - deadband)
                         {
                           Left_Rudder();
                         }  
  
         //  RUDDER_POSITION(); // 9.22.17 added to update rudder position and stop rudder in Dodge Mode but good for all modes in V14.7
                      //if(abs(rudder_position) > Maximum_Rudder)  
                       // {
                       //     Rudder_Stop();
                        //}

            } // end  if Steering 
        }  // end if(!DODGE_MODE)          
        }  // void rudder control

      
///////////////////////////////////////////////////////////////////////
  //------------------------  RUDDER POSITION  -----------------------
///////////////////////////////////////////////////////////////////////
#if RUDDER_MODE == 0
  void RUDDER_POSITION()
  {

    // use these print lines to get counts for calibration
     //Serial.print("Rudder pin = "); 
     //Serial.println(counts);

    #if RUDDER_SENSOR == 0
     float rudder_position_max = 45;
     float rudder_position_min = -45;
     float counts_max = 827;  // from calibration in print statement
     float counts_at_zero = 415;
     float counts_min = 0;
     float counts;
     #endif

#if RUDDER_SENSOR == 1
     float rudder_position_max = 45;
     float rudder_position_min = -45;
     float counts_max = 1500;  // from calibration in print statement
     //float counts_at_zero = 415;
          float counts_at_zero = 750;
     float counts_min = 0;
     float counts;
#endif

     //low pass filter
  int raw = analogRead(Rudder_Pin);  
  filt += alpha * (raw-filt);
  int smooth = (int)(filt + 0.5f);

  // low pass filter deadband
    if (abs(smooth - stableOut) > deadband) {
    stableOut = smooth;
  }

  counts = stableOut;

 
    // end low pass filter


      if(counts >= counts_at_zero) // linear calibration from zero
      {
          rudder_position = rudder_position_max *(counts - counts_at_zero) / (counts_max - counts_at_zero);
      }
      else
      {
          rudder_position = rudder_position_min * (counts - counts_at_zero) / (counts_min - counts_at_zero);
      }

      rudder_position = - rudder_position;  // reverse direction of positive rudder position for Jeff's setup.  
       
       
        #if RUDDER_SENSOR == 1
              rudder_position = rudder_position - rcal;
        #endif


  }  // END VOID RUDDER POSITION
#endif
  // ----------------------  END RUDDER POSITION --------------------
    

 
 //------------- RUDDER CONTROLS --------------------------------------------------


  void Rudder_Stop()
 {
    #if Motor_Controller != 3  
      //Serial_MotorControl.write(Motor_1_fwd); //  for Qik 141.  set motor 1 forward for Trex(193)
      //Serial_MotorControl.write(0); // set speed = 0
      motor.stop();
   #endif

   #if Motor_Controller == 3 
ledcWrite(L_PWM, 0);    // changing to ledcwrite for ESP32 PWM
ledcWrite(R_PWM, 0);
   #endif  

  //  rudder_stop_time = millis();
      rudder_on = false; 
      rudder_was_off = true;    
     //if(Print_Motor_Commands)
     //{  Serial.print ("Motor Code, motorspeed ");
     //   Serial.print(Motor_1_fwd);Serial.print(", "); Serial.println(0); 
    // }
 }  // end Rudder_Stop
 
 
  void Left_Rudder()
  {
     #if Motor_Controller != 3
      //Serial_MotorControl.write(Motor_1_rev);  //   set motor 1 in reverse, 143 for Qik, 194 for TREX
      //Serial_MotorControl.write(motorspeed); // set speed = motorspeed 0 to 127 is 0% to 100%
      motor.backward();
     #endif
     
     #if Motor_Controller == 3   // IBT-2 selected

ledcWrite(R_PWM, 0);
ledcWrite(L_PWM, motorspeed);    // changing to ledcwrite for ESP32 PWM

     #endif
     
   // if(Print_Motor_Commands)
    // {  Serial.print ("Rudder Command Motor Code, motorspeed ");
      //  Serial.print(rudder_command); Serial.print(", "); Serial.print(Motor_1_rev);Serial.print(", "); Serial.println(motorspeed);
     //}
      rudder_on = true; //used in rudder position
      if (rudder_was_off)
          {
            rudder_time_old = millis();
            rudder_was_off = false;
          }
   // } 
  } // end Left_Rudder()
// --------------------------------------- 

  void Right_Rudder()
  {  
    
     #if Motor_Controller != 3
      motor.forward();
     #endif
     
     #if Motor_Controller == 3   // IBT-2 selected

ledcWrite(L_PWM, 0);    // changing to ledcwrite for ESP32 PWM
ledcWrite(R_PWM, motorspeed);

     #endif
    
        rudder_on = true; //used in rudder position
        if (rudder_was_off)
          {
            rudder_time_old = millis();
            rudder_was_off = false;
          }
        //if(Print_Motor_Commands)
        // {   Serial.print ("Rudder Command Motor Code, motorspeed ");
        //Serial.print(rudder_command); Serial.print(", "); Serial.print(Motor_1_fwd);Serial.print(", "); Serial.println(motorspeed);
         //}  
    //}  // end if rudder < rudder MAX  
  } // end Right_Rudder
  

    /***********************************************************************/    

 /*************************************************************************************/   
 
void rudder_sensor_calibration(){
     float rudder_position_max = 45;
     float rudder_position_min = -45;
     float counts_max = 1500;  // from calibration in print statement
     //float counts_at_zero = 415;
          float counts_at_zero = 750;
     float counts_min = 0;
     float counts;


 counts = analogRead(Rudder_Pin);   


      if(counts >= counts_at_zero) // linear calibration from zero
      {
          rudder_position = rudder_position_max *(counts - counts_at_zero) / (counts_max - counts_at_zero);
      }
      else
      {
          rudder_position = rudder_position_min * (counts - counts_at_zero) / (counts_min - counts_at_zero);
      }

      rcal = - rudder_position;  // reverse direction of positive rudder position for Jeff's setup.  

}
