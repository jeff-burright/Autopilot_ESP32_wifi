#if Tiller == 1 

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
          //deadband = 2.0; 
 
         RUDDER_POSITION(); // 9.22.17 added to update rudder position and stop rudder in Dodge Mode but good for all modes in V14.7
         if(abs(rudder_position) > Maximum_Rudder)  {
           Rudder_Stop();
         }
         
        if(!DODGE_MODE) // if anything other than dodge mode, i.e., if steering
        { 
           // if keypad "1" was pushed Steering_Mode = 1 (compass steer) and heading_to_steer was set to the then current heading  
          // MSG = 0; // null

        heading_error = heading_to_steer - heading;  // This is the main PID proportional term for compass based steering           
        
           // Serial.print("Wind Error "); Serial.print(wind_error); Serial.print("; heading to steer "); Serial.println(heading_to_steer);            
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
          //NOTE if Compass == 1 (BNO055) bearing rate is calculated in that tab.  if other compasses added need to be sure bearing rate set by this point
          // Serial.print(heading_error);
          // Serial.print("  ");
          // Serial.println(differential_error);          
                        
          #if PID_MODE == 0          
            PID_output = PID_Ks[0] * (PID_Ks[1] * heading_error  - PID_Ks[2] * differential_error); 
            rudder_command = PID_output + Rudder_Offset; // rudder offset set when key 1 or 3 pressed captures rudderoffset only used when User input RUDDER_OFFSET == 1
          #endif
          
          #if PID_MODE == 1   
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
           // if(abs(heading_error) < 10) TACK_ON = false;  // used to limit tack rate probably not needed
          //  if(TACK_ON) rudder_MAX = Tack_rudder_MAX; // 
          #endif
          

          rudder_MAX = Maximum_Rudder;
         
  //        if(new_waypoint) rudder_MAX = new_waypoint_rudder_MAX;
         // rudder_command=constrain(rudder_command,-rudder_MAX,rudder_MAX);
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
         int motorspeed_min = 150; // no longer used 5/23/24
         float Rudder_Power_coeff = 0;  //  Set to 0 to not use. Use .5 for default starting point.applies more motor speed proportional to rudder position to have more force to increase rudder at
             // bigger rudder angles to counter weather helm. At bigger rudder positions it takes more force to increase rudder
              
          RUDDER_POSITION();// update rudder position  
          if (RUDDER_MODE ==1) rudder_position = 0; // rudder feed back RFB not availabl  
          rudder_error = rudder_command - rudder_position;
          
          if(Steering_Mode == 0 || !sw1 || !sw2)  // sw1 and sw2 need to be on for automated steering
         {
            Steering = false;
            //Rudder_Stop();
       //   backdrive();  // experimental
          
         }
        
        
       // if(Steering_Mode == 1 || Steering_Mode == 2 || Steering_Mode ==3 || Steering_Mode == 5) Steering = true; // could use if Steering_Mode >0
       if(Steering_Mode >0) Steering = true;
       // if(DODGE_MODE) Steering = false; //  if keypad LEFT or RIGHT RUDDER skip PID rudder control
            
        if(!DODGE_MODE) // do not steer if in dodge mode
        {
          if(rudder_on)  RUDDER_POSITION(); //if rudder on up date position      
          if(Steering)
          {   

              //motorspeed variable tells motor controller PWM how high to be 0-255.
        motorspeed = motorspeedMAX/ 30 *rudder_error; // where at 30 degree rudder error speed = MAX. 
                motorspeed = abs(motorspeed); 
        if (motorspeed < motorspeedMIN) motorspeed = motorspeedMIN;
 

         #if Rudder_Power_coeff > 0    //see main page for description of rudder power coefficient
          if(abs(rudder_command) > abs(rudder_position))
          {
            motorspeed = motorspeedMIN + float(motorspeedMAX-motorspeedMIN) * ((abs(rudder_position)/Maximum_Rudder) * Rudder_Power_coeff);  //  User define in this void 
          }
         #endif
        motorspeed = constrain(motorspeed, motorspeedMIN, motorspeedMAX);
       // Serial.print("motor speed ");Serial.println(motorspeed);     
        

            
/* This block treplaced as above to generalize it in terms of motorspeed max and Min to accomodate different controllers
         motorspeed = 128/30*rudder_error;  // rudder error of 30 deg will result in full speed on rudder
        //  motorspeed = 128.0/40.0*(rudder_command); // + = right, - = left
         if( motorspeed > 127) motorspeed = 127;
         if ( motorspeed < -127) motorspeed = -127;
         motorspeed= abs(motorspeed);
         if (motorspeed < motorspeed_min) motorspeed = motorspeed_min;
        // TACK_ON is set true for steering mode 3 when key 4 or 6 is pushed.  this sets max rudder and rudder speed to tack values 
        // when regular rudder command is less than deadband + 1, TACK_ON is turned off to revert to regular steering
 */
// For IBT-2 Controller voltage-based wheel stop. EXPERIMENTAL. Reads CS pin fed to IS pins on IBT-2 and compares to current limit
 // current = current*0.99+analogRead(CS)*0.01;
  // end wheel stop

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

//-------------------------   WHEEL STOP USING CURRENT SENSE IN IBT-2   -----------------------------------

  

    

//-------------------------   END WHEEL STOP -------------------------------------------------



            } // end  if Steering 
        }  // end if(!DODGE_MODE)          
        }  // void rudder control

      




  //------------------------  RUDDER POSITION  -----------------------

  void RUDDER_POSITION()
  {
     float rudder_position_max = 45;
     float rudder_position_min = -45;
     float counts_max = 827;  // from calibration in print statement
     float counts_at_zero = 415;
     float counts_min = 0;
     float counts;
     
     counts = analogRead(Rudder_Pin);  // rudder potentiometer pin input
     //Serial.print("Rudder pin = "); // use these print lines to get counts for calibration
     //Serial.println(counts);

      if(counts >= counts_at_zero) // linear calibration from zero
      {
          rudder_position = rudder_position_max *(counts - counts_at_zero) / (counts_max - counts_at_zero);
      }
      else
      {
          rudder_position = rudder_position_min * (counts - counts_at_zero) / (counts_min - counts_at_zero);
      }


      rudder_position = - rudder_position;  // reverse direction of positive rudder position for Jeff's setup. 
    
    // rudder_position =map(rudder_position, 187,910,-45,45); 
     
     //Serial.print("rudder angle, "); Serial.println(rudder_position);
    
  }  // END VOID RUDDER POSITION
   


   /*  OLD RUDDER POSITION USING TIMING 
    float rudder_rate = .015; // deg/millisec
    int rudder_delta_time;
    int rudder_time;
    static float rudder_change; 
    
    if(rudder_on)
    {
      rudder_time = millis();
      rudder_delta_time = rudder_time - rudder_time_old;
      rudder_total_time = rudder_total_time + float(rudder_delta_time)/1000;  // diagnostic to see how much rudder motor is on can be commented out
      rudder_change = float(rudder_delta_time) * rudder_rate * rudder_direction; //time in milliseconds, change in deg 
      rudder_time_old = rudder_time;
      rudder_position = rudder_position + rudder_change;
    }
     if(abs(rudder_position) > rudder_MAX) Rudder_Stop();
   */  // END OLD RUDDER ROSITION USING TIMING  
    
   
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
    //   motorspeed = 0;
      // Serial_MotorControl.write(Motor_1_fwd);
       //Serial_MotorControl.write(motorspeed & 0x1F);
       //Serial_MotorControl.write(motorspeed >> 5);
      // motor.stop();  // L298 commented out

//analogWrite(L_PWM, 0); //  IBT-2 direct override
//analogWrite(R_PWM, 0);

ledcWrite(7, 0);    // changing to ledcwrite for ESP32 PWM
ledcWrite(8, 0);

//motorController.Stop();   // library may be bugged

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

//motorController.TurnLeft(motorspeed);

ledcWrite(8, 0);
ledcWrite(7, motorspeed);    // changing to ledcwrite for ESP32 PWM


// --------------voltage-based wheel stop - experimental
/*
if(end_left || current > max_current){     
      Serial.print(current);
      Serial.println(" current too high end_left reached");
      end_left = true;
      Rudder_Stop();
      return;
    }
     if(end_right && motorspeed>0)end_right = false; //if we were at the right end, and now turning left  again, than end_right is false; 
     Right_Rudder();
     */
// ------------------ end wheel stop
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

//motorController.TurnRight(motorspeed);

ledcWrite(7, 0);    // changing to ledcwrite for ESP32 PWM noise reduction
ledcWrite(8, motorspeed);



// voltage wheel stop
/*
if(end_right || current > max_current){
      Serial.println("current too high end_right reached");
      end_right = true;      
      Rudder_Stop();
      return;
    }
    if(end_left && motorspeed>0)end_left = false; 
Left_Rudder();
*/
// end voltage wheel stop

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
 
 
/*
void backdrive()
{
  analogRead(L_IS);
  analogRead(R_IS);

  #if L_IS != 0

  motorspeed = L_IS / 16
  Left_Rudder();

  #endif

  #if R_IS != 0

  motorspeed = R_IS / 16
  Right_Rudder();

#endif


}

*/

 
 /*************************************************************************************/
 /*
     void Tracking_Error()
  {
    /*  tracking error is difference between course over ground and compass heading.
        It's purpose is to compensate for lack of accuracy in heading sensor. It also
        corrects drift or anything else that causes a deviation betweeen the direction the boat 
        is travelling and the direction the compass says it is pointing.  The results are low pass
        filtered to provide a time averaged result that will not change abruptly wwhen going to a new course
        but recompute on a new course.
     */
     

/*  
           float tracking_error_LPF = .999; // tracking error updates when GPS updates about  once per second
           if(GPRMC_fix && SOG > 1)// only updates if valid fix  may want to add a lower speed limit
           {
             tracking_error = course - heading; // would like to use an average heading but need to figure out how so the 1  and 359 doesn't avg to 180
             
                if (abs(tracking_error) > 180) // this limits err0r to < 180 and makes turn short way on compass + right, - left
                  {
                     if(course > heading)  tracking_error = tracking_error - 360;
                     if(course < heading) tracking_error = 360 + tracking_error;
                  }
             
             AVG_tracking_error = tracking_error_LPF * AVG_tracking_error + (1 - tracking_error_LPF) * tracking_error;
           } // end if
         } // end Tracking_Error
   
 */ 
/********************************************************************************/


#endif