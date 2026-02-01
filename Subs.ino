#if Compass == 0
void Bearing_Rate() 
{
// JNE coding bearing rate used in PID <--- from original AP creator
  // compute bearing rate two ways 1 Dyaw/DT or 2.  Gyros xyz resolved to +Z  
 //float Kbr = .05; // 1/50 = 1 second smoothing
 
//----------------- Block Bearing Rate based on delta compass / delta time. may be commented out
 /* static float yawold; // used for bearing rate see DCM/void Bearing Rate
  static unsigned long yawtime;
  static unsigned long yawtimeold;
  static float bearingrate1;
  static float lowpass = 0.0; // set to 0 for no filtering
    yawtime = millis();
    bearingrate1 = lowpass*bearingrate1 + (1.0-lowpass)*ToDeg(1000*(yaw-yawold)/((yawtime -yawtimeold))); // low pass filter
   // this works but bearingrate from gyros seems better if BRT not zero reset after a warmup period seems temp sensitive
    yawold = yaw;  
    yawtimeold = yawtime;
    */
 // ------------------ End of Block Bearing Rate. can be commented out
 
   bearingrate = ToDeg((Gyro_Vector[2]*cos(roll) + Gyro_Vector[1]*sin(roll))*cos(pitch) - Gyro_Vector[0]*sin(pitch));
   
   #if REVERSEYAW == 1
   bearingrate = -bearingrate;  // test 5/31/24 to fix apparent issue in bench version
#endif
   
   #if BEARINGRATE_OFFSET == 1
     bearingrate = bearingrate + bearingrate_Offset; // bearingrate_Offset = - bearingrate when   
                                   // keys 1, 2, 3 maybe 22 pressed  reset to 0 in key 0
   #endif    
      // vertical axis bearing rate from gyros
   // bearing rate needs to be zero if boat is not turning otherwise it creates a bias in heading error
   // using bearing rate smoothed to see what long term average is (boat stationary) to maybe use as a bias correction
   // Deleted the bearing rate smoothing in V 6_2_2 10/17/13
  //bearingrate_smoothed = (1-Kbr) * bearingrate_smoothed + Kbr * bearingrate; // updates 50/sec, 3000/min, 10,000 = 3.3 min avg.
  //bearingrate =  bearingrate_smoothed; // should remove bias but during a turn the BR smoothed may get to big
  //bearingrate = bearingrate + bearingrate_correction;  // try to calibrate bearing rate to zero when staionary, adjust in USER INPUT
    // bearingrate = bearingrate1;  // to use bearingrate1
    // to print and compare bearig rate methods
   // Serial.print("bearing rate 1 and BR Gyros ");
   //Serial.print(bearingrate); 
   //Serial.print(" "); 
   // Serial.print("Bearing Rate smoothed, ");
   // Serial.println(bearingrate_smoothed);
  
}  // void Bearing_Rate
#endif
/***********************  COMPASS CORRECTION *********************************/

void JNE_AP_Compass_Correction()
{
#if Compass == 0 
  MAG_Heading_Degrees = ToDeg(MAG_Heading);  // coding below here is JNE
  // AVG_Heading = 0.9*AVG_Heading + .1*MAG_Heading ; // low pass filter
  //yaw = -yaw;

  #if REVERSEYAW == 1
  yaw = -yaw;
  #endif

   heading = ToDeg(yaw); // using YAW  use varible heading becuse that is what is used in PID. 
  // heading = compassheading; // using raw LSM303 library heading function
  //  heading = MAG_Heading_Degrees;  // using tilt compensated compass heading from Compass_Heading() function
   //heading = ToDeg(AVG_Heading);  // using low pass filtered compass heading
#endif
   
  // Magnetic_Variation = MagVar_default;
  // if (GPRMC_fix) Magnetic_Variation = MagVar;
   
  // Magnetic_Variation = 0;  // use this to read magnetic heading for calibration etc.
    
   heading = heading + Magnetic_Variation + IMU_Install_Correction;
   if(heading < 0) heading = 360 + heading; //already a minus, convert to 0-360
   if(heading > 360) heading = heading -360;  //these corrections need if calibration result runs over or under 360
  
  //  Compass_Calibration();  // interpolate compass variation table (or curve fit)

  // Jeff commented out the below as a test
   /*  if(Screen == 0){
       lcd.setCursor(0, 1);
       lcd.print("HDG    "); // extra spaces clear old data
       lcd.setCursor(4, 1);
       lcd.print(heading,0);
     } // end if screen = 0

     */
  // get delta T for PID integral error on heading error
  //compass_time =millis(); 
  //delta_compass_time = compass_time - compass_timeold;
 // compass_timeold = compass_time;

/*
 void Compass_Calibration() //Compass Calibration added by Jack Edwards
{
  float compass_correction;
//two approaches interpolation of error and curve fit to error

//INTERPOLATION
  float Compass_CAL[25] = {.1,-.6,-1.2,-1.6,-2,-2.3,-2.3,-2.7,-2.0,-2.0,-1.1,-.5,-.3,-.1,0,.6,-.2,.2,-.3,-.5,-.4,.1,-.1,-.4,-.4};
 index = (unsigned int)heading/15;  //integer part of heading/15
   compass_correction = Compass_CAL[index] + (heading/15 -index)*(Compass_CAL[index +1] -Compass_CAL[index]);
 
 // CURVE FIT TO ERROR curent best fit is two intersecting straight lines use excel to plot error and get curve fits
 
       /*if (heading <=79.2)
       {
        compass_correction =-.054*heading - .3298;
       }
        else
       {
         compass_correction =.0177*heading - 6.0007;
       }
       */
 
 /*
  heading= heading + compass_correction;
  if(heading < 0) heading = 360 + heading; //already a minus, convert to 0-360
  if(heading>360) heading = heading -360;  //these corrections need if calibration result runs over or under 360
  */
          /*
          lcd.setCursor(0,2);
          lcd.print("Correction");
          lcd.setCursor(12,2);
          lcd.print(compass_correction);
          */
  //}  // End Compass Calibration
  
}  // End JNE_AP_Compass_Correction()
  /******************************/

 
// compcalib() may be turned on/off on the main .ino tab. It enables compass min/max values to be displayed in the HTML interface while the boat is running, as an alternative to bench calibration using the example calibration sketch from the IMU library
void compcalib(){

  running_min.x = min(running_min.x, compass.m.x);
  running_min.y = min(running_min.y, compass.m.y);
  running_min.z = min(running_min.z, compass.m.z);

  running_max.x = max(running_max.x, compass.m.x);
  running_max.y = max(running_max.y, compass.m.y);
  running_max.z = max(running_max.z, compass.m.z);
  
  snprintf(report, sizeof(report), "min: {%+6d, %+6d, %+6d}    max: {%+6d, %+6d, %+6d}",
    running_min.x, running_min.y, running_min.z,
    running_max.x, running_max.y, running_max.z);
  //Serial.println(report);

}
