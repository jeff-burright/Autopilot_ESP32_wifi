  /*********************** PRINT LCD ******************************/
void LCD(){
    // set the cursor to column 0, line 1
    // (note: counting begins with 0):
    String RP;
    int UTC_seconds;
    //lcd.clear();
   
  
     lcd.setCursor(0,0);
     lcd.print("");
     lcd.setCursor(0,0);
     //if(Use_CTS)lcd.print(Waypoint_next);
     //else 

          
    // lcd.print(HDG) also prints in compass (Subs) for fast print rate, prints here for more stable LCD view

    // Jeff commented back in
    lcd.setCursor(0, 1);
     lcd.print("HDG    ");
     lcd.setCursor(4, 1);
     lcd.print(heading,0);
      

     if(Steering_Mode != 4)
     {   
       lcd.setCursor(9, 1);   
       lcd.print("CRS    ");
       lcd.setCursor(13, 1);
       lcd.print(heading_to_steer,0);

     }
     
    // Display whether the wheel is right or left of center

    if(rudder_position > 0)
{
  rudd_dir = "L";
}

else if (rudder_position < 0)
{
  rudd_dir = "R"; 
}

else
{
  rudd_dir = "C";
}

     if( RUDDER_MODE == 0)  // IF THERE IS A RUDDER POSITION INDICATOR
    {
     lcd.setCursor(9,0);
     lcd.print("RUD    "); // extra spaces clear old data
     lcd.setCursor(13,0);
     lcd.print(rudd_dir);
     lcd.setCursor(14,0);
     lcd.print(abs(rudder_position),0);
    }
   
 //   if(MSG >0) // MSG  0 is null otherwise print message here
 //   {



  

 
}  // END Void LCD()
