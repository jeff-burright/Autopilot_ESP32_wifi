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
     lcd.print("HDG        ");
     lcd.setCursor(4, 1);
     lcd.print(heading,0);
      

     if(Steering_Mode != 4)
     {   
       lcd.setCursor(8, 1);   
       lcd.print("HTS      ");
       lcd.setCursor(13, 1);
       lcd.print(heading_to_steer,1);

     }
     
    
     if( RUDDER_MODE == 0)  // IF THERE IS A RUDDER POSITION INDICATOR
    {
     lcd.setCursor(5,3);
     lcd.print("Rud    "); // extra spaces clear old data
     lcd.setCursor(9,3);
     lcd.print(rudder_position,0);
    }
   
 //   if(MSG >0) // MSG  0 is null otherwise print message here
 //   {



  

 
}  // END Void LCD()
