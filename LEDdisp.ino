/*
void LEDstart(){



}


void LEDdisp(){


       display.setTextSize(1);
           display.setTextColor(SSD1306_WHITE);
           display.setCursor(0, 0);
           display.println("R = " + String(rudder_position, 2));  // Display R value
       
           display.setCursor(0, 16);  // Move to the next row
           display.println("C = " + String(heading_to_steer));  // Display C value
       
           display.display();
           delay(500);  // Update every second (adjust as needed)
           display.clearDisplay();


}

*/