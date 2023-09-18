/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// Import required libraries

void readheading()
{
  StaticJsonDocument<100> doc;
  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["heading"] = String(heading,0) ;
  object["rudder_position"] = String(rudder_position,0) ;
  serializeJson(doc, jsonString); // serialize the object and save teh result to teh string variable.
  Serial.println( jsonString ); // print the string for debugging.
  ws.textAll(jsonString); // send the JSON object through the websocket
  jsonString = ""; // clear the String.
}

/*
void readheading() {           // creates the data packet that is sent to the HTML every X seconds (set in HTML code on main page)
String headingstring = String(heading, 0);    
ws.textAll(headingstring);
}
*/

void notifyClients() {              // toggles the AP ON/OFF button visualization in the HTML
  ws.textAll(String(Steering_Mode));
//  ws.textAll(String(heading));

}

// remote control commands from HTML interface websocket inputs to ESP32 
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;

        if (strcmp((char*)data, "toggleAP") == 0) {
     toggleAP();  
    }

       else if (strcmp((char*)data, "subten") == 0) {
     remotesub10();  
    }

           else if (strcmp((char*)data, "addten") == 0) {
     remoteadd10();  
    }

           else if (strcmp((char*)data, "sub1") == 0) {
     remotesub1();  
    }

           else if (strcmp((char*)data, "add1") == 0) {
     remoteadd1();  
    }

           else if (strcmp((char*)data, "subninety") == 0) {
     remotesub90();  
    }

           else if (strcmp((char*)data, "addninety") == 0) {
     remoteadd90();  
    }

           else if (strcmp((char*)data, "lcdlightswitch") == 0) {
     lcdlightswitch();  
    }

               else if (strcmp((char*)data, "dodgeleft") == 0) {
     dodgeleft();  
    }

               else if (strcmp((char*)data, "dodgeright") == 0) {
     dodgeright();  
    }

               else if (strcmp((char*)data, "readheading") == 0) {
     readheading();
    }

               else if (strcmp((char*)data, "Rup") == 0) {
     Rup();
    }

               else if (strcmp((char*)data, "Rdown") == 0) {
     Rdown();
    }

               else if (strcmp((char*)data, "Gup") == 0) {
     Gup();
    }

               else if (strcmp((char*)data, "Gdown") == 0) {
     Gdown();
    }

               else if (strcmp((char*)data, "Pup") == 0) {
     Pup();
    }

               else if (strcmp((char*)data, "Pdown") == 0) {
     Pdown();
    }

               else if (strcmp((char*)data, "Iup") == 0) {
     Iup();
    }

               else if (strcmp((char*)data, "Idown") == 0) {
     Idown();
    }

               else if (strcmp((char*)data, "Dup") == 0) {
     Dup();
    }

               else if (strcmp((char*)data, "Ddown") == 0) {
     Ddown();
    }

               else if (strcmp((char*)data, "magvarup") == 0) {
     magvarup();
    }

               else if (strcmp((char*)data, "magvardown") == 0) {
     magvardown();
    }

    }
}



void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
   case WS_EVT_DATA:
     handleWebSocketMessage(arg, data, len);      // this handles incoming sockets from the HTML and links to the remote control commands above.
     break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}


// Sends heading and status information to the HTML as part of the processor function in the HTTP_Get call
String processor(const String& var){
  Serial.println(var);
  
  if(var == "STATE"){
    if (Steering_Mode == 1){        // this actually works to report the state on the phone
      return "ON";
    }
    else{
      return "OFF";
    }
  }

else if(var == "HEAD") return String(heading, 0);
        else if(var == "HTS") return String(heading_to_steer, 0);
        else if(var == "RUDD") return String(rudder_position, 0);
        else if(var == "DEADBAND") return String(deadband, 1);
    else if(var == "KOVERALL") return String(K_overall, 1);
    else if(var == "KHEAD") return String(K_heading, 1);
    else if(var == "KDIFF") return String(K_differential, 1);
    else if(var == "KINTEGRAL") return String(K_integral, 4);
        else if(var == "MAGVAR") return String(MagVar_default, 1);
    
return String();


}





