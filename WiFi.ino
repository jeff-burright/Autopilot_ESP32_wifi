/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/


#if smartwatch == 1   //if using smartwatch control, this sets up the ESP-NOW communication

void espnowsetup(){

  if ( esp_now_init() != ESP_OK ) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  } 
}

// ESP-NOW Callback when data is sent
void OnDataSent(const uint8_t *broadcastAddress, esp_now_send_status_t status) {
//void OnDataSent(const uint8_t *broadcastAddress, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// ESP-NOW Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&APCommand, incomingData, sizeof(APCommand));
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  Serial.println(APCommand.doit);
incomingCommand = APCommand.doit;

espnowhandler();

 esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &incomingReadings, sizeof(incomingReadings));

}

void espnowhandler(){
 // #if ( requestHTS == 1 )
 
 //Serial.println(incomingCommand);

       switch(APCommand.doit){     
         case 1: //  Toggle   
      toggleAP();
      break;

         case 2: //  
      remoteadd1();
      break;

               case 3: //    
      remotesub1();
      break;

               case 4: //     
      remoteadd10();
      break;

               case 5: //      
      remotesub10();
      break;

      case 6:
      break;

      default: break;
       }

  //bounce back a response to the watch upon data receipt
//incomingReadings.HDG = 8;
HTSout = heading_to_steer;
incomingReadings.HTS = HTSout;   // need to make whole integer?
if (Steering_Mode == 1) incomingReadings.STATE = 1;
else incomingReadings.STATE = 0;

}
#endif  // end smartwatch ESP-NOW commands

void readheading()
{
  StaticJsonDocument<100> doc;
  // create an object
  JsonObject object = doc.to<JsonObject>();
  object["heading"] = String(heading,0) ;
  object["rudder_position"] = String(rudder_position,0) ;
  serializeJson(doc, jsonString); // serialize the object and save teh result to teh string variable.
  //Serial.println( jsonString ); // print the string for debugging.
  ws.textAll(jsonString); // send the JSON object through the websocket
  jsonString = ""; // clear the String.
}


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

               else if (strcmp((char*)data, "MMINup") == 0) {
     MMINup();
    }

               else if (strcmp((char*)data, "MMINdown") == 0) {
     MMINdown();
    }
    
               else if (strcmp((char*)data, "magvarup") == 0) {
     magvarup();
    }

               else if (strcmp((char*)data, "magvardown") == 0) {
     magvardown();
    }


               else if (strcmp((char*)data, "PID1") == 0) {
     PIDmode1();
    }

                   else if (strcmp((char*)data, "PID2") == 0) {
     PIDmode2();
    }

                   else if (strcmp((char*)data, "PID3") == 0) {
     PIDmode3();
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
        else if(var == "PIDMODE") return String(PID_MODE, 1);
        else if (var == "CALREP") return String(report);
        else if (var == "PIDREP") return String(pidreport);
        else if (var == "MMIN") return String(motorspeedMIN, 0);
return String();


}





