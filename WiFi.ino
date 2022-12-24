/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-websocket-server-arduino/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

// Import required libraries



void notifyClients() {
  ws.textAll(String(Steering_Mode));
}

// remote control commands from HTML interface websocket inputs to ESP32 
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;

        if (strcmp((char*)data, "toggleAP") == 0) {
     //Steering_Mode = !Steering_Mode;
     toggleAP();  
      notifyClients();
    }

       else if (strcmp((char*)data, "subten") == 0) {
     //Steering_Mode = !Steering_Mode;
     remotesub10();  
      notifyClients();
    }

           else if (strcmp((char*)data, "addten") == 0) {
     //Steering_Mode = !Steering_Mode;
     remoteadd10();  
      notifyClients();
    }

           else if (strcmp((char*)data, "sub1") == 0) {
     //Steering_Mode = !Steering_Mode;
     remotesub1();  
      notifyClients();
    }

           else if (strcmp((char*)data, "add1") == 0) {
     //Steering_Mode = !Steering_Mode;
     remoteadd1();  
      notifyClients();
    }

           else if (strcmp((char*)data, "subninety") == 0) {
     //Steering_Mode = !Steering_Mode;
     remotesub90();  
      notifyClients();
    }

           else if (strcmp((char*)data, "addninety") == 0) {
     //Steering_Mode = !Steering_Mode;
     remoteadd90();  
      notifyClients();
    }

           else if (strcmp((char*)data, "lcdlightswitch") == 0) {
     //Steering_Mode = !Steering_Mode;
     lcdlightswitch();  
      notifyClients();
    }

  else {notifyClients();}

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
    
return String();


}





