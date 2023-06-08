// ----------------------------------------  freeRTOS  ---------------------------------------
// I will use FreeRTOS tasks (similar to threads) to manage concurrency 
// Each task is assigned to a core
// ---------------------------------------- freeRTOS tasks for Emre's section of ESP32 code --------------------------------
// Two main tasks -> 1) Websocket Server Setup 2) Reading data from the serial port
// No shared resources between tasks -> reduces potential for concurrency related bugs
/* serverTask responsible for setting up the WAP & server. It does not use any global variables or objects that the serialReadTask also uses.
The serialReadTask reads from the serial port and sends data over the WebSocket. 
It uses the global variables SerialPort, ws, and tempData, but none of these are accessed by the serverTask.
*/
//polling is used within the func to check whether data is available on the serial port
//func polls 'serialport.available() until there are 4 or more bytes to read
    //if there is enough data it processes it, if not it delays for a short period
// ----------------------------------------- TO DO ---------------------
// Figure out the appropriate stack size for the tasks
// include the image broadcasting code here too?
// determine a suitable delay time for vTaskDelay


#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <FreeRTOS.h>

//defining SSID and password for wifi access point
const char* ssid = "esp32aptestrle"; 
const char* password = "wasdqwerty32";

//websocket and serial configuration
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance
HardwareSerial SerialPort(2);
String tempData = "";

//websocket events
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    // client connected
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if(type == WS_EVT_DISCONNECT){
    // client disconnected
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);

  // FreeRTOS task 1 -> Websocket server setup
  // assigns task to core 1
  xTaskCreatePinnedToCore(
    serverTask,      // Task function
    "ServerTask",    // Name of task
    50,           // Stack size of task MIGHT NEED TO CHANGE!!!!!!!!!!!!!!!!!1
    NULL,            // Parameter of the task
    1,               // Priority of the task
    NULL,            // Task handle to keep track of created task
    1);              // Core where the task should run
  
    //same setup for task 2 -> Reading data from serial port
    xTaskCreatePinnedToCore(
    serialReadTask,  
    "SerialReadTask", 
    200,  
    NULL, 
    1, 
    NULL, 
    1);             // Assigned to core 1
}

// TASK 1 CORE 1 -> Websocket Server setup
void serverTask(void * parameter) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  Serial.println(WiFi.softAPIP());
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();

  //vTaskDelay blocks the currently running task for a specified number of tick periods
  //Introduces a delay of 1 sec to give other tasks a chance to execute
  //calculates delay as 1000 millisecs / constant value (duration of a single tick specific to port/platform used)
  //portTICK_PERIOD_MS defined by FreeRTOS during initialisation
  for (;;) {
    vTaskDelay(500 / portTICK_PERIOD_MS); // task should never return, so we just put it to sleep
  }
}


//reading 4 bytes from serial port and converting into a hex string
//polling is used within the func to check whether data is available on the serial port
void serialReadTask(void * parameter) {
  while(1) { //always true so loops iterates indefinetly until program is stopped
    //'available' method of 'hardwareserial' is being polled to check if there is enough data to read 
    while(SerialPort.available() >= 4){
      uint8_t buffer[4];
      SerialPort.readBytes(buffer, 4);
      char hexBuffer[9]; // one extra for null terminator
      sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0]);
      if(buffer[3] & 0x80){
        if(tempData.length() > 2){
          ws.textAll(tempData);
          tempData = "";
        }
        tempData += String(hexBuffer);
      }
      else{
        tempData += String(hexBuffer);
      }
    }
    //func polls 'serialport.available() until there are 4 or more bytes to read
    //if there is enough data it processes it, if not it delays for a short period
    vTaskDelay(10 / portTICK_PERIOD_MS); // slight delay to avoid blocking other tasks
  }
}

void loop() {
  // no code is required in loop when using FreeRTOS, loop could be deleted
  // to save memory space, however, some libraries might still use loop for background tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
