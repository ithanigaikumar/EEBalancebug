#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "esp32aptestrle";
const char* password = "wasdqwerty32";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance

HardwareSerial SerialPort(2);

TaskHandle_t Task1;

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
  xTaskCreatePinnedToCore(
        Task1code, /* Function to implement the task */
        "Task1", /* Name of the task */
        10000,  /* Stack size in words */
        NULL,  /* Task input parameter */
        0,  /* Priority of the task */
        &Task1,  /* Task handle. */
        0); /* Core where the task should run */
}

void Task1code( void * parameter) {
  String hexBuffer = "";
  // float dx;
  // float timeout_time; micros()*0.000001;
  for(;;) {
    while (SerialPort.available() < 1) { 
      SerialPort.write(0xEE); //flag to indicate to nios that esp is ready to receive frame
      delay(10);
    }
    uint8_t header = SerialPort.read();
    uint16_t header4x = header * 4;
    //Serial.println(header4x);


    // Read the frame based on the header
    while (SerialPort.available() < (header4x)) { delay(10); }
    uint8_t buffer[header4x];
    SerialPort.readBytes(buffer, header4x);

    // Convert the buffer to a hexadecimal string
    for (int i = header4x-1; i >= 0; i--) {
      char hex[3];
      sprintf(hex, "%02X", buffer[i]);
      hexBuffer += String(hex);
    }
    //Serial.println(hexBuffer);

    // Send the frame over WebSocket


    // timeout_time = micros()*0.000001;
    // while(sense_on && (micros()*0.000001-timeout_time) < (min_sense_period*3)){} //5.4ms maximum delay that can be added to image transmission
    // dx = x - x_prev;

    ws.textAll(hexBuffer);
    // x_prev = x;
    hexBuffer = "";
  }
}

void loop() { //by default this loop runs in core 1 so the controller code would go here while the image broadcasting is done on core 0
  delay(157);


}
