#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "esp32aptestrle";
const char* password = "wasdqwerty32";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance

HardwareSerial SerialPort(2);

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
}

void loop() {
  if(SerialPort.available() >= 2){
    uint8_t buffer[16];
    SerialPort.readBytes(buffer, 16);
    char hexBuffer[33]; // one extra for null terminator
    sprintf(hexBuffer, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0], buffer[7], buffer[6], buffer[5], buffer[4], buffer[11], buffer[10], buffer[9], buffer[8], buffer[15], buffer[14], buffer[13], buffer[12]);
    String tempData = String(hexBuffer);
    ws.textAll(tempData);
  }
  ws.cleanupClients();
}
// void loop() {
//   if(SerialPort.available() > 0){
//     uint8_t buffer[16];
//     for(int i = 0; i < 16; i++){
//       while(SerialPort.available() == 0){}  // wait for the next byte to arrive
//       SerialPort.readBytes(buffer, 1);
//     }
//     char hexBuffer[33]; // one extra for null terminator
//     sprintf(hexBuffer, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0], buffer[7], buffer[6], buffer[5], buffer[4], buffer[11], buffer[10], buffer[9], buffer[8], buffer[15], buffer[14], buffer[13], buffer[12]);
//     String tempData = String(hexBuffer);
//     Serial.println(tempData);
//   }
// }
