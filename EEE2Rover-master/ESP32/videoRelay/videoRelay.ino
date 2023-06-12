#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "esp32aptestrle";
const char* password = "wasdqwerty32";
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // create WebSocket instance

HardwareSerial SerialPort(2);
// String tempData = "";
String hexBuffer = "";

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

// void loop() {
//   while(SerialPort.available() >= 4){
//     uint8_t buffer[4];
//     SerialPort.readBytes(buffer, 4);
//     char hexBuffer[9]; // one extra for null terminator
//     sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0]);
//     if(buffer[3] & 0x80){
//       // if(tempData.length() > 2){
//       ws.textAll(tempData);
//       tempData = "";
//       //}
//       tempData += String(hexBuffer);
//     }
//     else{
//       tempData += String(hexBuffer);
//     }
//   }
// }

void loop() {
  while (SerialPort.available() < 1) { 
    SerialPort.write(0xEE);
    delay(10);
  }
  uint8_t header = SerialPort.read();
  uint16_t header4x = header * 4;
  Serial.println(header4x);


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
  Serial.println(hexBuffer);

  // Send the frame over WebSocket
  ws.textAll(hexBuffer);
  hexBuffer = "";
}