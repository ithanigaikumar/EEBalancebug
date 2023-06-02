#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "6gfL Hyperoptic 1Gb Fibre 2.4Ghz";
const char* password = "ftC0nV2Ux3t";
AsyncWebServer server(80);

HardwareSerial SerialPort(0);

String hexData = "";

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 3, 1);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  server.on("/img", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", hexData);
  });
  server.begin();
}

void loop() {
  if(SerialPort.available() > 0){
    uint8_t buffer[4];
    SerialPort.readBytes(buffer, 4);
    char hexBuffer[9]; // one extra for null terminator
    sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
    hexData = String(hexBuffer);
  }
}
