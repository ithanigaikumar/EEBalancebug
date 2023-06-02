#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <LinkedList.h>

const char* ssid = "6gfL Hyperoptic 1Gb Fibre 2.4Ghz";
const char* password = "";
AsyncWebServer server(80);

HardwareSerial SerialPort(0);

LinkedList<String> frameQueue = LinkedList<String>();
String tempData = "";

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
    if(frameQueue.size() > 0){
        String frameData = frameQueue.shift(); // get and remove the first frame data
        request->send(200, "text/plain", frameData);
    }
    else{
        request->send(200, "text/plain", " ");
    }
  });
  server.begin();
}

void loop() {
  if(SerialPort.available() > 0){
    uint8_t buffer[4];
    SerialPort.readBytes(buffer, 4);
    char hexBuffer[9]; // one extra for null terminator
    sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
    
    // If MSB of first byte is 1, it's a new frame
    if (buffer[0] & 0x80) {
        // If we already have some frame data, store it and start new frame
        if(tempData.length() > 0){
            frameQueue.add(tempData); // Add to queue
            tempData = String(hexBuffer); // start new frame
        }
        else {
            tempData += String(hexBuffer); // first frame
        }
    } else {
        tempData += " " + String(hexBuffer); // add to current frame
    }
  }
}