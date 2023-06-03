#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>

const char* ssid = "6gfL Hyperoptic 1Gb Fibre 2.4Ghz";
const char* password = "";
AsyncWebServer server(80);

HardwareSerial SerialPort(2);

#define MAX_FRAMES 5000  // maximum number of frames to queue
String frameBuffer[MAX_FRAMES];  // buffer to hold complete frames
int front = 0;  // index of first frame
int rear = -1;  // index of last frame
int frameCount = 0;  // number of frames in the buffer
String currentFrame = "";  // currently building frame

void enqueueFrame(String frame) {
  if (frameCount < MAX_FRAMES) {
    rear = (rear + 1) % MAX_FRAMES;
    frameBuffer[rear] = frame;
    frameCount++;
  } else {
    // Buffer is full. Consider handling this situation.
    Serial.println("Buffer is full");
  }
}

String dequeueFrame() {
  if (frameCount > 0) {
    String frame = frameBuffer[front];
    front = (front + 1) % MAX_FRAMES;
    frameCount--;
    return frame;
  }
  return "";  // No data available
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  server.on("/img", HTTP_GET, [](AsyncWebServerRequest *request){
    if (frameCount > 0) {
      request->send(200, "text/plain", dequeueFrame());
    } else {
      request->send(200, "text/plain", "");
    }
  });
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  server.begin();
}

void loop() {
  if(SerialPort.available() > 0){
    uint8_t buffer[4];
    SerialPort.readBytes(buffer, 4);
    char hexBuffer[9]; // one extra for null terminator
    sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[3], buffer[2], buffer[1], buffer[0]);
    String tempData = String(hexBuffer);
    

    // If MSB of first byte is 1, it's a new frame
    if (buffer[3] & 0x80) {
        // If we already have some frame data, store it in the buffer
        if(currentFrame.length() > 0){
            enqueueFrame(currentFrame);
            
        }
        currentFrame = tempData; // start new frame
    } else {
        currentFrame += tempData; // add to current frame
    }
  }
}
