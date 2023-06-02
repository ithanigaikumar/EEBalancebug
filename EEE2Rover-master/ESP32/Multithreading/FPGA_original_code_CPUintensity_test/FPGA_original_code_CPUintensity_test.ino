// --------------------------- CPU INTENSITY OF SINGLE THREADED FPGA CODE ----------------------------
// want to be able to compare if threading makes any improvements to the code so need to test cpu intensity here as well
// accessing CPU intensity directly in a single threaded context like arduino is harder as there aren't native tools available for monitoring
// This code estimates the time spent executing particular functions or blocks of code by using micro-second resolution timers

//micros() gets the current time in microseconds at the start and end of the loop. Difference between these times = execution time of loop
//limited method -> can't account for any background tasks that might be running on microcontroller

// -----------------------------  EXPECTED OUTPUT EXAMPLE/RESULT ANALYSIS ----------------------------------------
// Loop execution time (microseconds): 123
// Loop execution time (microseconds): 115 
// .....
// Average loop execution time (microsecond): 120
// the numbers show the time it takes to execute one pass of the loop() function (time taken to check if data is available on serial port, read data and convert to hex string)
// average calculated every 100 loops



#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> 
#include <Wire.h>

const char* ssid = "6gfL Hyperoptic 1Gb Fibre 2.4Ghz"; //defines wi-fi credentials to connect board to a specified network
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

unsigned long total_time = 0;
unsigned long loop_count = 0;

void loop() {
  unsigned long start_time = micros();  // Store the start time
  if(SerialPort.available() > 0){
    uint8_t buffer[4];
    SerialPort.readBytes(buffer, 4);
    char hexBuffer[9]; // one extra for null terminator
    sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
    hexData = String(hexBuffer);
  }
  unsigned long end_time = micros();  // Store the end time
  
  // Output the elapsed time to the Serial Monitor
  Serial.print("Loop execution time (microseconds): ");
  Serial.println(end_time - start_time);


  // print average every 100 loops
  unsigned long execution_time = end_time - start_time;
  total_time += execution_time;
  loop_count++;

  if (loop_count % 100 == 0) {
    Serial.print("Average loop execution time (microseconds): ");
    Serial.println(total_time / loop_count); 
  }

}
