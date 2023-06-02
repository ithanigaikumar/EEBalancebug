// ----------------------------------------  CORE ASSIGNMENT HANDLED USING freeRTOS  ---------------------------------------
// I will use FreeRTOS tasks (similar to threads) to manage concurrency -> don't need to manually assign tasks to specific cores
// Using 'xTaskCreate()' function -> don't need to specify which core task should run on, FreeRTOS handles this
//simplifies code making it more portable since we don't need to hardcode specific core assignments
// ---------------------------------------- freeRTOS tasks for Emre's section of ESP32 code --------------------------------
// Two main tasks -> 1) Handling Wi-Fi connections 2) Managing SerialPort Communications
// hexData variable is a shared resource so introduce a mutex (synchronisation mechanism) to deal with this

// ----------------------------------------- TO DO ---------------------
// Figure out the appropriate stack size and priority levels for the tasks

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> 
#include <Wire.h>
#include <freertos/FreeRTOS.h>  // FreeRTOS library
#include <freertos/task.h>      // FreeTROS tasks
#include <freertos/semphr.h>    // Mutex -> synchronisation mechanism

const char* ssid = "6gfL Hyperoptic 1Gb Fibre 2.4Ghz"; //defines wi-fi credentials to connect board to a specified network
const char* password = "ftC0nV2Ux3t";
AsyncWebServer server(80); //creates an instance to handle HTTP requests on port 80
HardwareSerial SerialPort(0);
String hexData = ""; //used to store converted hex data
SemaphoreHandle_t hexDataMutex; // mutex to protect shared resource `hexData`

//wi-fi connectivity
void WiFiTask(void* param) { 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(1000/portTICK_PERIOD_MS);   //vTaskDelay() is a function used to yield control back to the FreeRTOS scheduler so it can switch to other tasks
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());

  // configures AsyncWebServer to handle HTTP GET request to "/img" endpoint
  // when request made to this endpoint, it executes the lambda function
  // inside lambda function it protects access to 'hexData' using the semaphore
  server.on("/img", HTTP_GET, [](AsyncWebServerRequest *request){
    // protect hexData access
    xSemaphoreTake(hexDataMutex, portMAX_DELAY);
    request->send(200, "text/plain", hexData);
    xSemaphoreGive(hexDataMutex);
  });

  // starts the 'AsyncWebServer' enabling it to handle incoming HTTP requests
  server.begin();

   // infinite loop runs the task forever, introduces a delay of 1 sec to give other tasks a chance to execute
   //calculates delay as 1000 millisecs / constant value (duration of a single tick specific to port/platform used)
   //portTICK_PERIOD_MS defined by FreeRTOS during initialisation
  for(;;) { 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


//Reads data frim SerialPort and converts to a hexadecimal string
//Checks for available data in serial buffer
//Protects access to 'hexData' using semaphore 'hexdataMutex', assigns converted hex string to 'hexData' and relases semaphore
//Introduces a delay of 10 millisecs to allow other tasks to run
void SerialTask(void* param) {
  // task should never return, thus loop forever here
  for(;;) {
    if(SerialPort.available() > 0){
      uint8_t buffer[4];
      SerialPort.readBytes(buffer, 4);
      char hexBuffer[9]; // one extra for null terminator
      sprintf(hexBuffer, "%02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
      
      // protect hexData access
      xSemaphoreTake(hexDataMutex, portMAX_DELAY);
      hexData = String(hexBuffer);
      xSemaphoreGive(hexDataMutex);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // give some time for other tasks
  }
}


//setup function called once ESP32 starts
void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, 3, 1);

  // create mutex
  hexDataMutex = xSemaphoreCreateMutex();


 // check if the mutex was created successfully
  if(hexDataMutex == NULL) {
    Serial.println("Failed to create mutex. Halting setup.");
    return;
  }

  // create the tasks
  // STACK SIZES are assigned as 10000 bytes and PRIORITY LEVEL 2 for now CAN CHANGE!!!!!!!!!??????
  xTaskCreate(WiFiTask, "WiFiTask", 10000, NULL, 2, NULL);
  xTaskCreate(SerialTask, "SerialTask", 10000, NULL, 2, NULL);
}


// empty as main program logic is implemented in tasks
void loop() {
  // leave empty
}


