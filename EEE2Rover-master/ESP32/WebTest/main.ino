#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>


int l_en = 12;
int l_dir = 13;
int r_en = 8;
int r_dir = 9; //for motor controls
const int modRead = 1;
const int irRead = 3;
const int usRead = 4;
const char ssid[] = "Damani";
const char pass[] = "password";

//const char ssid[] = "Anthony's iPhone 11 Pro iOS 16";
//const char pass[] = "ejesjcoc43&/;pkndbs";
const int groupNumber = 5; // Set your group number to make the IP address constant - only do this on the EEERover network
WiFiWebServer server(80);

void imgUpdate(){

}


void handleRoot()
{
  server.send(200, F("text/html"), webpage);
}

void handleNotFound()
{
  String message = F("File Not Found"); 
  message += F("URI: ");
  message += server.uri();
  message += F("nMethod: ");
  message += (server.method() == HTTP_GET) ? F("GET") : F("POST");
  message += F("nArguments: ");
  message += server.args();
  message += F("n");
  for (uint8_t i = 0; i < server.args(); i++)
  {
    message += " " + server.argName(i) + ": " + server.arg(i) + "n";
  }
  server.send(404, F("text/plain"), message);
}

void motion(float l_in, float r_in) {
 
  if(l_in > 0){ //forward
    digitalWrite(l_dir, HIGH);
  }
  else if(l_in < 0){ //left
    digitalWrite(l_dir, LOW);
  }
  if(r_in > 0){ 
    digitalWrite(r_dir, HIGH);
    
    
  }
  else if(r_in < 0){ 
    digitalWrite(r_dir, LOW);
  }
 
  analogWrite(l_en, abs(l_in));
  analogWrite(r_en, abs(r_in));
  Serial.println(l_in);
  Serial.println(r_in);
}
void handleMotion(){
  if(server.arg("left")&& server.arg("right")){
    int right=(server.arg("right")).toInt();
    int left=(server.arg("left")).toInt();
     
     server.sendHeader("Access-Control-Allow-Origin", "*");
     motion(left,right);
     server.send(200, F("text/plain"), F("Motor Control Engaged"));
     Serial.println("Args Present");
  }
  else{
     server.send(404, F("text/plain"), F("Motor Control Error"));
     Serial.println("Arg Error");
  }
}

void sendImage(imgRLE){
  HTTPClient server;   
 
   http.begin("http://jsonplaceholder.typicode.com/posts/1");
   http.addHeader("Content-Type", "text/plain");            
 
   int httpResponseCode = http.PUT(imgRLE);   
 
   if(httpResponseCode>0){
 
    String response = http.getString();   
 
    Serial.println(httpResponseCode);
    Serial.println(response);          
 
   }else{
 
    Serial.print("Error on sending PUT Request: ");
    Serial.println(httpResponseCode);
 
   }
}

void setup()
{
  Serial.begin(9600);
  //Wait 10s for the serial connection before proceeding
  //This ensures you can see messages from startup() on the monitor
  //Remove this for faster startup when the USB host isn't attached
  while (!Serial && millis() < 10000);  

  Serial.println(F("nStarting Web Server"));

  //Check WiFi shield is present
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println(F("WiFi shield not present"));
    while (true);
  }

  //Configure the static IP address if group number is set
  if (groupNumber)
  //EERover IP
  //WiFi.config(IPAddress(192,168,0,groupNumber+1));

  //Damani Hotspot IP
  WiFi.config(IPAddress(192,168,43,groupNumber+1));

  // iphone hotspot 172.20.10.2
  //WiFi.config(IPAddress(172,20,10,groupNumber+1));

  // attempt to connect to WiFi network
  Serial.print(F("Connecting to WPA SSID: "));
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED)
  {
    delay(500);
    Serial.print('.');
  }
  
  
  //Register the callbacks to respond to HTTP requests
  server.on(F("/"), handleRoot);
  server.on(F("/img"), imgUpdate);
  server.on(F("/motion"),handleMotion);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.print(F("HTTP server started @ "));
  Serial.println(static_cast<IPAddress>(WiFi.localIP()));
  
}

//Call the server polling function in the main loop
void loop()
{
  server.handleClient();
}