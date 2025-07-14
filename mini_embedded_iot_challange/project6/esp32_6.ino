#include "WiFi.h"
#include <HardwareSerial.h> //to use many UART 
#include <ArduinoJson.h> //to parse json data
#include <HTTPClient.h> //to send HTTP GET/POST requests

#define RX 16
#define TX 17

HardwareSerial mySerial(2); // second UART on ESP32

const char* ssid = "TP-LINK_3AA6";
const char* password = "17934244";

const char* server = "http://api.thingspeak.com";
const String apiKey = "W4LCL6XSC97GTND4";

void setup() {
  Serial.begin(115200); // to monitoring serial monitor
  mySerial.begin(9600, SERIAL_8N1, RX, TX); // to connect via UART with STM32
  
  // connect the wifi
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println("WiFi Connecting...");
  }
  Serial.println("CONNECTED!");
}

void loop() {
  static String incoming = "";

  while (mySerial.available()) {
    char ch = mySerial.read();
    if (ch == '\n') {
      Serial.print("Received JSON: ");
      Serial.println(incoming);

      // parse JSON string
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, incoming);
      if (error) {
        Serial.print("Parse failed: ");
        Serial.println(error.c_str());
      } else {
        float temp = doc["temp"];
        float hum  = doc["hum"];
        float ir   = doc["ir"];
       
        Serial.print("Temp: "); Serial.println(temp);
        Serial.print("Hum : "); Serial.println(hum);
        Serial.print("IR  : "); Serial.println(ir);

       // send HTTP GET
        if (WiFi.status() == WL_CONNECTED) {
          HTTPClient http;
          String url = String(server) + "/update?api_key=" + apiKey + 
                       "&field1=" + String(temp) + 
                       "&field2=" + String(hum) + 
                       "&field3=" + String(ir);

          http.begin(url);
          int httpResponseCode = http.GET();

          if (httpResponseCode > 0) { // if http response code is 200, the process succesful
            String payload = http.getString();
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            Serial.print("Payload: ");
            Serial.println(payload);
          } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
          }
          http.end();
        } else {
          Serial.println("NOT CONNECTED TO WIFI!");
        }
      }
      incoming = ""; // string reset
      delay(2000);  // send data every 2 seconds
    } else {
      incoming += ch;
    }
  }
}
