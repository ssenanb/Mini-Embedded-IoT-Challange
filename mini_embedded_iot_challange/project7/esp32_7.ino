#include "WiFi.h"
#include <HardwareSerial.h> //to use many UART 
#include <ArduinoJson.h> //to parse json data
#include <WebServer.h>

#define RX 16
#define TX 17

HardwareSerial mySerial(2); // second UART on ESP32

const char* ssid = "TP-LINK_3AA6";
const char* password = "17934244";

WebServer server(80);  // 80 port web server 

String incoming = ""; // raw data from the STM32
String sensorData = "{}"; // display on the HTML 

// send JSON data when the request comes
void handleData() {
 if(sensorData.length() > 0)
    server.send(200, "text/plain", sensorData);
  else
    server.send(204, "text/plain", ""); // 204 No Content -> if sensorData is empty
}
void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Canlı Sensor Verisi</title>
      <meta charset="utf-8">
      <script>
        function fetchData() {
          fetch("/data")
            .then(response => {
               if (response.status === 200) return response.text();
              else return "";
            })
            .then(text => {
              if (text !== "") {
              let output = document.getElementById("output");
              output.innerHTML += text + "<br>";
              }
            });
        }
        setInterval(fetchData, 1000); 
      </script>
    </head>
    <body>
      <h2>Live Sensor Data</h2>
      <div id="output" style="font-family: monospace;"></div>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

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

   // Web server route define
  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.begin();  // start server
  Serial.println("HTTP SERVER STARTED!");
}

void loop() {
  server.handleClient();  // listen to web server clients

  while (mySerial.available()) {
    char ch = mySerial.read();
    if (ch == '\n') {
      // parse JSON string
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, incoming);
      if (error) {
        Serial.println("Parse error");
      } else {
        String time = doc["time"];
        float distance  = doc["distance"];
       
        Serial.print("Time: "); Serial.println(time);
        Serial.print("Distance: "); Serial.println(distance);

        // only send data the above threshold
         if(distance > 20){
           sensorData = "Time: " + time + " - Distance: " + String(distance) + " cm";
        } else {
            sensorData = "";  // clear
        }
      }
      incoming = ""; // string reset
    } else {
      incoming += ch;
    }
  }
}
