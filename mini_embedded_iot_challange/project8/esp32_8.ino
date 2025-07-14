#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// WiFi settings
#define WLAN_SSID       "TP-LINK_3AA6"
#define WLAN_PASS       "17934244"

// Adafruit IO settings
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "ssenanb"
#define AIO_KEY         "aio_SlPI36gFJrAu6pp0mlqH2QNkqIHR"

// UART settings
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial SerialSTM(2);  

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish stm32Data = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/stm32_data");
Adafruit_MQTT_Subscribe buzzerControl = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buzzer_control");

String incomingCommand = "";

void MQTT_connect();

void setup() {
  Serial.begin(115200);
  SerialSTM.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  MQTT_connect();
  mqtt.subscribe(&buzzerControl);
}

void loop() {
  MQTT_connect();

  // Check incoming MQTT messages
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(10))) {
    if (subscription == &buzzerControl) {
      incomingCommand = (char *)buzzerControl.lastread;
      incomingCommand.trim();
      Serial.print("Received command: '");
      Serial.print(incomingCommand);
      Serial.println("'");

      if (incomingCommand.equals("OFF")) {
        SerialSTM.print("OFF\r\n");  // send "OFF" command to the STM32
        Serial.println("Sent OFF command to STM32");
      }
    }
  }

  // read the incoming message from the STM32
  if (SerialSTM.available()) {
    String stmMessage = SerialSTM.readStringUntil('\n');
    stmMessage.trim();
      
    // if the message is not empty, publish it
    if (stmMessage.length() > 0) {
      Serial.print("STM32 message: ");
      Serial.println(stmMessage);

      if (!stm32Data.publish(stmMessage.c_str())) {
        Serial.println("Publish failed");
      } else {
        Serial.println("Publish success");
      }
    }
  }

  delay(100);
}

void MQTT_connect() {
  if (mqtt.connected()) return;

  Serial.print("Connecting to MQTT...");
  int8_t ret;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.println("MQTT Connected!");
}
