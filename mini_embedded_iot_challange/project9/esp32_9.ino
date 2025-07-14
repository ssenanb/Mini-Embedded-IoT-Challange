#include "BluetoothSerial.h"

HardwareSerial SerialSTM(2); // UART2 (

BluetoothSerial SerialBT; // Bluetooth serial connection

String rxData = "";
String cmd = "";

void setup() {
  Serial.begin(9600);         
  SerialSTM.begin(9600, SERIAL_8N1, 16, 17); // STM32 - UART connection
  SerialBT.begin("ESP32_Device"); // Bluetooth device name

  Serial.println("Bluetooth is started.");
}

void loop() {
  if (SerialSTM.available()) {
    String stmData = SerialSTM.readStringUntil('\n');
    SerialBT.println(stmData); // send via Bluetooth
    Serial.println("Data from the STM : " + stmData);
  }

  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c != '\n' && c != '\r') {
      cmd += c;
    } else {
      cmd.trim();
      Serial.println("Incoming command : " + cmd);
      
      if (cmd == "OFF") {
        SerialSTM.println("OFF"); // send "OFF" to the STM
      }
      
      cmd = ""; // reset command
    }
  }
}
