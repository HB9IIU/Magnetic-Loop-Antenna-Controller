#include <BluetoothSerial.h>  // Include the Bluetooth library
#include <Arduino.h>  // Include the Bluetooth library

BluetoothSerial SerialBT;  // Create a BluetoothSerial object

void setup() {
  Serial.begin(115200);  // Start the serial monitor
  SerialBT.begin("ESP32MLA");  // Start Bluetooth SPP server with a name

  Serial.println("Bluetooth device is ready to pair");
}

void loop() {
  static String receivedMessage = "";  // Buffer to store the incoming message

  if (SerialBT.available()) {  // If there's data received via Bluetooth
    char received = SerialBT.read();  // Read the data
    Serial.print(received);  // Print the data on the Serial Monitor
    receivedMessage += received;  // Add the received character to the message

    // If the end of the message is detected (e.g., newline or carriage return)
    if (received == '\n') {  
      // Send the entire message back
      String response = "Received: " + receivedMessage;
      SerialBT.write((const uint8_t*)response.c_str(), response.length());  // Send the full response
      Serial.println("Response sent.");

      // Clear the received message buffer for the next message
      receivedMessage = "";
    }
  }

  if (Serial.available()) {  // If there's data from Serial (e.g., from Raspberry Pi)
    char data = Serial.read();
    SerialBT.write(data);  // Send the data over Bluetooth
  }
}
