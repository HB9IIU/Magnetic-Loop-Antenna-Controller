#include <WiFi.h>
#include <HTTPClient.h>

// Wi-Fi credentials
const char* ssid = "MESH";
const char* password = "Nestle2010Nestle";

// Raspberry Pi endpoint
const char* serverURL = "http://pi4.local:5000/vfo";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Connect to the server
    http.begin(serverURL);

    // Send a GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) { // If successful
      String response = http.getString(); // Get the response payload
      Serial.println("VFO Frequency: " + response);
    } else {
      Serial.println("Error in HTTP request: " + String(httpResponseCode));
    }

    http.end();
  } else {
    Serial.println("Wi-Fi not connected!");
  }

  delay(20); // Query every second
}
