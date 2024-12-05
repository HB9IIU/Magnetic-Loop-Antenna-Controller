
#include <Arduino.h>
#include <TFT_eSPI.h> // Include the TFT_eSPI library

#include <WiFi.h>
#include <HTTPClient.h>

TFT_eSPI tft = TFT_eSPI(); // Initialize the TFT display

//_________________________________________________________________________________________
// Wi-Fi credentials
#define SSID "MESH"
#define PASS "Nestle2010Nestle"
// IPAddress slaveIP(192, 168, 4, 1); // IP address of the SLAVE device to send HTTP commands to
IPAddress yaesuCatServerIP(192, 168, 0, 221); // IP address of the SLAVE device to send HTTP commands to

HTTPClient http; // HTTPClient instance

const char *ssid = "MESH";

const char *password = "Nestle2010Nestle";

// Raspberry Pi endpoint
const char *serverURL = "http://pi4.local:5000/vfo";
//_________________________________________________________________________________________

void establish_WIFI_connection_with_Slave();
void getLinkQuality(int8_t rssi, String &description, uint16_t &color);
void printOnTFT(const char *text, uint16_t textColor, uint16_t backgroundColor); // to display console type info
void checkYaesuCATserver();

//_________________________________________________________________________________________

void setup()
{

  Serial.begin(115200);
  Serial.println("HB9IIU MLA Controller Starting");
  pinMode(TFT_BLP, OUTPUT);    // Configure the backlight pin
  digitalWrite(TFT_BLP, HIGH); // Turn on the backlight

  tft.begin();               // Initialize the TFT display
  tft.fillScreen(TFT_BLACK); // Set background to black
  tft.setRotation(1);        // Set rotation if needed

  establish_WIFI_connection_with_Slave();
  checkYaesuCATserver();
  // Initialize HTTPClient
  // String url = String("http://") + slaveIP.toString() + "/command"; // Ensure slaveIP is defined
  // http.begin(url);                                                  // Ensure http is defined and initialized
  // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
}

void loop()
{
  /*
  if (WiFi.status() == WL_CONNECTED)
  {

    // Connect to the server
    http.begin(serverURL);

    // Send a GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode == 200)
    {                                     // If successful
      String response = http.getString(); // Get the response payload
      Serial.println("VFO Frequency: " + response);
    }
    else
    {
      Serial.println("Error in HTTP request: " + String(httpResponseCode));
    }

    http.end();
  }
  else
  {
    Serial.println("Wi-Fi not connected!");
  }
*/
  delay(20); // Query every second
}

void establish_WIFI_connection_with_Slave()
{
  Serial.print("Establishing WiFi Connection with Slave...");
  printOnTFT("Connecting To Slave via WiFi...", TFT_WHITE, TFT_BLACK);

  // Attempt to connect to WiFi
  WiFi.begin(SSID, PASS);

  int attemptCounter = 0;
  String dots = "";

  while (WiFi.status() != WL_CONNECTED && attemptCounter < 15)
  {
    delay(500);
    dots += "*"; // Add a dot for each attempt
    Serial.print(".");
    attemptCounter++; // Increment the attempt counter
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    int8_t rssi = WiFi.RSSI(); // Get WiFi RSSI (signal strength)
    String qualityDescription;
    uint16_t qualityColor;
    getLinkQuality(rssi, qualityDescription, qualityColor);
    Serial.println("\nSuccessful Wifi Connection to Slave");
    Serial.print("Signal Quality:");
    Serial.println(qualityDescription.c_str());
    String message = "Link Quality: " + qualityDescription;
    printOnTFT(message.c_str(), qualityColor, TFT_BLACK);
  }
  else
  {
    Serial.println("\nConnection failed, rebooting in 3 seconds");
    printOnTFT("No WiFi Connection to Slave", TFT_WHITE, TFT_BLACK);
    printOnTFT("Please check your Slave", TFT_WHITE, TFT_BLACK);
    printOnTFT("Re-Booting in 3 seconds", TFT_WHITE, TFT_BLACK);
    delay(3000);
    ESP.restart();
  }
}

void getLinkQuality(int8_t rssi, String &description, uint16_t &color)
{
  if (rssi > -30)
  {
    description = "Amazing"; // Strong signal
    color = TFT_GREEN;       // Assign green color for amazing signal
  }
  else if (rssi > -67)
  {
    description = "Great"; // Good signal
    color = TFT_GREEN;     // Assign light green for great signal
  }
  else if (rssi > -70)
  {
    description = "Average"; // Fair signal
    color = TFT_YELLOW;      // Assign yellow for average signal
  }
  else if (rssi > -80)
  {
    description = "Poor"; // Weak signal
    color = TFT_ORANGE;   // Assign orange for poor signal
  }
  else if (rssi > -90)
  {
    description = "Unusable"; // Very weak signal
    color = TFT_RED;          // Assign red for unusable signal
  }
  else
  {
    description = "Unusable"; // Signal is too weak
    color = TFT_RED;          // Assign dark red for very weak signal
  }
}
void printOnTFT(const char *text, uint16_t textColor, uint16_t backgroundColor)
{
  // Static variable to keep track of the Y position across function calls
  static int currentYonTFT = 15; // Start at the top of the display

  tft.setFreeFont(&FreeMonoBold12pt7b);         // Use the bold font
  tft.setTextColor(textColor, backgroundColor); // Set text color to white, background to black

  if (currentYonTFT > 255) // adjusted manually
  {
    tft.fillScreen(TFT_BLACK); // Clear the screen with a black background
    currentYonTFT = 15;        // Reset Y position to the top of the display
  }

  int lineHeight = 24; // Adjust this value according to your font size (e.g., FreeMonoBold12pt7b)

  // Print the text at the current cursor position
  tft.setCursor(10, currentYonTFT); // Set x to 10, adjust as needed
  tft.println(text);

  // Move to the next line
  currentYonTFT += lineHeight;
}

void checkYaesuCATserver()
{
  while (true) // Infinite loop until the server is online
  {
    // Query Yaesu CAT server
    Serial.println("Querying Yaesu CAT server...");
    String url = String("http://") + yaesuCatServerIP.toString() + ":5000/vfo";

    http.begin(url); // Set URL for Server 1
    http.setTimeout(5000); // Optional: Set a 5-second timeout

    int httpResponseCode = http.GET();

    if (httpResponseCode == 200) // If the server is online
    {
      Serial.println("Yaesu CAT server online!");
      printOnTFT("Yaesu CAT Server Online!", TFT_GREEN, TFT_BLACK);
      http.end(); // Close the connection
      break; // Exit the loop
    }
    else
    {
      Serial.println("Yaesu CAT server Offline!");
      printOnTFT("Yaesu CAT Server Offline!", TFT_RED, TFT_BLACK);
            printOnTFT("Retrying.....", TFT_YELLOW, TFT_BLACK);
    }
    http.end(); // Close the connection
    delay(2000); // Wait before retrying
  }
}
