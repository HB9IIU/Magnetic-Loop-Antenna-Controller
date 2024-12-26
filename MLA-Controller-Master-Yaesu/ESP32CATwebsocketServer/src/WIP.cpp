#include <Arduino.h>

// Define UART pins
#define TX2_PIN 17  // MAX3232 RX
#define RX2_PIN 16  // MAX3232 TX

// UART settings
#define BAUD_RATE 38400

// Radio state variables
unsigned long VFOFrequency = 0;  // Store frequency as an unsigned long
String Mode = "";                // Store mode as a human-readable string
String PTTStatus = "";           // Store PTT status as "ON" or "OFF"
int RFPower = 0;                 // Store RF power as an integer (in watts)
float SWR = 0.0;                 // Store SWR as a float

// CAT Commands
const char* CAT_COMMANDS[] = {
  "IF;",  // Information: VFO Frequency and Mode
  "TX;",  // PTT Status
  "PC;",  // RF Power
  "RM6;"  // SWR
};

// Function to calculate SWR from the CAT value using a 5th-order polynomial fit
float calculateSWR(int swrCatValue) {
  float x = swrCatValue;
  return (
    5.84151360e-10 * pow(x, 5)
    - 1.51007177e-07 * pow(x, 4)
    + 1.50120555e-05 * pow(x, 3)
    - 5.98745703e-04 * pow(x, 2)
    + 1.46314171e-02 * x
    + 1.0
  );
}

// Function to send CAT command and read response
String sendCATCommand(const char* command) {
  Serial2.print(command);  // Send CAT command
  delay(20);               // Short delay to allow the radio to respond
  String response = "";
  while (Serial2.available()) {
    response += char(Serial2.read());  // Read response from radio
  }
  return response;
}

// Function to translate mode code to human-readable mode
String modeCodeToName(char modeCode) {
  switch (modeCode) {
    case '1': return "LSB";
    case '2': return "USB";
    case '3': return "CW-U";
    case '4': return "FM";
    case '5': return "AM";
    case '6': return "RTTY-L";
    case '7': return "CW-L";
    case '8': return "DATA-L";
    case '9': return "RTTY-U";
    case 'A': return "DATA-FM";
    case 'B': return "FM-N";
    case 'C': return "DATA-U";
    case 'D': return "AM-N";
    case 'E': return "PSK";
    case 'F': return "DATA-FM-N";
    default:  return "Unknown";
  }
}

// Function to parse CAT responses
void parseCATResponses() {
  // Query VFO Frequency and Mode
  String response = sendCATCommand(CAT_COMMANDS[0]);
  if (response.length() >= 22 && response.startsWith("IF") && response.endsWith(";")) {
    VFOFrequency = response.substring(5, 14).toInt();  // Extract VFO Frequency as unsigned long
    char modeCode = response[21];                      // Extract Mode Code
    Mode = modeCodeToName(modeCode);                   // Convert to human-readable mode
  }

  // Query PTT Status
  response = sendCATCommand(CAT_COMMANDS[1]);
  if (response.startsWith("TX") && response.length() >= 3) {
    int status = response[2] - '0';
    PTTStatus = (status == 1 || status == 2) ? "ON" : "OFF";
  }

  // Query RF Power
  response = sendCATCommand(CAT_COMMANDS[2]);
  if (response.startsWith("PC") && response.length() >= 5) {
    RFPower = response.substring(3, 5).toInt();  // Extract RF Power as integer
  }

  // Query SWR (only if PTT is ON)
  if (PTTStatus == "ON") {
    response = sendCATCommand(CAT_COMMANDS[3]);
    if (response.startsWith("RM") && response.length() >= 6) {
      int swrCatValue = response.substring(3, 6).toInt();
      SWR = calculateSWR(swrCatValue);  // Calculate SWR using polynomial fit
    } else {
      SWR = 0.0;  // Invalid response, set SWR to 0.0
    }
  } else {
    SWR = 0.0;  // SWR is irrelevant if not transmitting
  }
}

// Setup function
void setup() {
  // Start Serial Monitor for debugging
  Serial.begin(115200);
  // Initialize Serial2 for CAT communication
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.println("ESP32 CAT Polling Initialized");
}

// Main loop
void loop() {
  // Poll radio and update variables
  parseCATResponses();

  // Print the current state
  Serial.println("---- Radio State ----");
  Serial.print("VFO Frequency: ");
  Serial.println(VFOFrequency);
  Serial.print("Mode: ");
  Serial.println(Mode);
  Serial.print("PTT Status: ");
  Serial.println(PTTStatus);
  Serial.print("RF Power: ");
  Serial.print(RFPower);
  Serial.println(" W");
  Serial.print("SWR: ");
  Serial.println(SWR, 1);  // Print SWR with 1 decimal place
  Serial.println("----------------------");

  delay(1000);  // Poll every 1 second
}
