#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define UART pins for RS232 com
#define TX2_PIN 17 // MAX3232 RX
#define RX2_PIN 16 // MAX3232 TX

// UART settings
#define BAUD_RATE 38400

const unsigned long updateInterval = 50; // Update every 100 ms

// Radio state variables
unsigned long VFOFrequency = 0; // Store frequency as an unsigned long
String Mode = "";               // Store mode as a human-readable string
String PTTStatus = "";          // Store PTT status as "ON" or "OFF"
int RFPower = 0;                // Store RF power as an integer (in watts)
float SWR = 1.0;                // Store SWR as a float

// Previous values for tracking changes
unsigned long prevVFOFrequency = 0;
String prevMode = "";
String prevPTTStatus = "";
int prevRFPower = 0;
float prevSWR = 0.0;

// CAT Commands
const char *CAT_COMMANDS[] = {
    "IF;", // Information: VFO Frequency and Mode
    "TX;", // PTT Status
    "PC;", // RF Power
    "RM6;" // SWR
};

String pendingCATCommand = ""; // Store pending command from client

// Function to calculate SWR from the CAT value using a 5th-order polynomial fit
float calculateSWR(int swrCatValue)
{
  float x = swrCatValue;
  return (
      5.84151360e-10 * pow(x, 5) - 1.51007177e-07 * pow(x, 4) +
      1.50120555e-05 * pow(x, 3) - 5.98745703e-04 * pow(x, 2) +
      1.46314171e-02 * x + 1.0);
}

// Function to send CAT command and read response
String sendCATCommand(const char *command)
{
  // Serial.print("Sending CAT command: ");
  // Serial.println(command);

  Serial2.print(command); // Send CAT command over UART
  delay(50);              // Allow time for the radio to respond

  String response = "";
  while (Serial2.available())
  {
    response += char(Serial2.read());
  }

  if (response.length() == 0 || !response.endsWith(";"))
  {
    Serial.println("Invalid CAT response received.");
    return ""; // Return empty string for invalid responses
  }

  // Serial.print("CAT response received: ");
  // Serial.println(response);
  return response;
}

// Function to translate mode code to human-readable mode
String modeCodeToName(char modeCode)
{
  switch (modeCode)
  {
  case '1':
    return "LSB";
  case '2':
    return "USB";
  case '3':
    return "CW-U";
  case '4':
    return "FM";
  case '5':
    return "AM";
  case '6':
    return "RTTY-L";
  case '7':
    return "CW-L";
  case '8':
    return "DATA-L";
  case '9':
    return "RTTY-U";
  case 'A':
    return "DATA-FM";
  case 'B':
    return "FM-N";
  case 'C':
    return "DATA-U";
  case 'D':
    return "AM-N";
  case 'E':
    return "PSK";
  case 'F':
    return "DATA-FM-N";
  default:
    return "Unknown";
  }
}

// Function to translate mode name to mode code
char modeNameToModeCode(String modeName) {
  if (modeName == "LSB") return '1';
  if (modeName == "USB") return '2';
  if (modeName == "CW-U") return '3';
  if (modeName == "FM") return '4';
  if (modeName == "AM") return '5';
  if (modeName == "RTTY-L") return '6';
  if (modeName == "CW-L") return '7';
  if (modeName == "DATA-L") return '8';
  if (modeName == "RTTY-U") return '9';
  if (modeName == "DATA-FM") return 'A';
  if (modeName == "FM-N") return 'B';
  if (modeName == "DATA-U") return 'C';
  if (modeName == "AM-N") return 'D';
  if (modeName == "PSK") return 'E';
  if (modeName == "DATA-FM-N") return 'F';
  return 'X'; // Unknown mode
}


// Function to parse CAT responses
void parseCATResponses()
{
  static int currentCommandIndex = 0; // Track the current command index

  String response = sendCATCommand(CAT_COMMANDS[currentCommandIndex]);

  if (currentCommandIndex == 0)
  { // VFO Frequency and Mode
    if (response.length() >= 22 && response.startsWith("IF") && response.endsWith(";"))
    {
      VFOFrequency = response.substring(5, 14).toInt();
      char modeCode = response[21];
      Mode = modeCodeToName(modeCode);
    }
  }
  else if (currentCommandIndex == 1)
  { // PTT Status
    if (response.startsWith("TX") && response.length() >= 3)
    {
      int status = response[2] - '0';
      PTTStatus = (status == 1 || status == 2) ? "ON" : "OFF";
    }
  }
  else if (currentCommandIndex == 2)
  { // RF Power
    if (response.startsWith("PC") && response.length() >= 5 && response.endsWith(";"))
    {
      RFPower = response.substring(3, 5).toInt();
    }
  }
  else if (currentCommandIndex == 3 && PTTStatus == "ON")
  { // SWR (if PTT is ON)
    if (response.startsWith("RM") && response.length() >= 6 && response.endsWith(";"))
    {
      int swrCatValue = response.substring(3, 6).toInt();
      SWR = calculateSWR(swrCatValue);
    }
    else
    {
      SWR = 1.0; // Invalid response, set default SWR
    }
  }

  currentCommandIndex = (currentCommandIndex + 1) % 4; // Move to next command
}

//---------- BLE RELATED -------------------------------------
#define SERVICE_UUID "f789b9d5-bc7e-438a-891c-5f9dce9c37a3"
#define NOTIFY_CHAR_UUID "bd294fde-579c-41f8-b05a-72cbbd1dbf1f"
#define WRITE_CHAR_UUID "c36a7a4b-7d5e-4bb8-9212-f5d08ef5d69e"

BLEServer *pServer;
BLECharacteristic *pNotifyCharacteristic;
BLECharacteristic *pWriteCharacteristic;
bool deviceConnected = false;
bool isAdvertising = false;

bool sendAllVariables = false; // Flag to trigger sending all variables once
/*
class MyWriteCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0)
    {
      Serial.print("Received message: ");
      Serial.println(value.c_str());

      String receivedData = String(value.c_str());
      int separatorIndex = receivedData.indexOf(':');
      if (separatorIndex != -1)
      {
        String command = receivedData.substring(0, separatorIndex);
        String data = receivedData.substring(separatorIndex + 1);

        if (command == "SETFREQ")
        {
          VFOFrequency = data.toInt();
          char formattedCommand[15];
          snprintf(formattedCommand, sizeof(formattedCommand), "FA%09lu;", VFOFrequency);
          pendingCATCommand = String(formattedCommand);
        }
        else if (command == "SETMODE")
        {
          pendingCATCommand = "MD" + data + ";";
        }
        else
        {
          Serial.println("Invalid or unrecognized command format.");
        }
      }
      else
      {
        Serial.println("Invalid or unrecognized command format.");
      }
    }
  }
};


class MyWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received message: ");
      Serial.println(value.c_str());

      String receivedData = String(value.c_str());
      int separatorIndex = receivedData.indexOf(':');
      if (separatorIndex != -1) {
        String command = receivedData.substring(0, separatorIndex);
        String data = receivedData.substring(separatorIndex + 1);

        // Handle specific commands
        if (command == "SETFREQ") {
          unsigned long frequency = data.toInt();
          Serial.print("Set VFO Frequency to: ");
          Serial.println(frequency);

          char formattedCommand[15];
          snprintf(formattedCommand, sizeof(formattedCommand), "FA%09lu;", frequency);
          Serial.print("Formatted CAT command: ");
          Serial.println(formattedCommand);
          sendCATCommand(formattedCommand);
        } else if (command == "SETMODE") {
          Serial.print("Set Mode to: ");
          Serial.println(data);
          String formattedCommand = "MD" + data + ";";
          sendCATCommand(formattedCommand.c_str());
        } else if (command == "SETPOWER") {
          int power = data.toInt();
          Serial.print("Set RF Power to: ");
          Serial.println(power);

          // Format CAT command for setting power
          char formattedCommand[10];
          snprintf(formattedCommand, sizeof(formattedCommand), "PC%03d;", power);
          Serial.print("Formatted CAT command: ");
          Serial.println(formattedCommand);
          sendCATCommand(formattedCommand);
        } else {
          Serial.println("Invalid or unrecognized command format.");
        }
      } else {
        Serial.println("Invalid or unrecognized command format.");
      }
    }
  }
};



class MyWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received message: ");
      Serial.println(value.c_str());

      String receivedData = String(value.c_str());
      int separatorIndex = receivedData.indexOf(':');
      if (separatorIndex != -1) {
        String command = receivedData.substring(0, separatorIndex);
        String data = receivedData.substring(separatorIndex + 1);

        // Handle specific commands
        if (command == "SETFREQ") {
          unsigned long frequency = data.toInt();
          Serial.print("Set VFO Frequency to: ");
          Serial.println(frequency);

          char formattedCommand[15];
          snprintf(formattedCommand, sizeof(formattedCommand), "FA%09lu;", frequency);
          Serial.print("Formatted CAT command: ");
          Serial.println(formattedCommand);
          sendCATCommand(formattedCommand);
        } else if (command == "SETMODE") {
          Serial.print("Set Mode to: ");
          Serial.println(data);

          char modeCode = modeNameToModeCode(data);
          if (modeCode != 'X') {
            char formattedCommand[10];
            snprintf(formattedCommand, sizeof(formattedCommand), "MD0%c;", modeCode);
            Serial.print("Formatted CAT command: ");
            Serial.println(formattedCommand);
            sendCATCommand(formattedCommand);
          } else {
            Serial.println("Invalid mode name received.");
          }
        } else {
          Serial.println("Invalid or unrecognized command format.");
        }
      } else {
        Serial.println("Invalid or unrecognized command format.");
      }
    }
  }
};

*/

class MyWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received message: ");
      Serial.println(value.c_str());

      String receivedData = String(value.c_str());
      int separatorIndex = receivedData.indexOf(':');
      if (separatorIndex != -1) {
        String command = receivedData.substring(0, separatorIndex);
        String data = receivedData.substring(separatorIndex + 1);

        // Handle specific commands
        if (command == "SETFREQ") {
          unsigned long frequency = data.toInt();
          Serial.print("Set VFO Frequency to: ");
          Serial.println(frequency);

          char formattedCommand[15];
          snprintf(formattedCommand, sizeof(formattedCommand), "FA%09lu;", frequency);
          Serial.print("Formatted CAT command: ");
          Serial.println(formattedCommand);
          sendCATCommand(formattedCommand);
        } else if (command == "SETMODE") {
          Serial.print("Set Mode to: ");
          Serial.println(data);

          char modeCode = modeNameToModeCode(data);
          if (modeCode != 'X') {
            char formattedCommand[10];
            snprintf(formattedCommand, sizeof(formattedCommand), "MD0%c;", modeCode);
            Serial.print("Formatted CAT command: ");
            Serial.println(formattedCommand);
            sendCATCommand(formattedCommand);
          } else {
            Serial.println("Invalid mode name received.");
          }
        } else if (command == "SETPWR") {
          int power = data.toInt();
          Serial.print("Set RF Power to: ");
          Serial.println(power);

          char formattedCommand[10];
          snprintf(formattedCommand, sizeof(formattedCommand), "PC%03d;", power);
          Serial.print("Formatted CAT command: ");
          Serial.println(formattedCommand);
          sendCATCommand(formattedCommand);
        } else if (command == "SETPTT") {
          if (data == "ON") {
            Serial.println("Set PTT to ON");
            sendCATCommand("TX1;");
          } else if (data == "OFF") {
            Serial.println("Set PTT to OFF");
            sendCATCommand("TX0;");
          } else {
            Serial.println("Invalid PTT state received.");
          }
        } else {
          Serial.println("Invalid or unrecognized command format.");
        }
      } else {
        Serial.println("Invalid or unrecognized command format.");
      }
    }
  }
};



class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Client connected!");
    deviceConnected = true;
    isAdvertising = false;
    delay(800);
    sendAllVariables = true;
  }

  void onDisconnect(BLEServer *pServer)
  {
    Serial.println("Client disconnected. Restarting advertising...");
    deviceConnected = false;
    BLEDevice::startAdvertising();
    isAdvertising = true;
  }
};

void setup()
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting BLE Server...");

  BLEDevice::init("HB9IIU Yaesu CAT Server");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pNotifyCharacteristic = pService->createCharacteristic(
      NOTIFY_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pNotifyCharacteristic->setValue("0");

  pWriteCharacteristic = pService->createCharacteristic(
      WRITE_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new MyWriteCallback());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  isAdvertising = true;

  Serial.println("Server ready and advertising...");
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.println("ESP32 CAT Polling Initialized");
}
void loop()
{
  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  // Periodically update radio state
  if (currentTime - lastUpdateTime >= updateInterval)
  {
    lastUpdateTime = currentTime;

    // Parse CAT responses to update radio variables
    // Serial.println("Polling CAT responses...");
    parseCATResponses();

    // If there's a pending CAT command from the client, send it
    if (!pendingCATCommand.isEmpty())
    {
      Serial.print("Sending pending CAT command: ");
      Serial.println(pendingCATCommand);
      sendCATCommand(pendingCATCommand.c_str());
      pendingCATCommand = ""; // Clear the pending command
    }

    // Send all variables to the client if this flag is set
    if (deviceConnected && sendAllVariables)
    {
      Serial.println("Sending all variables to client...");

      String message = "FREQ:" + String(VFOFrequency);
      pNotifyCharacteristic->setValue(message.c_str());
      pNotifyCharacteristic->notify();
      Serial.println("Sent: " + message);

      message = "MODE:" + Mode;
      pNotifyCharacteristic->setValue(message.c_str());
      pNotifyCharacteristic->notify();
      Serial.println("Sent: " + message);

      message = "PTT:" + PTTStatus;
      pNotifyCharacteristic->setValue(message.c_str());
      pNotifyCharacteristic->notify();
      Serial.println("Sent: " + message);

      message = "POWER:" + String(RFPower);
      pNotifyCharacteristic->setValue(message.c_str());
      pNotifyCharacteristic->notify();
      Serial.println("Sent: " + message);

      message = "SWR:" + String(SWR, 1);
      pNotifyCharacteristic->setValue(message.c_str());
      pNotifyCharacteristic->notify();
      Serial.println("Sent: " + message);

      sendAllVariables = false; // Reset the flag after sending
    }

    // Send notifications for changed variables
    if (deviceConnected)
    {
      if (VFOFrequency != prevVFOFrequency)
      {
        String message = "FREQ:" + String(VFOFrequency);
        pNotifyCharacteristic->setValue(message.c_str());
        pNotifyCharacteristic->notify();
        Serial.print("Updated VFO Frequency: ");
        Serial.println(VFOFrequency);
        prevVFOFrequency = VFOFrequency;
      }

      if (Mode != prevMode)
      {
        String message = "MODE:" + Mode;
        pNotifyCharacteristic->setValue(message.c_str());
        pNotifyCharacteristic->notify();
        Serial.print("Updated Mode: ");
        Serial.println(Mode);
        prevMode = Mode;
      }

      if (PTTStatus != prevPTTStatus)
      {
        String message = "PTT:" + PTTStatus;
        pNotifyCharacteristic->setValue(message.c_str());
        pNotifyCharacteristic->notify();
        Serial.print("Updated PTT Status: ");
        Serial.println(PTTStatus);
        prevPTTStatus = PTTStatus;
      }

      if (RFPower != prevRFPower)
      {
        String message = "POWER:" + String(RFPower);
        pNotifyCharacteristic->setValue(message.c_str());
        pNotifyCharacteristic->notify();
        Serial.print("Updated RF Power: ");
        Serial.print(RFPower);
        Serial.println(" W");
        prevRFPower = RFPower;
      }

      if (SWR != prevSWR)
      {
        String message = "SWR:" + String(SWR, 1);
        pNotifyCharacteristic->setValue(message.c_str());
        pNotifyCharacteristic->notify();
        Serial.print("Updated SWR: ");
        Serial.println(SWR, 1);
        prevSWR = SWR;
      }
    }
  }

  // Restart advertising if disconnected
  if (!deviceConnected && !isAdvertising)
  {
    Serial.println("Restarting advertising...");
    BLEDevice::startAdvertising();
    isAdvertising = true;
  }
}
