#include <Arduino.h>
#include "BLEDevice.h"

// UUIDs for the service and characteristics
static BLEUUID serviceUUID("f789b9d5-bc7e-438a-891c-5f9dce9c37a3");
static BLEUUID notifyCharUUID("bd294fde-579c-41f8-b05a-72cbbd1dbf1f");
static BLEUUID writeCharUUID("c36a7a4b-7d5e-4bb8-9212-f5d08ef5d69e");

static boolean doConnect = false;
static boolean connected = false;
static boolean reconnect = false;
static BLERemoteCharacteristic *pNotifyCharacteristic = nullptr;
static BLERemoteCharacteristic *pWriteCharacteristic = nullptr;
static BLEAdvertisedDevice *myDevice = nullptr;

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{
    char value[length + 1];
    memcpy(value, pData, length);
    value[length] = '\0'; // Null-terminate the string

    Serial.print("Notification received: ");
    Serial.println(value);

    // Parse the prefix and value
    String message = String(value);
    int separatorIndex = message.indexOf(':');
    if (separatorIndex == -1)
    {
        Serial.println("Invalid message format.");
        return;
    }

    String prefix = message.substring(0, separatorIndex);
    String data = message.substring(separatorIndex + 1);

    // Process based on the prefix
    if (prefix == "FREQ")
    {
        unsigned long VFOFrequency = data.toInt();
        Serial.print("VFO Frequency: ");
        Serial.println(VFOFrequency);
    }
    else if (prefix == "MODE")
    {
        Serial.print("Mode: ");
        Serial.println(data);
    }
    else if (prefix == "PTT")
    {
        Serial.print("PTT Status: ");
        Serial.println(data);
    }
    else if (prefix == "POWER")
    {
        int RFPower = data.toInt();
        Serial.print("RF Power: ");
        Serial.println(RFPower);
    }
    else if (prefix == "SWR")
    {
        float SWR = data.toFloat();
        Serial.print("SWR: ");
        Serial.println(SWR);
    }
    else
    {
        Serial.println("Unknown prefix.");
    }
}

class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("Connected to server.");
        connected = true;
    }

    void onDisconnect(BLEClient *pclient)
    {
        connected = false;
        reconnect = true; // Trigger reconnection process
        Serial.println("Disconnected from server. Restarting scan...");
    }
};

bool connectToServer()
{
    Serial.println("Connecting to server...");
    BLEClient *pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    if (!pClient->connect(myDevice))
    {
        Serial.println("Failed to connect to server.");
        return false;
    }

    Serial.println("Connected to server. Retrieving service...");
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (!pRemoteService)
    {
        Serial.println("Failed to find service.");
        pClient->disconnect();
        return false;
    }

    // Retrieve notification characteristic
    pNotifyCharacteristic = pRemoteService->getCharacteristic(notifyCharUUID);
    if (!pNotifyCharacteristic)
    {
        Serial.println("Failed to find notification characteristic.");
        pClient->disconnect();
        return false;
    }

    if (pNotifyCharacteristic->canNotify())
    {
        pNotifyCharacteristic->registerForNotify(notifyCallback);
    }

    // Retrieve write characteristic
    pWriteCharacteristic = pRemoteService->getCharacteristic(writeCharUUID);
    if (!pWriteCharacteristic || !pWriteCharacteristic->canWrite())
    {
        Serial.println("Failed to find writable characteristic.");
        pClient->disconnect();
        return false;
    }

    connected = true;
    reconnect = false; // Clear reconnection flag
    return true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        Serial.print("Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());

        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
        {
            Serial.println("Matching device found. Stopping scan.");
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
        }
    }
};

void resetScan()
{
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->stop();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(0, false);
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("Starting BLE Client...");

    BLEDevice::init("");
    resetScan();
}


void loop()
{
    // Configuration settings
    bool testFrequency = true;                  // Enable or disable frequency tests
    unsigned long frequencyInterval = 10000;     // Interval for frequency tests (ms)

    bool testMode = true;                       // Enable or disable mode tests
    unsigned long modeInterval = 8000;          // Interval for mode tests (ms)

    bool testPTT = false;                        // Enable or disable PTT tests
    unsigned long pttOnInterval = 5000;         // Interval for PTT ON (ms)
    unsigned long pttOffInterval = 1000;        // Interval for PTT OFF (ms)

    bool testPower = true;                      // Enable or disable power tests
    unsigned long powerInterval = 10000;         // Interval for power tests (ms)

    if (doConnect)
    {
        if (connectToServer())
        {
            Serial.println("Connected to server successfully.");
        }
        else
        {
            Serial.println("Connection failed. Restarting scan...");
            resetScan();
        }
        doConnect = false;
    }

    // Handle reconnection if flagged
    if (reconnect && !connected)
    {
        Serial.println("Attempting to reconnect to server...");
        resetScan();
        reconnect = false; // Reset flag after starting scan
    }

    static unsigned long lastFrequencyTestTime = 0; // Tracks the last time a test frequency was sent
    static unsigned long lastModeTestTime = 0;      // Tracks the last time a mode command was sent
    static unsigned long lastPTTTime = 0;           // Tracks the last time a PTT command was sent
    static unsigned long lastPowerTestTime = 0;     // Tracks the last time a power command was sent
    static bool pttState = false;                   // Current PTT state (false = OFF, true = ON)

    static unsigned long testFrequencies[] = {
        1800000, 2000000,   // 160m band
        3500000, 4000000,   // 80m band
        7000000, 7300000,   // 40m band
        10100000, 10150000, // 30m band
        14000000, 14350000, // 20m band
        18068000, 18168000, // 17m band
        21000000, 21450000, // 15m band
        24890000, 24990000, // 12m band
        28000000, 29700000, // 10m band
        50000000, 54000000, // 6m band
        70000000, 71000000  // 4m band (regional allocation)
    }; // Frequencies in Hz

    static const char *modes[] = {
        "LSB", "USB", "CW-U", "FM", "AM", "RTTY-L", "CW-L", "DATA-L", "RTTY-U", "DATA-FM", "FM-N", "DATA-U", "AM-N", "PSK", "DATA-FM-N"
    };
    static int frequencyIndex = 0; // Current index in the frequency sequence
    static int modeIndex = 0;      // Current index in the mode sequence

    if (connected && pWriteCharacteristic)
    {
        unsigned long currentTime = millis(); // Get the current time

        // Frequency Test
        if (testFrequency && currentTime - lastFrequencyTestTime >= frequencyInterval)
        {
            unsigned long frequency = testFrequencies[frequencyIndex];
            String command = "SETFREQ:" + String(frequency);
            pWriteCharacteristic->writeValue(command.c_str());
            Serial.println("Frequency command sent: " + command);

            frequencyIndex = (frequencyIndex + 1) % (sizeof(testFrequencies) / sizeof(testFrequencies[0]));
            lastFrequencyTestTime = currentTime;
        }

        // Mode Test
        if (testMode && currentTime - lastModeTestTime >= modeInterval)
        {
            String mode = modes[modeIndex];
            String command = "SETMODE:" + String(mode);
            pWriteCharacteristic->writeValue(command.c_str());
            Serial.println("Mode command sent: " + command);

            modeIndex = (modeIndex + 1) % (sizeof(modes) / sizeof(modes[0]));
            lastModeTestTime = currentTime;
        }

        // PTT Test
        if (testPTT && currentTime - lastPTTTime >= (pttState ? pttOffInterval : pttOnInterval))
        {
            String command = "SETPTT:" + String(pttState ? "OFF" : "ON");
            pWriteCharacteristic->writeValue(command.c_str());
            Serial.println("PTT command sent: " + command);

            pttState = !pttState;
            lastPTTTime = currentTime;
        }

        // Power Test
        if (testPower && currentTime - lastPowerTestTime >= powerInterval)
        {
            int power = random(5, 101); // Random power between 5W and 100W
            String command = "SETPWR:" + String(power);
            pWriteCharacteristic->writeValue(command.c_str());
            Serial.println("Power command sent: " + command);

            lastPowerTestTime = currentTime;
        }
    }

    //delay(20); // Reduce delay for more frequent updates
}
