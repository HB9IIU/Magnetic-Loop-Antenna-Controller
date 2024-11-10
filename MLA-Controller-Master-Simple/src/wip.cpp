// https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/tree/main/3dModels
// https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display/issues/47
#include <Arduino.h>
#include "BLEDevice.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
// display related
#include <TFT_eSPI.h>
#include "Free_Fonts.h"
#include <XPT2046_Touchscreen.h>
// WIFI RElated
#include <WiFi.h>
#include <HTTPClient.h>
// Include the PNG decoder library
#include <PNGdec.h>
#include "splash.h" // Image is stored here in an 8-bit array
PNG png; // PNG decoder instance

//-----------------------------------------------------------------------------------------------
// Create TFT instance
TFT_eSPI tft = TFT_eSPI();
// Touchscreen pins
#define XPT2046_IRQ 36  // T_IRQ
#define XPT2046_MOSI 32 // T_DIN
#define XPT2046_MISO 39 // T_OUT
#define XPT2046_CLK 25  // T_CLK
#define XPT2046_CS 33   // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// Define the frequency ranges
#define LOWER_40M 7000000
#define UPPER_40M 7300000
#define LOWER_20M 14000000
#define UPPER_20M 14350000
unsigned long CurrentVFOFrequency = 0;        // VFO frequency in Hz
unsigned long previousVFOFrequency = 0;       // to decide wether to refresh display or not
unsigned long previousResonanceFrequency = 0; // to decide wether to refresh display or not
const char *deviceName = "IC705-MLA-HB9IIU";  // Bluetooth Device Name
int currentYonTFT = 15;                       // to control new line position when printing on TFT display
bool reconnectFlag = false;                   // used for display refreshing purposes

#define SSID "HB9IIU-MLA"          // WiFi SSID to connect to
IPAddress slaveIP(192, 168, 4, 1); // IP address of the SLAVE device to send HTTP commands to
HTTPClient http;                   // HTTPClient instance

uint64_t theoreticalResonanceFrequency = 0; // theoretical resonance frequency at current stepper position

uint32_t currentStepperPosition = 0;   // current stepper position
uint32_t predictedStepperPosition = 0; // predicted stepper position from lookup table
uint32_t deltaSteps = 0;               // diff between current stepper position and predicted stepper position
// Variable to hold the decoded frequency
// Variable to hold the last known frequency
// unsigned long last_VFO_Frequency = 0; // Last known VFO frequency in Hz
// Variables for tracking PTT state and double-click detection
unsigned long lastPTTPressTime = 0;
unsigned long doubleClickThreshold = 300; // Adjust the time threshold as needed (in milliseconds)
bool PTTFirstClickDetected = false;
bool errorBanner = false;
//-----------------------------------------------------------------------------------------------

// UUIDs for the remote Nordic UART service and its characteristics
static BLEUUID serviceUUID("14cf8001-1ec2-d408-1b04-2eb270f14203");
static BLEUUID charUUID_RX("14cf8002-1ec2-d408-1b04-2eb270f14203"); // RX Characteristic
static BLEUUID charUUID_TX("14cf8002-1ec2-d408-1b04-2eb270f14203"); // TX Characteristic

// Variables for BLE connection state and characteristics
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic *pTXCharacteristic;
static BLERemoteCharacteristic *pRXCharacteristic;
uint8_t radio_address = 0xA4; // A4 for IC-705

//-----------------------------------------------------------------------------------------------
// only what is in this section was tested

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
// Function prototypes
void displayWelcomeScreen(int duration, const char *version, const char *date);
void printOnTFT(const char *text);
void establish_WIFI_connection_with_Slave();
bool isPTTPressed = false;                    // Track the current PTT state
const unsigned long longPressThreshold = 500; // Duration for long press detection (in milliseconds)
bool doublePTTdetected = false;
uint8_t currentMode = 0;   // Will store the current mode value
uint8_t currentFilter = 0; // Will store the current filter value
uint16_t swrByteValue;     // from CAT message
void displayVFOfrequency(long freq, int x, int y, uint16_t colour);
void displayRESONANCEfrequency(long freq, int x, int y, uint16_t colour);
void displayStepperInfo(int currentPos, int stepsToGo);
void drawProgressBar(int duration);
long estimated_movement_duration_in_microseconds;
void GetTunedStatusFromSlave();
void setNewPositionForCurrentVFOfrequency(uint32_t targetFrequency);
bool PTTisON = false;
bool allStepperInfotoBeRedrawn = true;

void displayMessageAtBottom(const char *message, int font);
void updateWiFiWidget(int x, int y, int radius, float sizePercentage, int rssi);

//-----------------------------------------------------------------------------------------------
// SWR RELATED ONLY
// Define the display area for the SWR meter
#define SWR_BAR_X 8      // X position of the bar
#define SWR_BAR_Y 189     // Y position of the bar
#define SWR_BAR_WIDTH 310 // Width of the bar
#define SWR_BAR_HEIGHT 23  // Height of the bar
#define SWR_TICK_HEIGHT 10 // Height of tick marks
#define SEGMENT_WIDTH 8    // Width of the segments (spacing between vertical lines)
//Define the colors
#define SWR_COLOR_GOOD TFT_GREENYELLOW
#define COLOR_WARNING TFT_ORANGE
#define COLOR_DANGER TFT_RED
#define SWR_COLOR_BG TFT_BLACK
#define SWR_COLOR_TICK TFT_WHITE
#define SEGMENT_COLOR TFT_WHITE       // Color for the segment lines
#define SEGMENT_COLOR_BLACK TFT_BLACK // Color for segment lines in the green portion

// Store the previous SWR value to compare for optimization
float previous_swr = -1.0;
bool SWR_first_draw = true;

// Store a flag for the first draw
// Function declarations
float SWR_mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float SWR_calculate(int byte_value);
float SWR_applyLogarithmicScale(float swr);
void SWR_drawTicksAndLabels();
int SWR_mapSWRToBarLength(float swr);
void SWR_drawSegment(int x, int bar_length, int green_length);
void SWR_eraseBar(int from_x, int to_x);
void SWR_drawSegmentsInFilledArea(int bar_length, int green_length);
void SWR_drawSWRBar(float swr);

void pngDraw(PNGDRAW *pDraw) ;
void pngSplashScreen();



//-----------------------------------------------------------------------------------------------

//.......
String formatStepperPosForConsoleOutput(uint32_t value);
String formatStepsToGoForTFT(int32_t stepsToGo);
String formatStepsToGoForConsole(int32_t value);
String getLinkQualityDescription(int8_t rssi);
char *formatFrequencyForConsoleOutput(uint64_t vfo);

void checkIfWifiIsStillConnected();

//-----------------------------------------------------------------------------------------------

String getLinkQualityDescription(int8_t rssi)
{
    if (rssi > -30)
    {
        return "Amazing"; // Strong signal
    }
    else if (rssi > -67)
    {
        return "Great"; // Good signal
    }
    else if (rssi > -70)
    {
        return "Average"; // Fair signal
    }
    else if (rssi > -80)
    {
        return "Poor"; // Weak signal
    }
    else if (rssi > -90)
    {
        return "Unusable"; // Very weak signal
    }
    else
    {
        return "Unusable"; // Signal is too weak
    }
}

String sendCommandToSlave(const String &command, const String &argument)
{
    String postData = "command=" + command;
    if (argument.length() > 0)
    {
        postData += "&argument=" + argument;
    }
    // Serial.println("Sending POST request with data: " + postData);
    int httpResponseCode = http.POST(postData); // Send POST request
    if (httpResponseCode > 0)
    {
        // Serial.print("HTTP Response code: ");
        // Serial.println(httpResponseCode);
        String response = http.getString();
        // Serial.print("Raw Response: '");
        // Serial.print(response);
        // Serial.println("'");
        return response;
    }
    else
    {
        Serial.print("Error on sending POST: ");
        Serial.println(httpResponseCode);
        return "";
    }
}

void GetTunedStatusFromSlave()
{
    const int maxRetries = 5;    // Maximum number of retries
    const int retryDelay = 1000; // Delay between retries in milliseconds
    int attempts = 0;
    bool success = false;

    while (attempts < maxRetries && !success)
    {
        attempts++;
        Serial.printf("\nAttempt %d: Getting Current stepper position and associated frequency from Slave\n", attempts);
        String response = sendCommandToSlave("GetTunedStatusFromSlave", "");

        if (response.length() > 0)
        {
            // Parse the response to extract stepper position and VFO_Frequency
            int separatorIndex = response.indexOf(',');
            if (separatorIndex != -1)
            {
                String stepperPositionStr = response.substring(0, separatorIndex);
                String frequencyStr = response.substring(separatorIndex + 1);
                currentStepperPosition = stepperPositionStr.toInt();
                theoreticalResonanceFrequency = frequencyStr.toInt();
                Serial.print("Current Stepper Position (from slave): ");
                Serial.println(formatStepperPosForConsoleOutput(currentStepperPosition));
                Serial.print("Resulting Lookup Resonance Frequency:  ");
                Serial.println(formatFrequencyForConsoleOutput(theoreticalResonanceFrequency));

                Serial.println("");
                success = true; // Mark as successful xxxxxxxxx

                // updateRESONANCEfreqOnLCD(theoreticalResonanceFrequency);
            }
            else
            {
                Serial.println("Unexpected response format.");
            }
        }
        else
        {
            Serial.println("Failed to get a response from Slave. Retrying...");
            delay(retryDelay); // Wait before retrying
        }
    }

    if (!success)
    {
        Serial.println("Failed to get tuned status from Slave after multiple attempts.");
        // displayErrorOnLCD("GetTunedStatusFromSlave()");
    }
}
// Function to decode the frequency from the received data
void decodeFrequency(uint8_t *pData, size_t length)
{
    if (length != 11)
        return; // Ensure that we have the valid length for frequency data

    // Extract the frequency blocks based on the expected positions
    uint8_t bloc_1 = pData[5];
    uint8_t bloc_2 = pData[6];
    uint8_t bloc_3 = pData[7];
    uint8_t bloc_4 = pData[8];
    uint8_t bloc_5 = pData[9];

    // Validate that each nibble is within BCD range (0 to 9)
    if (((bloc_1 & 0xF0) >> 4) > 9 || (bloc_1 & 0x0F) > 9 ||
        ((bloc_2 & 0xF0) >> 4) > 9 || (bloc_2 & 0x0F) > 9 ||
        ((bloc_3 & 0xF0) >> 4) > 9 || (bloc_3 & 0x0F) > 9 ||
        ((bloc_4 & 0xF0) >> 4) > 9 || (bloc_4 & 0x0F) > 9 ||
        ((bloc_5 & 0xF0) >> 4) > 9 || (bloc_5 & 0x0F) > 9)
    {
        Serial.println("Invalid BCD data detected. Skipping this packet.");
        return;
    }

    // Decode each block into its respective digits
    int a = (bloc_1 & 0xF0) >> 4;
    int b = bloc_1 & 0x0F;
    int c = (bloc_2 & 0xF0) >> 4;
    int d = bloc_2 & 0x0F;
    int e = (bloc_3 & 0xF0) >> 4;
    int f = bloc_3 & 0x0F;
    int g = (bloc_4 & 0xF0) >> 4;
    int h = bloc_4 & 0x0F;
    int i = (bloc_5 & 0xF0) >> 4;
    int j = bloc_5 & 0x0F;

    // Calculate the frequency in Hz
    unsigned long decodedFrequency = (unsigned long)(i * 1e9 + j * 1e8 + g * 1e7 + h * 1e6 + e * 1e5 + f * 1e4 + c * 1e3 + d * 1e2 + a * 10 + b);

    // Validate the decoded frequency range
    if (decodedFrequency < 1000000 || decodedFrequency > 30000000)
    {
        Serial.println("Decoded frequency out of valid range. Skipping this packet.");
        return;
    }

    // Check if the frequency has changed
    if (decodedFrequency != previousVFOFrequency)
    {

        CurrentVFOFrequency = decodedFrequency;
        // Print the decoded frequency
        Serial.print("CAT Decoded VFO Frequency: ");
        Serial.println(formatFrequencyForConsoleOutput(decodedFrequency));

        // Update the last known frequency
        previousVFOFrequency = decodedFrequency;
        CurrentVFOFrequency = decodedFrequency;
    }
}

// Function declaration for converting a string to a CIV format array
void stringToCIVArray(const char *inputString, uint8_t *outputArray, uint8_t &outputLength, uint8_t commandPrefix = 0x62);

// Function to convert a string to a uint8_t array in the CIV format
void stringToCIVArray(const char *inputString, uint8_t *outputArray, uint8_t &outputLength, uint8_t commandPrefix)
{
    const uint8_t CIV_HEADER[] = {0xFE, 0xF1, 0x00, commandPrefix};
    uint8_t endMarker = 0xFD;
    uint8_t headerLength = sizeof(CIV_HEADER);

    // Copy the header to the output array
    for (uint8_t i = 0; i < headerLength; ++i)
    {
        outputArray[i] = CIV_HEADER[i];
    }

    // Copy the input string to the output array in ASCII format
    uint8_t i = 0;
    while (inputString[i] != '\0')
    {
        outputArray[headerLength + i] = inputString[i];
        ++i;
    }

    // Add the end marker
    outputArray[headerLength + i] = endMarker;

    // Set the total length of the output array
    outputLength = headerLength + i + 1;
}

// Callback class for handling advertised devices
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID))
        {
            Serial.println("Found a device with the desired ServiceUUID!");

            advertisedDevice.getScan()->stop();
            pServerAddress = new BLEAddress(advertisedDevice.getAddress());
            doConnect = true;
        }
    }
};
bool connectToServer(BLEAddress pAddress);
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);

void getStepperPositionForCurrentVFOfrequency(uint32_t currentVFOfrequency);
void setNewPositionForCurrentVFOfrequency(uint32_t targetFrequency)
{

    Serial.print("Sending command to Slave to move stepper to position for frequency: ");
    Serial.println(formatFrequencyForConsoleOutput(targetFrequency));

    String response = sendCommandToSlave("SetNewPositionForCurrentVFOfrequency", String(targetFrequency));

    // Parse the response to extract estimated duration
    int separatorIndex = response.indexOf(',');

    uint32_t targetStepperPosition = response.substring(0, separatorIndex).toInt();
    estimated_movement_duration_in_microseconds = response.substring(separatorIndex + 1).toInt();

    Serial.print("Received Target Position:");
    Serial.println(targetStepperPosition);

    Serial.print("Estimated movement duration: ");
    Serial.print(estimated_movement_duration_in_microseconds);
    Serial.println(" microseconds");
    Serial.println("Animating Progress Bar");
    GetTunedStatusFromSlave();
}

String formatFrequencyNumbers(unsigned long number)
{
    String formattedNumber = "";

    // Add leading space for numbers less than 10,000,000
    if (number < 10000000)
    {
        formattedNumber += ' '; // Leading space
    }

    String numStr = String(number);
    int len = numStr.length();

    for (int i = 0; i < len; i++)
    {
        if (i > 0 && (len - i) % 3 == 0)
        {
            formattedNumber += '.'; // Add a dot as a thousands separator
        }
        formattedNumber += numStr[i]; // Add the next digit
    }

    return formattedNumber;
}

//------------------------------------------------------------------------------------------------------------------------
// DISPLAY RELATED FUNCTIONS

//------------------------------------------------------------------------------------------------------------------------

// Setup function
void setup()
{
    Serial.begin(115200);
    Serial.println("HB9IIU MLA Controller Starting");
    // Print chip model
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Chip model: %s\n", (chip_info.model == CHIP_ESP32) ? "ESP32" : "Unknown");
    // Print number of cores
    Serial.printf("Number of cores: %d\n", chip_info.cores);
    // Print chip revision
    Serial.printf("Chip revision: %d\n", chip_info.revision);
    // Print flash size
    Serial.printf("Flash size: %dMB\n", spi_flash_get_chip_size() / (1024 * 1024));
    // Initialize the TFT display
    tft.begin();
    tft.fillScreen(TFT_BLACK); // Set background to black
    tft.setRotation(1);        // Set rotation
    pngSplashScreen();
    delay(1300);
  // Seed the random number generator
    //randomSeed(analogRead(0)); // Use an unconnected analog pin for randomness

    // Endless loop in setup
    /*
    while (true) {
        // Generate a random float between 1.0 and 4.0 with one decimal place
        float swr = (random(10, 41)) / 10.0; 
        SWR_drawSWRBar(swr); // Call the function with the generated value
        delay(500); // Optional delay to control the rate of function calls
    }
*/

    displayWelcomeScreen(2000, "1.0", "November 2024");
    printOnTFT("HB9IIU MLA Controller Starting");
    establish_WIFI_connection_with_Slave();

    Serial.println("Scanning for Bluetooth devices");
    printOnTFT("Scanning for Bluetooth devices"); // Print the message on the TFT display

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(5, false);

    String message = "Found Bluetooth devices: " + String(foundDevices.getCount());
    printOnTFT(message.c_str()); // Use c_str() to convert the String to a C-style string
    Serial.println("Scan done!");
    printOnTFT("Scan done!");
    Serial.print("Found Bluetooth devices: ");
    Serial.println(foundDevices.getCount());

    if (!doConnect)
    {
        Serial.print("Scan did not reveal device: ");
        Serial.println(deviceName);
        String message = String(deviceName) + " not found";
        printOnTFT(message.c_str());
    }
    else
    {
        printOnTFT("IC705 Found!");
    }

    // Start the SPI for the touchscreen and init the touchscreen
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    // Set the Touchscreen rotation in landscape mode
    // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
    touchscreen.setRotation(1);
}

void loop()
{
    // Rescan only if not connected and doConnect is false
    if (!connected && !doConnect)
    {
        Serial.println("Rescanning for device...");

        BLEScan *pBLEScan = BLEDevice::getScan();
        pBLEScan->clearResults(); // Clear previous scan results
        BLEScanResults foundDevices = pBLEScan->start(5, false);

        Serial.print("Devices found: ");
        Serial.println(foundDevices.getCount());
        // Combine the text with the device count
        String message = "Devices found: " + String(foundDevices.getCount());
        // Print the combined message on the TFT display

        Serial.println("Rescan done!");

        if (!doConnect)
        {
            Serial.print("Device still not found: ");
            Serial.println(deviceName);
            delay(2000); // Wait for 2 seconds before re-scanning
        }
    }

    // Attempt to connect if doConnect is true and not yet connected
    if (doConnect && !connected)
    {
        if (connectToServer(*pServerAddress))
        {
            Serial.println("We are now connected to the BLE Server.");
            printOnTFT("Connection to BLE Server is OK!");
            printOnTFT("We can start !!!");
            delay(1500);
            tft.fillScreen(TFT_BLACK); // Set background to black

            updateWiFiWidget(160, 120, 100, 100, WiFi.RSSI());

            delay(2000);
            tft.fillScreen(TFT_BLACK); // Set background to black
            // getting VFO frequency for th e1st time
            uint8_t CIV_frequency[] = {0xFE, 0xFE, radio_address, 0xE0, 0x03, 0xFD};
            pRXCharacteristic->writeValue(CIV_frequency, sizeof(CIV_frequency));
            delay(80);

            // display fake value to initiate
            displayVFOfrequency(1111111111, 20, 10, TFT_CYAN);
            displayVFOfrequency(CurrentVFOFrequency, 20, 10, TFT_CYAN);
            GetTunedStatusFromSlave(); // to get current stepper pos and theoretical resonance freq
            // display fake value to initiate
            displayRESONANCEfrequency(1111111111, 22, 105, TFT_GREEN);
            displayRESONANCEfrequency(theoreticalResonanceFrequency, 22, 105, TFT_GREEN);
        }
        else
        {
            Serial.println("We have failed to connect to the server.");
        }
        doConnect = false; // Ensure we don’t attempt to reconnect immediately
    }

    // If connected, perform BLE communication
    if (connected)
    {
        // Check if still connected
        if (pTXCharacteristic == nullptr || !pTXCharacteristic->getRemoteService()->getClient()->isConnected())
        {
            Serial.println("Connection lost. Attempting to reconnect...");

            reconnectFlag = true; // to force VFO frequency display for unchanged frequency
            connected = false;
            doConnect = true; // Reset flags to trigger rescan
            delay(1000);      // Delay before retrying
            return;
        }
        // Continue normal communication if still connected
        // to get frequency
        uint8_t CIV_frequency[] = {0xFE, 0xFE, radio_address, 0xE0, 0x03, 0xFD};
        pRXCharacteristic->writeValue(CIV_frequency, sizeof(CIV_frequency));
        delay(80); // important, otherwise hangs
        // Send the CIV command to get the PTT status
        uint8_t CIV_PTT_Status[] = {0xFE, 0xFE, radio_address, 0xE0, 0x1C, 0x00, 0xFD};
        pRXCharacteristic->writeValue(CIV_PTT_Status, sizeof(CIV_PTT_Status));
        delay(80);
        displayVFOfrequency(CurrentVFOFrequency, 20, 10, TFT_CYAN);
        getStepperPositionForCurrentVFOfrequency(CurrentVFOFrequency);

        if (errorBanner == true

        )
        {

            displayMessageAtBottom("Out of Range!", 2);
            allStepperInfotoBeRedrawn = true;
        }

        if (PTTisON)
        {
            displayMessageAtBottom("ON AIR", 1);

            allStepperInfotoBeRedrawn = true;
        }
        // checkIfWifiIsStillConnected();
        if (touchscreen.tirqTouched() && touchscreen.touched())
        {
            // Get Touchscreen points
            TS_Point p = touchscreen.getPoint();
            // Calibrate Touchscreen points with map function to the correct width and height
            int x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
            int y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
            int z = p.z;
            Serial.print("X = ");
            Serial.print(x);
            Serial.print(" | Y = ");
            Serial.print(y);
            Serial.print(" | Pressure = ");
            Serial.print(z);
            Serial.println();
            if (deltaSteps != 0)
            {
                setNewPositionForCurrentVFOfrequency(CurrentVFOFrequency);
                drawProgressBar(estimated_movement_duration_in_microseconds);
                // getStepperPositionForCurrentVFOfrequency(CurrentVFOFrequency); // pas encore bon
                GetTunedStatusFromSlave(); // getting the new stepper position
                // cheating a bit because GetTunedStatusFromSlave will return a slightly different value of couple of Hz
                // so we print VFO frequency to avoid confusio
                allStepperInfotoBeRedrawn = true;
                displayRESONANCEfrequency(CurrentVFOFrequency, 22, 105, TFT_GREEN);
                deltaSteps = 0; // set manually, will be update at VFO frequ change
                displayStepperInfo(currentStepperPosition, deltaSteps);
            } else{
           
           
       // Loop to call SWR_drawSWRBar
    for (int i = 0; i < 10; i++) { // Change 10 to however many times you want to call the function
        // Generate a random float between 10.0 and 40.0, then divide by 10.0 to get 1.0 to 4.0
        float swr = (random(10, 41)) / 10.0; 
        SWR_drawSWRBar(swr); // Call the function with the generated value
        delay(100); // Optional delay between calls (500 ms here)
    }
 allStepperInfotoBeRedrawn = true;
            tft.fillRect(0, 185, 320, 240, TFT_BLACK); // Fill the rectangle from (0,y) to (displayWidth, displayHeight)

                displayStepperInfo(currentStepperPosition, deltaSteps);
    SWR_first_draw=true;
        }}
    }
}

String formatStepperPosForConsoleOutput(uint32_t value)
{
    // Convert the number to a string
    String result = String(value);

    // Variable to hold the result with separators
    String formattedResult = "";

    // Length of the number string
    int len = result.length();

    // Insert thousands separators every three digits from the right
    int insertPosition = len % 3;

    for (int i = 0; i < len; i++)
    {
        // Append current digit to the result
        formattedResult += result[i];

        // Insert separator if we are not at the end of the string
        if ((i - insertPosition + 1) % 3 == 0 && i != len - 1)
        {
            formattedResult += "."; // Separator
        }
    }

    return formattedResult;
}

char *formatFrequencyForConsoleOutput(uint64_t vfo)
{

    static char vfo_str[20] = {""};
    uint32_t MHz = (vfo / 1000000 % 1000000);
    uint16_t Hz = (vfo % 1000);
    uint16_t KHz = ((vfo % 1000000) - Hz) / 1000;
    sprintf(vfo_str, "%lu.%03u.%03u", MHz, KHz, Hz);
    return vfo_str;
}

void getStepperPositionForCurrentVFOfrequency(uint32_t currentVFOfrequency)
{

    static uint64_t prev_freq = 0;

    if (currentVFOfrequency != prev_freq && currentVFOfrequency != 0)

    {

        const int maxRetries = 5;    // Maximum number of retries
        const int retryDelay = 1000; // Delay between retries in milliseconds
        int attempts = 0;
        bool success = false;

        while (attempts < maxRetries && !success)
        {

            attempts++;
            Serial.printf("\nAttempt %d: Sending command to Slave to get theoretical stepper position for vfo frequency\n", attempts);
            String response = sendCommandToSlave("getStepperPositionForCurrentVFOfrequency", String(currentVFOfrequency));

            if (response.length() > 0)
            {
                // Parse the response to extract stepper position and VFO_Frequency
                int separatorIndex = response.indexOf(',');
                if (separatorIndex != -1)
                {
                    String predictedStepperPositionStr = response.substring(0, separatorIndex);
                    String currentStepperPositionStr = response.substring(separatorIndex + 1);
                    currentStepperPosition = currentStepperPositionStr.toInt();
                    predictedStepperPosition = predictedStepperPositionStr.toInt();
                    deltaSteps = predictedStepperPosition - currentStepperPosition;
                    Serial.print("Current Stepper Position (from Slave):   ");
                    Serial.println(formatStepperPosForConsoleOutput(currentStepperPosition));
                    Serial.print("Predicted Stepper Position (from Slave): ");
                    Serial.println(formatStepperPosForConsoleOutput(predictedStepperPosition));
                    Serial.print("Delta Steps:                            ");
                    Serial.println(formatStepsToGoForConsole(deltaSteps));

                    // Check if the frequency is within the defined ranges
                    if ((currentVFOfrequency >= LOWER_40M && currentVFOfrequency <= UPPER_40M) ||
                        (currentVFOfrequency >= LOWER_20M && currentVFOfrequency <= UPPER_20M))
                    {
                        Serial.println("Frequency is within the 40m range.");
                        displayStepperInfo(currentStepperPosition, deltaSteps);
                        errorBanner = false;
                    }
                    else
                    {
                        errorBanner = true;
                        // displayMessageAtBottom("Out of Range!",2);

                        Serial.println("Frequency is outside the defined ranges.");
                    }
                    success = true;
                }

                else
                {
                    Serial.println("Unexpected response format.");
                }
            }
            else
            {
                Serial.println("Failed to get a response from Slave. Retrying...");
                delay(retryDelay); // Wait before retrying
            }
        }

        if (!success)
        {
            Serial.println("Failed to get theoretical stepper position for vfo frequency from Slave after multiple attempts.");
            // displayErrorOnLCD("getStepperPositionForCurrentVFOfrequency()");
        }

        prev_freq = currentVFOfrequency;
    }
}

String formatStepsToGoForConsole(int32_t value)
{
    char buffer[11]; // Buffer to hold the formatted string, including null terminator
    String result = "";

    // Add the sign
    if (value < 0)
    {
        result = "-" + String(abs(value));
    }
    else
    {
        result = "+" + String(value);
    }

    int len = result.length();
    int insertPosition = len - 3;

    while (insertPosition > 1)
    { // Start from position 1 to leave the sign in front
        result = result.substring(0, insertPosition) + "'" + result.substring(insertPosition);
        insertPosition -= 3;
    }

    // Add padding spaces to make it 8 characters long
    snprintf(buffer, sizeof(buffer), "%8s", result.c_str());

    return String(buffer);
}

void checkIfWifiIsStillConnected()
{

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("\nWiFi Connection lost");
        currentYonTFT = 15; // Start at the top of the display

        establish_WIFI_connection_with_Slave();
    }
}

//

void displayWelcomeScreen(int duration, const char *version, const char *date)
{
    // Center coordinates
    int centerX = tft.width() / 2;  // X coordinate of the center
    int centerY = tft.height() / 2; // Y coordinate of the center
    // Circle parameters
    const int INITIAL_RADIUS = 10;
    const int GAP = 25;
    const int MAX_RADIUS = centerX; // Max radius based on screen width
    const int CIRCLE_THICKNESS = 2; // Thickness of the circle

    // Text Y positions
    const int TEXT_Y_WELCOME = 40;
    const int TEXT_Y_CONTROLLER = 100;
    const int TEXT_Y_VERSION = 160;

    // Erase color
    uint16_t eraseColor = TFT_BLACK;

    // Calculate the number of circles
    int numCircles = (MAX_RADIUS - INITIAL_RADIUS) / GAP + 1;
    int diameters[numCircles];

    // Populate the diameter array
    for (int i = 0; i < numCircles; i++)
    {
        diameters[i] = INITIAL_RADIUS + i * GAP;
    }

    // Draw the splash screen text

    tft.fillScreen(TFT_BLACK);   // Clear the screen with black color
    tft.setTextColor(TFT_WHITE); // Set text color to white
    tft.setTextFont(4);          // Set the font
    tft.setTextSize(1);          // Set text size to 1

    int xWelcome = (tft.width() - tft.textWidth("Welcome To")) / 2;
    tft.setCursor(xWelcome, TEXT_Y_WELCOME);
    tft.print("Welcome To");
    int xController = (tft.width() - tft.textWidth("HB9IIU MLA CONTROLLER")) / 2;
    tft.setCursor(xController, TEXT_Y_CONTROLLER);
    tft.print("HB9IIU MLA CONTROLLER");
    // Format the version string
    char versionText[50];
    snprintf(versionText, sizeof(versionText), "Version %s", version);
    // Format the date string
    char versionDate[50];
    snprintf(versionDate, sizeof(versionDate), "%s", date);
    tft.setTextFont(2); // Set the font
    int xVersion = (tft.width() - tft.textWidth(versionText)) / 2;
    int xDate = (tft.width() - tft.textWidth(versionDate)) / 2;
    // Draw version and date text
    tft.setCursor(xVersion, TEXT_Y_VERSION);
    tft.print(versionText);
    tft.setCursor(xDate, TEXT_Y_VERSION + 20);
    tft.print(versionDate);
    // Initialize random seed
    randomSeed(analogRead(0)); // Use an analog pin to seed random for better randomness
    // Display concentric circles with animation
    for (int j = 0; j < 5; j++)
    {
        // Draw concentric circles with random colors
        for (int i = 0; i < numCircles; i++)
        {
            // Generate a random color
            uint16_t randomColor = tft.color565(random(0, 256), random(0, 256), random(0, 256));

            // Draw multiple concentric circles to create thickness
            for (int thickness = 0; thickness < CIRCLE_THICKNESS; thickness++)
            {
                tft.drawCircle(centerX, centerY, diameters[i] + thickness, randomColor);
            }
            tft.setTextFont(4); // Set the font
            tft.setCursor(xWelcome, TEXT_Y_WELCOME);
            tft.print("Welcome To");
            tft.setCursor(xController, TEXT_Y_CONTROLLER);
            tft.print("HB9IIU MLA CONTROLLER");
            tft.setTextFont(2);
            tft.setCursor(xVersion, TEXT_Y_VERSION);
            tft.print(versionText);
            tft.setCursor(xDate, TEXT_Y_VERSION + 20);
            tft.print(versionDate);
        }
        // Erase circles by drawing them in the erase color
        for (int i = 0; i < numCircles; i++)
        {
            for (int thickness = 0; thickness < CIRCLE_THICKNESS; thickness++)
            {
                tft.drawCircle(centerX, centerY, diameters[i] + thickness, eraseColor);
            }
            tft.setTextFont(4); // Set the font
            tft.setCursor(xWelcome, TEXT_Y_WELCOME);
            tft.print("Welcome To");
            tft.setCursor(xController, TEXT_Y_CONTROLLER);
            tft.print("HB9IIU MLA CONTROLLER");
            tft.setTextFont(2); // Set the font
            tft.setCursor(xVersion, TEXT_Y_VERSION);
            tft.print(versionText);
            tft.setCursor(xDate, TEXT_Y_VERSION + 20);
            tft.print(versionDate);
        }
    }
    tft.setTextFont(4); // Set the font
    tft.setCursor(xWelcome, TEXT_Y_WELCOME);
    tft.print("Welcome To");
    tft.setCursor(xController, TEXT_Y_CONTROLLER);
    tft.print("HB9IIU MLA CONTROLLER");
    tft.setCursor(xVersion, TEXT_Y_VERSION);
    tft.setTextFont(2); // Set the font
    tft.print(versionText);
    tft.setCursor(xDate, TEXT_Y_VERSION + 20);
    tft.print(versionDate);
    // Delay before clearing the screen
    delay(duration);
    tft.fillScreen(TFT_BLACK); // Clear the screen
}

void printOnTFT(const char *text)
{
    // Static variable to keep track of the Y position across function calls
    static int currentYonTFT = 25; // Start at the top of the display

    tft.setFreeFont(FSS9);                 // Use the bold font
    tft.setTextColor(TFT_CYAN, TFT_BLACK); // Set text color to white, background to black

    if (currentYonTFT > 255) // adjusted manually
    {
        tft.fillScreen(TFT_BLACK); // Clear the screen with a black background
        currentYonTFT = 15;        // Reset Y position to the top of the display
    }

    int lineHeight = 20; // Adjust this value according to your font size (e.g., FreeMonoBold12pt7b)

    // Print the text at the current cursor position
    tft.setCursor(15, currentYonTFT); // Set x to 10, adjust as needed
    tft.println(text);

    // Move to the next line
    currentYonTFT += lineHeight;
}

void establish_WIFI_connection_with_Slave()
{
    Serial.print("Establishing WiFi Connection with Slave...");
    printOnTFT("Connecting To Slave via WiFi...");

    // Attempt to connect to WiFi
    WiFi.begin(SSID);

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
        // Display WiFi signal strength
        int8_t rssi = WiFi.RSSI();                            // Get WiFi RSSI (signal strength)
        String linkQuality = getLinkQualityDescription(rssi); // Ensure this function is defined
        Serial.println("\nSuccessful Wifi Connection to Slave");
        Serial.print("Signal Quality:");
        Serial.println(linkQuality);

        printOnTFT("Wifi Connection to Slave OK");
        String message = "Link Quality: " + linkQuality;
        printOnTFT(message.c_str());

        // Initialize HTTPClient
        String url = String("http://") + slaveIP.toString() + "/command"; // Ensure slaveIP is defined
        http.begin(url);                                                  // Ensure http is defined and initialized
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    }
    else
    {
        Serial.println("\nConnection failed, rebooting in 3 seconds");
        printOnTFT("No WiFi Connection to Slave");
        printOnTFT("Please check your Slave");
        printOnTFT("Re-Booting in 3 seconds");
        delay(3000);
        ESP.restart();
    }
}

bool connectToServer(BLEAddress pAddress)
{
    Serial.print("Establishing a connection to device address: ");
    Serial.println(pAddress.toString().c_str());

    // Convert both to Arduino Strings
    String fullMessage = "Connecting to: " + String(pAddress.toString().c_str());

    // Pass the concatenated string to printOnTFT
    printOnTFT(fullMessage.c_str());

    BLEClient *pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remote BLE Server
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Set maximum MTU (default is 23 otherwise)
    pClient->setMTU(517);

    // Obtain a reference to the Nordic UART service on the remote BLE server
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr)
    {
        Serial.print("Failed to find Nordic UART service UUID: ");
        Serial.println(serviceUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our service");

    // Obtain references to the TX and RX characteristics of the Nordic UART service
    pTXCharacteristic = pRemoteService->getCharacteristic(charUUID_TX);
    if (pTXCharacteristic == nullptr)
    {
        Serial.print("Failed to find TX characteristic UUID: ");
        Serial.println(charUUID_TX.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Remote BLE TX characteristic reference established");

    // std::string value = pTXCharacteristic->readValue();
    // Serial.print("The characteristic value is currently: ");
    // Serial.println(value.c_str());

    pRXCharacteristic = pRemoteService->getCharacteristic(charUUID_RX);
    if (pRXCharacteristic == nullptr)
    {
        Serial.print("Failed to find RX characteristic UUID: ");
        Serial.println(charUUID_RX.toString().c_str());
        return false;
    }
    Serial.println(" - Remote BLE RX characteristic reference established");

    // value = pRXCharacteristic->readValue();
    // Serial.print("The characteristic value was: ");
    // Serial.println(value.c_str());

    // Register notification callback
    if (pTXCharacteristic->canNotify())
        pTXCharacteristic->registerForNotify(notifyCallback);

    uint8_t CIV_ID0[] = {0xFE, 0xF1, 0x00, 0x61, 0x30, 0x30, 0x30, 0x30, 0x31, 0x31, 0x30, 0x31, 0x2D, 0x30, 0x30, 0x30, 0x30, 0x2D, 0x31, 0x30, 0x30, 0x30, 0x2D, 0x38, 0x30, 0x30, 0x30, 0x2D, 0x30, 0x30, 0x38, 0x30, 0x35, 0x46, 0x39, 0x42, 0x33, 0x34, 0x46, 0x42, 0xFD}; // Send our UUID
    pRXCharacteristic->writeValue(CIV_ID0, sizeof(CIV_ID0));
    pRXCharacteristic->canNotify();
    delay(20);

    // hardcoding "IC705-MLA-HB9IIU"
    uint8_t CIV_ID1[] = {0xFE, 0xF1, 0x00, 0x62, 0x49, 0x43, 0x37, 0x30, 0x35, 0x2D, 0x4D, 0x4C, 0x41, 0x2D, 0x48, 0x42, 0x39, 0x49, 0x49, 0x55, 0xFD}; // Send Name

    pRXCharacteristic->writeValue(CIV_ID1, sizeof(CIV_ID1));
    pRXCharacteristic->canNotify();
    delay(20); // a small delay was required or this message would be missed (collision likely).

    // Send Token
    uint8_t CIV_ID2[] = {0xFE, 0xF1, 0x00, 0x63, 0xEE, 0x39, 0x09, 0x10, 0xFD}; // Send Token
    pRXCharacteristic->writeValue(CIV_ID2, 9);
    pRXCharacteristic->canNotify();
    delay(20);

    connected = true;
    return true;
}

// Notification callback function
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
{

    /* Uncomment to analyze the received CAT packets
    for (int i = 0; i < length; i++)
    {
        Serial.print(pData[i], HEX); // Print raw data to UART in hexadecimal format
        Serial.print(" ");
        if (pData[i] == 0xFD) // If the end marker is found, print a new line
            Serial.println();
    }
    */
    // Check for PTT status change
    // Variable to hold the previous PTT status
    static bool previousPTTisON = false; // Use static so it retains value between calls
    if (length == 8 &&
        pData[0] == 0xFE && pData[1] == 0xFE && // Start of the packet
        pData[2] == 0xE0 &&                     // Source address
        pData[3] == radio_address &&            // Verify radio address
        pData[4] == 0x1C &&                     // PTT command identifier
        pData[5] == 0x00)                       // Sub-command for PTT status
    {
        PTTisON = (pData[6] == 0x01); // PTT ON if 0x01, OFF if 0x00
                                      // Check if the status has changed from ON (1) to OFF (0)
        if (previousPTTisON == true && PTTisON == false)
        {
            Serial.println("PTT turned OFF.");
            // Draw a black rectangle covering the bottom part of the screen
            tft.fillRect(0, 190, 320, 240, TFT_BLACK); // Fill the rectangle from (0,y) to (displayWidth, displayHeight)

            displayStepperInfo(currentStepperPosition, deltaSteps);
        }

        // Update previousPTTisON for the next iteration
        previousPTTisON = PTTisON;
    }

    // Check for SWR packet
    if (length == 9 &&
        pData[0] == 0xFE && pData[1] == 0xFE && // Start of the packet
        pData[2] == 0xE0 &&                     // Source address is always 0xE0 in this case (IC-705)
        pData[3] == radio_address &&            // Verify radio address (e.g., 0xA4)
        pData[4] == 0x15 && pData[5] == 0x12)   // SWR command identifier
    {
        // Extract the SWR value (high and low bytes are in pData[6] and pData[7])
        uint8_t swr_high = pData[6];
        uint8_t swr_low = pData[7];
        swrByteValue = (swr_high << 8) | swr_low;
        // Serial.print("SWR Byte Value: ");
        // Serial.println(swrByteValue);
    }

    // Call the function to decode the frequency
    decodeFrequency(pData, length);
}

void displayVFOfrequency(long freq, int x, int y, uint16_t colour)
{
    tft.textfont = 7;   // Set font to 7-segment font
    tft.setTextSize(1); // Set small font size

    static char previousDisplayedfrequency[11]; // No initial value
    static bool isInitialized = false;          // Flag to ensure initialization only happens once

    // Initialize previousDisplayedfrequency only on the first function call
    if (!isInitialized)
    {
        strcpy(previousDisplayedfrequency, "0000000000"); // Initialize with the default placeholder value
        isInitialized = true;                             // Set the flag to true to prevent future initialization

        // Draw the frame only during initialization
        int charWidth = tft.textWidth("00.000.000");
        int frameWidth = charWidth + 20;          // Increase padding for a larger frame
        int frameHeight = tft.fontHeight(7) + 30; // Increase padding for a larger frame
        int frameX = x - 10;                      // Adjust x for larger padding
        int frameY = y - 10;                      // Adjust y for larger padding

        tft.drawRoundRect(frameX, frameY, frameWidth, frameHeight, 10, colour); // Draw rounded rectangle

        // Draw "Resonance Frequency" text at the bottom center of the frame
        tft.setTextFont(4);

        const char *label = "VFO Frequency";

        int labelX = frameX + (frameWidth - tft.textWidth(label)) / 2; // Center the label horizontally
        int labelY = frameY + frameHeight - tft.fontHeight(4) + 13;

        // Draw a black filled rectangle on top of the frame
        tft.fillRect(labelX - 5, labelY, tft.textWidth(label) + 10, tft.fontHeight(4), TFT_BLACK); // Draw filled rectangle

        // Draw the label in the specified color
        tft.setTextColor(colour);                 // Set text color
        tft.drawString(label, labelX, labelY, 4); // Draw the label
    }

    char frequencyArray[11]; // Array to hold the formatted frequency characters

    // Convert the frequency to a string (without decimals)
    String numberStr = String(freq);

    // Prepend leading zeros if the length is less than 8 digits
    while (numberStr.length() < 8)
    {
        numberStr = "0" + numberStr;
    }

    // Fill the frequencyArray with formatted characters (##.###.###)
    frequencyArray[0] = numberStr.charAt(0);
    frequencyArray[1] = numberStr.charAt(1);
    frequencyArray[2] = '.';
    frequencyArray[3] = numberStr.charAt(2);
    frequencyArray[4] = numberStr.charAt(3);
    frequencyArray[5] = numberStr.charAt(4);
    frequencyArray[6] = '.';
    frequencyArray[7] = numberStr.charAt(5);
    frequencyArray[8] = numberStr.charAt(6);
    frequencyArray[9] = numberStr.charAt(7);
    frequencyArray[10] = '\0'; // Null terminator

    int digitX = x; // Start drawing at the specified x position

    tft.textfont = 7; // Set font to 7-segment font

    // Loop through each character in the frequency array
    for (int i = 0; i < 10; i++)
    {
        char newFreqtempChar[2];
        newFreqtempChar[0] = frequencyArray[i];
        newFreqtempChar[1] = '\0';

        char oldFreqtempChar[2];
        oldFreqtempChar[0] = previousDisplayedfrequency[i];
        oldFreqtempChar[1] = '\0';

        // Special case for the first digit: if it's '0', print in black (to hide it)
        if (i == 0 && newFreqtempChar[0] == '0')
        {
            tft.setTextColor(TFT_BLACK);
            tft.drawString(newFreqtempChar, digitX, y, 7);
        }
        // Compare the old and new characters, and update if they are different
        else if (newFreqtempChar[0] != oldFreqtempChar[0])
        {
            // Erase the old digit by printing it in black
            tft.setTextColor(TFT_BLACK);
            tft.drawString(oldFreqtempChar, digitX, y, 7); // Erase the previous character

            // Print the new digit in the specified color
            tft.setTextColor(colour);
            tft.drawString(newFreqtempChar, digitX, y, 7); // Draw the new character
        }

        // Update the x position for the next character
        int charWidth = tft.textWidth(newFreqtempChar); // Get the width of the character
        digitX += charWidth;                            // Move to the next position
    }

    // Copy the current frequency to previousDisplayedfrequency for future comparison
    strcpy(previousDisplayedfrequency, frequencyArray);
}

void displayRESONANCEfrequency(long freq, int x, int y, uint16_t colour)
{
    tft.textfont = 7;   // Set font to 7-segment font
    tft.setTextSize(1); // Set small font size

    static char previousDisplayedfrequency[11]; // No initial value
    static bool isInitialized = false;          // Flag to ensure initialization only happens once

    // Initialize previousDisplayedfrequency only on the first function call
    if (!isInitialized)
    {
        strcpy(previousDisplayedfrequency, "0000000000"); // Initialize with the default placeholder value
        isInitialized = true;                             // Set the flag to true to prevent future initialization

        // Draw the frame only during initialization
        int charWidth = tft.textWidth("00.000.000");
        int frameWidth = charWidth + 20;          // Increase padding for a larger frame
        int frameHeight = tft.fontHeight(7) + 30; // Increase padding for a larger frame
        int frameX = x - 10;                      // Adjust x for larger padding
        int frameY = y - 10;                      // Adjust y for larger padding

        tft.drawRoundRect(frameX, frameY, frameWidth, frameHeight, 10, colour); // Draw rounded rectangle

        // Draw "Resonance Frequency" text at the bottom center of the frame
        tft.setTextFont(4);

        const char *label = "Resonance Frequency";

        int labelX = frameX + (frameWidth - tft.textWidth(label)) / 2; // Center the label horizontally
        int labelY = frameY + frameHeight - tft.fontHeight(4) + 13;

        // Draw a black filled rectangle on top of the frame
        tft.fillRect(labelX - 5, labelY, tft.textWidth(label) + 10, tft.fontHeight(4), TFT_BLACK); // Draw filled rectangle

        // Draw the label in the specified color
        tft.setTextColor(colour);                 // Set text color
        tft.drawString(label, labelX, labelY, 4); // Draw the label
    }

    char frequencyArray[11]; // Array to hold the formatted frequency characters

    // Convert the frequency to a string (without decimals)
    String numberStr = String(freq);

    // Prepend leading zeros if the length is less than 8 digits
    while (numberStr.length() < 8)
    {
        numberStr = "0" + numberStr;
    }

    // Fill the frequencyArray with formatted characters (##.###.###)
    frequencyArray[0] = numberStr.charAt(0);
    frequencyArray[1] = numberStr.charAt(1);
    frequencyArray[2] = '.';
    frequencyArray[3] = numberStr.charAt(2);
    frequencyArray[4] = numberStr.charAt(3);
    frequencyArray[5] = numberStr.charAt(4);
    frequencyArray[6] = '.';
    frequencyArray[7] = numberStr.charAt(5);
    frequencyArray[8] = numberStr.charAt(6);
    frequencyArray[9] = numberStr.charAt(7);
    frequencyArray[10] = '\0'; // Null terminator

    int digitX = x; // Start drawing at the specified x position

    tft.textfont = 7; // Set font to 7-segment font

    // Loop through each character in the frequency array
    for (int i = 0; i < 10; i++)
    {
        char newFreqtempChar[2];
        newFreqtempChar[0] = frequencyArray[i];
        newFreqtempChar[1] = '\0';

        char oldFreqtempChar[2];
        oldFreqtempChar[0] = previousDisplayedfrequency[i];
        oldFreqtempChar[1] = '\0';

        // Special case for the first digit: if it's '0', print in black (to hide it)
        if (i == 0 && newFreqtempChar[0] == '0')
        {
            tft.setTextColor(TFT_BLACK);
            tft.drawString(newFreqtempChar, digitX, y, 7);
        }
        // Compare the old and new characters, and update if they are different
        else if (newFreqtempChar[0] != oldFreqtempChar[0])
        {
            // Erase the old digit by printing it in black
            tft.setTextColor(TFT_BLACK);
            tft.drawString(oldFreqtempChar, digitX, y, 7); // Erase the previous character

            // Print the new digit in the specified color
            tft.setTextColor(colour);
            tft.drawString(newFreqtempChar, digitX, y, 7); // Draw the new character
        }

        // Update the x position for the next character
        int charWidth = tft.textWidth(newFreqtempChar); // Get the width of the character
        digitX += charWidth;                            // Move to the next position
    }

    // Copy the current frequency to previousDisplayedfrequency for future comparison
    strcpy(previousDisplayedfrequency, frequencyArray);
}

// Function to display stepper information
void displayStepperInfo(int currentPos, int stepsToGo)
{

    int x = 12;
    int y = 195;

    static bool labelsDrawn = false; // Static variable to track if labels have been drawn
    if (allStepperInfotoBeRedrawn == true)
    {
        labelsDrawn = false;
        allStepperInfotoBeRedrawn = false;
    }
    // Helper function to format a number with thousand separators for Current Pos
    auto formatCurrentPos = [](int number)
    {
        String numStr = String(abs(number)); // Get the absolute value for formatting
        String formatted = "";

        int len = numStr.length();
        int separatorPos = len % 3;

        // Add thousand separators for Current Pos
        for (int i = 0; i < len; i++)
        {
            formatted += numStr[i];
            if ((i + 1) % 3 == separatorPos && (i + 1) != len)
            {
                formatted += '.';
            }
        }

        // Ensure the result is in #.###.### format (padded with spaces if needed)
        while (formatted.length() < 9)
        { // Ensure format like #.###.###
            formatted = " " + formatted;
        }

        return formatted;
    };

    // Helper function to format Steps to Go with sign and right alignment
    auto formatStepsWithSign = [](int number)
    {
        String numStr = String(abs(number)); // Get the absolute value for formatting
        String formatted = "";

        int len = numStr.length();
        int separatorPos = len % 3;

        // Add thousand separators
        for (int i = 0; i < len; i++)
        {
            formatted += numStr[i];
            if ((i + 1) % 3 == separatorPos && (i + 1) != len)
            {
                formatted += '.';
            }
        }

        // Add the sign at the beginning first
        if (number >= 0)
        {
            formatted = "+" + formatted;
        }
        else
        {
            formatted = "-" + formatted;
        }

        // Ensure the result is in ###.### format (padded with spaces if needed)
        // Ensure proper alignment (9 characters total, including the sign)
        while (formatted.length() < 9)
        {
            formatted = " " + formatted;
        }

        return formatted;
    };

    // Set text properties
    tft.setTextDatum(TL_DATUM);             // Set text alignment to top-left
    tft.setTextFont(4);                     // Set font size (adjust as needed)
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color with background
    tft.setFreeFont(&FreeMonoBold12pt7b);   // Set the FreeMonoBold12pt7b font from TFT_eSPI

    // Only draw the labels once (first time)
    if (!labelsDrawn)
    {
        tft.fillRect(x, y, 320, 240 - y, TFT_BLACK); // Fill background

        tft.drawString("Current Pos: ", x, y);
        tft.drawString("Steps to Go: ", x, y + 25); // Adjust y position for spacing
        labelsDrawn = true;                         // Set the flag so labels are not drawn again
    }

    // Display formatted values
    String formattedPos = formatCurrentPos(currentPos); // Proper format for Current Pos
    tft.drawString(formattedPos, x + 175, y);           // Adjust x position for alignment

    String formattedSteps = formatStepsWithSign(stepsToGo); // Format with sign for Steps to Go
    tft.drawString(formattedSteps, x + 175, y + 25);        // Adjust x position for alignment
}

// Function to draw the progress bar
void drawProgressBar(int duration)
{
    int x = 2;
    int y = 196;
    int givenWidth = 320;
    int height = 35;
    int color = TFT_GREENYELLOW;

    // clear the area
    tft.fillRect(0, y, 480, 320 - y, TFT_BLACK);

    int gap = 2;       // Fixed width of the gap between rectangles
    int rectWidth = 4; // Fixed width of each rectangle

    int totalBarWidth = rectWidth + gap;            // Width occupied by one rectangle and one gap
    int numRectangles = givenWidth / totalBarWidth; // Total number of rectangles that fit

    int lastRectangleRightEdge = x + (numRectangles * totalBarWidth - gap);
    int frameWidth = lastRectangleRightEdge - x;

    // Draw the background for the progress bar frame
    tft.fillRect(x, y, frameWidth, height, TFT_RED); // Fill background with very dark grey

    // Draw the frame for the progress bar
    tft.drawRect(x, y, frameWidth, height, TFT_WHITE); // Draw the white frame

    unsigned long startTime = millis();
    unsigned long currentTime = startTime;

    for (int i = 0; i < numRectangles; i++)
    {

        tft.fillRect(x + i * totalBarWidth + 1, y + 1, rectWidth, height - 2, color);
        currentTime = millis();
        while (currentTime - startTime < duration / numRectangles)
        {
            currentTime = millis();
        }
        startTime = currentTime;
    }

    // Clear the progress bar after completion
    tft.fillRect(0, y, 320, height, TFT_BLACK); // Clear the bar to black
}

void displayMessageAtBottom(const char *message, int font)
{
    int y = 190;                   // Y position of the rectangle's top
    int rectangleHeight = 240 - y; // Height of the rectangle

    // Draw a black rectangle covering the bottom part of the screen
    tft.fillRect(0, y, 320, rectangleHeight, TFT_BLACK); // Fill the rectangle from (0,y) to (displayWidth, displayHeight)

    static int blinkState = 0; // Static variable to retain state

    // Set the free font
    if (font == 1)
    {
        tft.setFreeFont(FMB24);
    }
    else
    {
        tft.setFreeFont(FMB18);
    }

    // Get the height of the currently selected font
    int textHeight = tft.fontHeight();

    // Get the width of the message text
    int textWidth = tft.textWidth(message); // Get the width of the text

    // Calculate the starting X position to center the text
    int x = (320 - textWidth) / 2; // Center the text in a 320-pixel wide display

    // Toggle text color
    if (blinkState == 0)
    {
        tft.setTextColor(TFT_RED, TFT_BLACK); // Set text color to red
    }
    else
    {
        tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set text color to dark grey (invisible)
    }

    // Update the blink state
    blinkState = 1 - blinkState; // Toggle between 0 and 1

    // Position the cursor for text
    tft.setCursor(x, 230); // Use textY to center vertically
    tft.print(message);    // Print the passed message using the selected font

    delay(200);
}

void updateWiFiWidget(int x, int y, int radius, float sizePercentage, int rssi)
{
    // int rssi=WiFi.RSSI();

    static int previousRSSILevel = -1; // Track the previous RSSI level to avoid unnecessary redraws

    // Function to convert degrees to radians
    auto degToRad = [](float deg)
    {
        return deg * 3.14159265 / 180;
    };

    // Function to fill a 90° arc between two radii
    auto fillArcWithTriangles = [&](int x, int y, int startRadius, int endRadius, int startAngle, int endAngle, uint16_t color)
    {
        for (int r = startRadius; r < endRadius; r++)
        {
            for (int angle = startAngle; angle <= endAngle; angle++)
            {
                float angleRad1 = degToRad(angle);
                float angleRad2 = degToRad(angle + 1); // Draw small segments between angles

                // Calculate the coordinates for the outer and inner points of the arc
                int x1Outer = x + r * cos(angleRad1);
                int y1Outer = y + r * sin(angleRad1);
                int x2Outer = x + r * cos(angleRad2);
                int y2Outer = y + r * sin(angleRad2);

                int x1Inner = x + (r + 1) * cos(angleRad1);
                int y1Inner = y + (r + 1) * sin(angleRad1);
                int x2Inner = x + (r + 1) * cos(angleRad2);
                int y2Inner = y + (r + 1) * sin(angleRad2);

                // Draw triangles between the outer and inner points of the arc
                tft.fillTriangle(x1Outer, y1Outer, x2Outer, y2Outer, x1Inner, y1Inner, color);
                tft.fillTriangle(x1Inner, y1Inner, x2Inner, y2Inner, x2Outer, y2Outer, color);
            }
        }
    };

    // Map RSSI value to the number of filled sectors (0 to 4)
    int rssiLevel;
    if (rssi > -50)
        rssiLevel = 4; // Excellent signal
    else if (rssi > -60)
        rssiLevel = 3; // Good signal
    else if (rssi > -70)
        rssiLevel = 2; // Fair signal
    else if (rssi > -80)
        rssiLevel = 1; // Weak signal
    else
        rssiLevel = 0; // Very weak or no signal

    if (rssiLevel == previousRSSILevel)
        return; // If RSSI level hasn't changed, do nothing

    // Scale the radius by the provided sizePercentage
    int scaledRadius = radius * (sizePercentage / 100.0);

    // Define color for active and inactive sectors
    uint16_t activeColor = TFT_GREEN;
    uint16_t inactiveColor = TFT_DARKGREY;

    // Update only the sectors that have changed
    for (int i = 1; i <= 4; i++)
    {
        uint16_t color = (rssiLevel >= i) ? activeColor : inactiveColor;
        if ((previousRSSILevel >= i && rssiLevel < i) || (previousRSSILevel < i && rssiLevel >= i))
        {
            if (i == 1)
            {
                tft.fillCircle(x, y, scaledRadius * 1 / 7, color); // Center dot
            }
            else if (i == 2)
            {
                fillArcWithTriangles(x, y, scaledRadius * 2 / 7, scaledRadius * 3 / 7, 225, 315, color); // First arc
            }
            else if (i == 3)
            {
                fillArcWithTriangles(x, y, scaledRadius * 4 / 7, scaledRadius * 5 / 7, 225, 315, color); // Second arc
            }
            else if (i == 4)
            {
                fillArcWithTriangles(x, y, scaledRadius * 6 / 7, scaledRadius * 7 / 7, 225, 315, color); // Third arc
            }
        }
    }

    // Update the previous RSSI level
    previousRSSILevel = rssiLevel;

    // Display the dB level below the icon
    tft.setFreeFont(FMB18);                 // Set small font size
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background to clear previous text
    tft.setCursor(x - 33, y + 53);          // Position the text below the Wi-Fi icon

    tft.printf("%d", rssi);        // Print the RSSI value in dBm
    tft.setCursor(x - 33, y + 80); // Position the text below the Wi-Fi icon
    tft.printf("dBm");             // Print the RSSI value in dBm
}


// SWR _--------------------------------------------------------

// Function to draw tick marks and labels for SWR values (1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0)
void SWR_drawTicksAndLabels()
{
    // Tick values and corresponding positions on the bar
    float tick_values[] = {1.0, 1.2, 1.5, 2.0, 2.5, 3.0};
    int num_ticks = sizeof(tick_values) / sizeof(tick_values[0]);

    for (int i = 0; i < num_ticks; i++)
    {
        // Get the tick position based on logarithmic scaling
        float log_swr_scaled = SWR_applyLogarithmicScale(tick_values[i]);
        int tick_x = SWR_BAR_X + (log_swr_scaled * SWR_BAR_WIDTH);

        // Draw the tick mark
        tft.drawLine(tick_x, SWR_BAR_Y + SWR_BAR_HEIGHT, tick_x, SWR_BAR_Y + SWR_BAR_HEIGHT + SWR_TICK_HEIGHT, SWR_COLOR_TICK);

        // Draw the label
        tft.setFreeFont(&FreeMonoBold9pt7b);
        tft.setTextColor(SWR_COLOR_TICK, SWR_COLOR_BG);
        tft.drawString(String(tick_values[i], 1), tick_x - 10, SWR_BAR_Y + SWR_BAR_HEIGHT + SWR_TICK_HEIGHT +3);
    }
}

// Function to map SWR to a bar length with logarithmic scaling
int SWR_mapSWRToBarLength(float swr)
{
    if (swr <= 1.0)
        return 0;

    float log_swr_scaled = SWR_applyLogarithmicScale(swr);
    int bar_length = log_swr_scaled * SWR_BAR_WIDTH;

    // Ensure full bar is drawn for SWR = 4.0
    if (swr >= 4.0)
    {
        bar_length = SWR_BAR_WIDTH;
    }

    return bar_length;
}

// Function to draw segments (white or black based on SWR range)
void SWR_drawSegment(int x, int bar_length, int green_length)
{
    if (x <= SWR_BAR_X + bar_length)
    {
        if (x <= SWR_BAR_X + green_length) // Green portion
        {
            tft.drawLine(x, SWR_BAR_Y, x, SWR_BAR_Y + SWR_BAR_HEIGHT, SEGMENT_COLOR_BLACK);
        }
        else // Other portions
        {
            tft.drawLine(x, SWR_BAR_Y, x, SWR_BAR_Y + SWR_BAR_HEIGHT, SEGMENT_COLOR);
        }
    }
}

// Function to erase segments and bar area when the SWR decreases
void SWR_eraseBar(int from_x, int to_x)
{
    tft.fillRect(from_x, SWR_BAR_Y, to_x - from_x, SWR_BAR_HEIGHT, SWR_COLOR_BG);
}

// Function to handle drawing the segments in the filled area
void SWR_drawSegmentsInFilledArea(int bar_length, int green_length)
{
    // Draw segments within the current filled area
    for (int x = SWR_BAR_X + SEGMENT_WIDTH; x <= SWR_BAR_X + bar_length; x += SEGMENT_WIDTH)
    {
        SWR_drawSegment(x, bar_length, green_length);
    }

    // Add a partially filled segment if needed
    if (bar_length % SEGMENT_WIDTH != 0)
    {
        int partial_segment_x = SWR_BAR_X + ((bar_length / SEGMENT_WIDTH) * SEGMENT_WIDTH) + SEGMENT_WIDTH;
        if (partial_segment_x <= SWR_BAR_X + SWR_BAR_WIDTH)
        {
            SWR_drawSegment(partial_segment_x, bar_length, green_length);
        }
    }
}

// Function to draw the SWR meter bar with minimal flickering and correct segment handling
void SWR_drawSWRBar(float swr)
{
    // First, draw the background and ticks once

    if (SWR_first_draw)
    {
        tft.fillRect(0, SWR_BAR_Y, 480, 320 - SWR_BAR_Y, SWR_COLOR_BG); // Draw background

        SWR_drawTicksAndLabels();
        SWR_first_draw = false;
    }

    // Map the SWR values to bar lengths
    int current_length = SWR_mapSWRToBarLength(swr);
    int previous_length = SWR_mapSWRToBarLength(previous_swr);
    int green_length = SWR_mapSWRToBarLength(2.0);

    // Draw or erase portions based on whether SWR increases or decreases
    if (swr > previous_swr) // SWR increases
    {
        // Draw the green portion (up to SWR = 2.0)
        if (current_length > green_length)
        {
            tft.fillRect(SWR_BAR_X, SWR_BAR_Y, green_length, SWR_BAR_HEIGHT, SWR_COLOR_GOOD);
        }
        else
        {
            tft.fillRect(SWR_BAR_X, SWR_BAR_Y, current_length, SWR_BAR_HEIGHT, SWR_COLOR_GOOD);
        }

        // Draw the orange portion (between SWR = 2.0 and SWR = 3.0)
        if (swr > 2.0)
        {
            int orange_length = SWR_mapSWRToBarLength(3.0) - green_length;
            if (current_length > green_length)
            {
                if (current_length <= SWR_mapSWRToBarLength(3.0))
                {
                    tft.fillRect(SWR_BAR_X + green_length, SWR_BAR_Y, current_length - green_length, SWR_BAR_HEIGHT, COLOR_WARNING);
                }
                else
                {
                    tft.fillRect(SWR_BAR_X + green_length, SWR_BAR_Y, orange_length, SWR_BAR_HEIGHT, COLOR_WARNING);
                }
            }
        }

        // Draw the red portion (above SWR = 3.0)
        if (swr > 3.0)
        {
            int red_length = current_length - SWR_mapSWRToBarLength(3.0);
            tft.fillRect(SWR_BAR_X + SWR_mapSWRToBarLength(3.0), SWR_BAR_Y, red_length, SWR_BAR_HEIGHT, COLOR_DANGER);
        }
    }
    else if (swr < previous_swr) // SWR decreases
    {
        // Erase the excess part of the bar when SWR decreases
        SWR_eraseBar(SWR_BAR_X + current_length, SWR_BAR_X + previous_length);

        // Erase any segments beyond the current length
        for (int x = SWR_BAR_X + current_length + SEGMENT_WIDTH; x <= SWR_BAR_X + previous_length; x += SEGMENT_WIDTH)
        {
            SWR_eraseBar(x, x + SEGMENT_WIDTH); // Clear segments that should be removed
        }
    }

    // Draw the outline of the entire bar
    tft.drawRect(SWR_BAR_X, SWR_BAR_Y, SWR_BAR_WIDTH, SWR_BAR_HEIGHT, TFT_WHITE);

    // Erase previous SWR value by printing it in black
    tft.setFreeFont(&FreeMonoBold18pt7b);
    String prevSWRText = String(previous_swr, 1);
    int text_x = SWR_BAR_X + SWR_BAR_WIDTH + 10;
    int text_y = SWR_BAR_Y + 8;
    tft.setTextColor(SWR_COLOR_BG, TFT_BLACK); // Print in background color to erase
    tft.drawString(prevSWRText, text_x, text_y);

    // Print new SWR value
    String swrText = String(swr, 1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(swrText, text_x, text_y);

    // Update the previous SWR value
    previous_swr = swr;

    // After everything is drawn, apply the segment effect in the filled area
    SWR_drawSegmentsInFilledArea(current_length, green_length);
}

// Function to map SWR to the logarithmic scale (1.0 to 4.0 SWR)
float SWR_applyLogarithmicScale(float swr)
{
    if (swr > 4.0)
        swr = 4.0; // Cap SWR at 4.0
    if (swr < 1.0)
        swr = 1.0; // Ensure SWR does not go below 1.0

    // Logarithmic scale between 1.0 and 4.0
    float log_swr_min = log10(1.0);
    float log_swr_max = log10(4.0);

    // Return scaled value (between 0 and 1)
    return (log10(swr) - log_swr_min) / (log_swr_max - log_swr_min);
}


void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[320];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(0, 0 + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}
void pngSplashScreen() {
int16_t rc = png.openFLASH((uint8_t *)panda, sizeof(panda), pngDraw);
  if (rc == PNG_SUCCESS) {
    Serial.println("Successfully opened png file");
    Serial.printf("image specs: (%d x %d), %d bpp, pixel type: %d\n", png.getWidth(), png.getHeight(), png.getBpp(), png.getPixelType());
    tft.startWrite();
    uint32_t dt = millis();
    rc = png.decode(NULL, 0);
    Serial.print(millis() - dt); Serial.println("ms");
    tft.endWrite();
}}