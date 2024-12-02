// Introduction:
// This code was written for a standard 4MB ESP32 development board.
// It serves as the GUI for an automatic magnetic antenna loop tuner,
// communicating with another ESP32 over WiFi in AP mode.
// The companion ESP32 controls a stepper motor that drives a capacitor,
// tuning based on frequencies received from an Icom IC-705 radio via Bluetooth.
//
// License:
// Feel free to use, modify, and share sections or the full code for personal projects or educational purposes.
// However, please don’t use this code to create a questionable gadget for AliExpress!
// Let’s keep it for fun, learning, and the joy of tinkering – not mass production.
//
// Disclaimer:
// This code is written by a novice, so it may not follow the most elegant or optimized structure.
// Experts may spot improvements, while fellow beginners might find it relatable!
// Use at your own risk – it's an adventure.
//
// For questions, you can reach me at daniel.staempfli@gmail.com.
// However, please note I don’t have much time for support, so responses may be slow.
//
// Credits:
// IC-705-BLE, BT Classic, and USB Host Serial Examples
// Repository: https://github.com/K7MDL2/IC-705-BLE-Serial-Example?tab=readme-ov-file
//
// IC-705 CI-V Reference Guide
// Document: https://www.icomeurope.com/wp-content/uploads/2020/08/IC-705_ENG_CI-V_1_20200721.pdf
//
// Source of Inspiration:
// "Automatically Tune a Magnetic Loop Antenna" - https://sites.google.com/site/lofturj/13-to-automatically-tune-a-magnetic-loop-antenna
//
// Using TFT_eSPI Library With Visual Studio Code and PlatformIO on an ESP32 Microcontroller
// Guide: https://www.instructables.com/Using-TFTeSPI-Library-With-Visual-Studio-Code-and-/
//
// TFT Display
// 320x480 SPI module 4.0/3.95 inch TFT LCD display screen ILI9488
// Connection guide:: see platformio.ini
// https://www.aliexpress.com/item/1005005789916865.html?spm=a2g0o.order_list.order_list_main.17.1d8018024uAQsP
//
// Housing: https://github.com/HB9IIU/MLA-Controller-Master
// ########################################################################################

// ########################################################################################
#include "config.h"
#include <Arduino.h>
#include <TFT_eSPI.h>      // Include the TFT_eSPI library
TFT_eSPI tft = TFT_eSPI(); // Initialize the TFT display

uint16_t initialRFPower;

// Define timing constants
const unsigned long DOUBLE_CLICK_MAX_INTERVAL = 200; // Max interval (ms) for double click
const unsigned long LONG_PRESS_THRESHOLD = 500;      // Threshold (ms) for long press

// Variables to track PTT state and timing
bool lastPTTState = false;         // Stores the last PTT state (false = OFF, true = ON)
unsigned long lastPressTime = 0;   // Time when the PTT was last pressed (ON)
unsigned long lastReleaseTime = 0; // Time when the PTT was last released (OFF)
bool possibleDoubleClick = false;  // Flag to check for potential double-click

void checkPTT(bool currentPTTState);

bool isFT8ButtonPressed = false;  // Global variable to track button state
bool isWSPRButtonPressed = false; // Global variable to track button state
//--------------------------------------------------------------------------------------------------------
// for new Linear SWR meter added 05.11.2024
const int segmentWidth = 14;   // Width of each segment
const int segmentGap = 1;      // Gap between segments
const int segmentHeight = 40;  // Height of each segment
const int meterX = 20;         // X position of the top-left corner of the meter
const int numSegments = 25;    // Number of segments in the SWR meter
const int tickLength = 15;     // Adjustable length of tick marks
float previousSWRValue = -1.0; // Variable to store previous SWR value for erasing
int meterY;                    // Y position of the top-left corner of the meter, calculated dynamically

//--------------------------------------------------------------------------------------------------------
//  related to Buttons on Main Page
#include <FS.h>
#include <TFT_eWidget.h> // Widget library
#define CALIBRATION_FILE "/TouchCalData"
#define REPEAT_CAL false
ButtonWidget buttonSETonMainPage = ButtonWidget(&tft);
ButtonWidget buttonPTTonMainPage = ButtonWidget(&tft);
ButtonWidget buttonFT8onMainPage = ButtonWidget(&tft);
ButtonWidget buttonWSPRonMainPage = ButtonWidget(&tft);
ButtonWidget *btn[] = {&buttonSETonMainPage, &buttonPTTonMainPage, &buttonFT8onMainPage, &buttonWSPRonMainPage};
uint8_t buttonCount = sizeof(btn) / sizeof(btn[0]);

//--------------------------------------------------------------------------------------------------------
// related to BLUETOOTH
#include "BLEDevice.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

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
uint8_t radio_address = 0xA4;                // A4 for IC-705
const char *deviceName = "IC705-MLA-HB9IIU"; // Bluetooth Device Name

//--------------------------------------------------------------------------------------------------------
// WIFI RElated
#include <WiFi.h>
#include <HTTPClient.h>
#define SSID "HB9IIU-MLA"          // WiFi SSID to connect to
IPAddress slaveIP(192, 168, 4, 1); // IP address of the SLAVE device to send HTTP commands to
HTTPClient http;                   // HTTPClient instance
//--------------------------------------------------------------------------------------------------------
// Define the display area for the SWR meter
#define SWR_BAR_X 20       // X position of the bar
#define SWR_BAR_Y 245      // Y position of the bar
#define SWR_BAR_WIDTH 380  // Width of the bar
#define SWR_BAR_HEIGHT 40  // Height of the bar
#define SWR_TICK_HEIGHT 10 // Height of tick marks
#define SEGMENT_WIDTH 8    // Width of the segments (spacing between vertical lines)

// Define the colors
#define SWR_COLOR_GOOD TFT_GREENYELLOW
#define COLOR_WARNING TFT_ORANGE
#define COLOR_DANGER TFT_RED
#define SWR_COLOR_BG TFT_BLACK
#define SWR_COLOR_TICK TFT_WHITE
#define SEGMENT_COLOR TFT_WHITE       // Color for the segment lines
#define SEGMENT_COLOR_BLACK TFT_BLACK // Color for segment lines in the green portion

// Store the previous SWR value to compare for optimization
float previous_swr = -1.0;
uint16_t swrByteValue; // from CAT message
// Store a flag for the first draw
bool SWR_first_draw = true;
float swr;
bool PTTisPressed = false;
//--------------------------------------------------------------------------------------------------------

//
bool allStepperInfotoBeRedrawn = true;
bool freqIsOutOfRange = false;
// bool DisplayAndManageMainPage = true;
// bool DisplayKeyPadPage = false;

unsigned long doubleClickThreshold = 1100; // Adjust the time threshold as needed (in milliseconds)
bool doublePTTdetected = false;
bool weAreInTheMainPage = true;
int64_t theoreticalResonanceFrequency = 0; // theoretical resonance

// Declare global variables for mode and filter
uint8_t currentMode = 0;   // Will store the current mode value
uint8_t currentFilter = 0; // Will store the current filter value

unsigned long CurrentVFOFrequency = 0;                // VFO frequency in Hz
unsigned long previousVFOFrequency = 0;               // to decide wether to refresh display or not
unsigned long previousMillisForWiFiStrengthCheck = 0; // Store the last time the function was called
unsigned long previousMillisForKeypadScan = 0;        // Store the last time the function was called

uint32_t currentStepperPosition = 0;   // current stepper position
uint32_t predictedStepperPosition = 0; // predicted stepper position from lookup table
uint32_t deltaSteps = 0;               // diff between current stepper position and predicted stepper position
long estimated_movement_duration_in_microseconds;

// Define the frequency ranges
#define LOWER_40M 7000000
#define UPPER_40M 7300000
#define LOWER_20M 14000000
#define UPPER_20M 14350000

// Define the FT8 frequencies for 40m and 20m bands
#define FT8_40M 7074000
#define FT8_20M 14074000

// Define the WSPR frequencies for 40m and 20m bands
#define WSPR_40M 7040100
#define WSPR_20M 14097100

//--------------------------------------------------------------------------------------------------------
// Function declarations
void displayVFOfrequency(long freq, int x, int y, uint16_t colour);
void displayRESONANCEfrequency(long freq, int x, int y, uint16_t colour);
void drawProgressBar(int x, int y, int givenWidth, int height, int duration);
void updateWiFiWidget(int x, int y, int radius, float sizePercentage, int rssi);
void displayStepperInfo(int x, int y, int currentPos, int stepsToGo); // full block displaying Current Pos. & Steps to go

// For the Buttons
void buttonSETonMainPage_pressAction(void);
void buttonSETonMainPage_releaseAction(void);
void buttonPTTonMainPage_pressAction(void);
void buttonPTTonMainPage_releaseAction(void);
void buttonFT8onMainPage_pressAction(void);
void buttonFT8onMainPage_releaseAction(void);
void buttonWSPRonMainPage_pressAction(void);
void buttonWSPRonMainPage_releaseAction(void);

//
void initButtonsOnMainPage(void);
void calibrateTouchDisplay(void);
//
void printOnTFT(const char *text, uint16_t textColor, uint16_t backgroundColor); // to display console type info
//
void establish_WIFI_connection_with_Slave();
void getLinkQuality(int8_t rssi, String &description, uint16_t &color);

//

void TuneOnDoublePTTclick();
void decodeFrequency(uint8_t *pData, size_t length);
char *formatFrequencyForConsoleOutput(uint64_t vfo);
void getStepperPositionForCurrentVFOfrequency(uint32_t currentVFOfrequency);
String formatStepperPosForConsoleOutput(uint32_t value);
String formatStepsToGoForConsole(int32_t value);
String sendCommandToSlave(const String &command, const String &argument);
void GetTunedStatusFromSlave();
void setNewPositionForCurrentVFOfrequency(uint32_t targetFrequency);

bool isSlaveConnected();
void set_IC705_frequency(uint32_t frequency);
// KEYPAD RELATED (Page 2)
void drawKeypad();
void displayInfoTextOnRightPane();
void formatRightAlignedNumber(char *buffer, char *numberBuffer, int maxLength);
unsigned long convertBufferToNumber(char *buffer);
int drawTextWithWordWrap(const char *text, int x, int y, int maxX, int lineHeight);
void checkKeypadPage();
void displayWelcomeScreen(int duration, const char *version, const char *date);
void displayWelcomeScreenSimple(int duration, const char *version, const char *date);
void setNewStepperPosToSlavePreferencesForGivenFrequency(uint32_t newVFOFrequency);
void displayRebootingMessageScreen();
//
bool isTouchPointInRegion(int t_x, int t_y, int x1, int y1, int x2, int y2);
//
void displayOutofRangeMessage(int x, int y);
void drawBlackRectangleToErasePortionOfScreen(int x1, int y1, int x2, int y2);
bool backInRange = true;
float SWR_mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float SWR_applyLogarithmicScale(float swr);
void SWR_drawTicksAndLabels();
int SWR_mapSWRToBarLength(float swr);
void SWR_drawSegment(int x, int bar_length, int green_length);
void SWR_eraseBar(int from_x, int to_x);
void SWR_drawSegmentsInFilledArea(int bar_length, int green_length);
void SWR_drawLogarithmicSWRbar(float swr);
void SWR_drawLinearSWRbar(float swr);
void test();

//  Keypad start position, key sizes and spacing
#define KEY_X 45 // Centre of key
#define KEY_Y 108
#define KEY_W 72 // Width and height
#define KEY_H 48
#define KEY_SPACING_X 20 // X and Y gap
#define KEY_SPACING_Y 13

// Using two fonts since numbers are nice when bold
#define buttonFont &FreeSansBold12pt7b // Key label fonts

// Numeric display box size and location
#define DISP_X 4
#define DISP_Y 4
#define DISP_W 270
#define DISP_H 70

TFT_eSPI_Button exitButton; // Declare a button object

// Number length, buffer for storing it and character index
#define NUM_LEN 8
char numberBuffer[NUM_LEN + 1] = "";
uint8_t numberIndex = 0;

// Create 12 keys for the keypad
char keyLabel[12][5] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "CLS", "Set"};
uint16_t keyColor[12] = {TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLUE, TFT_BLUE,
                         TFT_BLUE, TFT_BLACK, TFT_BLACK};

// Invoke the TFT_eSPI button class and create all the button objects
TFT_eSPI_Button key[12];

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
//--------------------------------------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200); // Start Serial communication for debugging
    Serial.println("HB9IIU MLA Controller Starting");
    pinMode(TFT_BLP, OUTPUT);    // Configure the backlight pin
    digitalWrite(TFT_BLP, HIGH); // Turn on the backlight

    tft.begin();               // Initialize the TFT display
    tft.fillScreen(TFT_BLACK); // Set background to black
                               // tft.fillScreen(TFT_WHITE); // Set background to black

    tft.setRotation(1); // Set rotation if needed

    calibrateTouchDisplay();

    if (WROOM32U == true)
    { // because we have memory issues to display welcome with th U
        displayWelcomeScreenSimple(1500, VERSION, RELEASE_DATE);
    }
    else
    {
        displayWelcomeScreen(1500, VERSION, RELEASE_DATE);
    }

    printOnTFT("HB9IIU MLA Controller Starting", TFT_WHITE, TFT_BLACK);

    Serial.println("Sketch Size: " + String(ESP.getSketchSize()) + " bytes");
    Serial.println("Free Sketch Space: " + String(ESP.getFreeSketchSpace()) + " bytes");
    Serial.println("Flash Chip Size: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB");
    Serial.println("Flash Frequency: " + String(ESP.getFlashChipSpeed() / 1000000) + " MHz");

    // Print chip model
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Chip model: %s\n", (chip_info.model == CHIP_ESP32) ? "ESP32" : "Unknown");
    // Print number of cores
    Serial.printf("Number of cores: %d\n", chip_info.cores);
    // Print chip revision
    Serial.printf("Chip revision: %d\n", chip_info.revision);

    establish_WIFI_connection_with_Slave();
    // Call the function 10 times with a 1-second interval
    /*
       for (int i = 0; i < 100000; i++) {
           Serial.println(isSlaveConnected());
           delay(0); // Wait for 1 second (1000 milliseconds)
       }
   */

    Serial.println("Scanning for Bluetooth devices");
    printOnTFT("Scanning for Bluetooth devices", TFT_WHITE, TFT_BLACK); // Print the message on the TFT display

    BLEDevice::init("");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    BLEScanResults foundDevices = pBLEScan->start(5, false);

    String message = "Found Bluetooth devices: " + String(foundDevices.getCount());
    printOnTFT(message.c_str(), TFT_WHITE, TFT_BLACK); // Use c_str() to convert the String to a C-style string
    Serial.println("Scan done!");
    printOnTFT("BLE devices Scan done!", TFT_GREEN, TFT_BLACK);
    Serial.print("Found Bluetooth devices: ");
    Serial.println(foundDevices.getCount());

    if (!doConnect)
    {
        Serial.print("Scan did not reveal device: ");
        Serial.println(deviceName);
        String message = String(deviceName) + " not found";
        printOnTFT(message.c_str(), TFT_RED, TFT_BLACK);
    }
    else
    {
        printOnTFT("IC705 Found!", TFT_GREEN, TFT_BLACK);
    }
}

void loop()
{
    // Rescan only if not connected and doConnect is false
    if (!connected && !doConnect)
    {
        Serial.println("Rescanning for device...");
        printOnTFT("Rescanning for device...", TFT_WHITE, TFT_BLACK);

        BLEScan *pBLEScan = BLEDevice::getScan();
        pBLEScan->clearResults(); // Clear previous scan results
        BLEScanResults foundDevices = pBLEScan->start(5, false);

        Serial.print("Devices found: ");
        Serial.println(foundDevices.getCount());
        // Combine the text with the device count
        String message = "Devices found: " + String(foundDevices.getCount());
        // Print the combined message on the TFT display
        printOnTFT(message.c_str(), TFT_WHITE, TFT_BLACK);
        Serial.println("Rescan done!");
        printOnTFT("Rescan done!", TFT_GREEN, TFT_BLACK);

        if (!doConnect)
        {
            Serial.print("IC705 still not found: ");
            printOnTFT("IC705 still not found!", TFT_RED, TFT_BLACK);
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
            printOnTFT("Connection to BLE Server is OK!", TFT_GREEN, TFT_BLACK);
            printOnTFT("        We can start !!!", TFT_GREEN, TFT_BLACK);
            delay(2000);
            tft.fillScreen(TFT_BLACK); // Set background to black

            // Update the Wi-Fi icon based on the new RSSI level
            updateWiFiWidget(60, 50, 70, 50, WiFi.RSSI());
            initButtonsOnMainPage();

            //   getting VFO frequency for th e1st time
            uint8_t CIV_frequency[] = {0xFE, 0xFE, radio_address, 0xE0, 0x03, 0xFD};
            pRXCharacteristic->writeValue(CIV_frequency, sizeof(CIV_frequency));
            delay(80);

            // display fake value to initiate
            displayVFOfrequency(1111111111, 160, 20, VFOFrequDisplayColor);
            displayVFOfrequency(CurrentVFOFrequency, 160, 20, VFOFrequDisplayColor);
            GetTunedStatusFromSlave(); // to get current stepper pos and theoretical resonance freq

            // display fake value to initiate
            displayRESONANCEfrequency(1111111111, 160, 130, ResonanceFrequDisplayColor);
            displayRESONANCEfrequency(theoreticalResonanceFrequency, 160, 130, ResonanceFrequDisplayColor);
        }
        else
        {
            Serial.println("We have failed to connect to the server.");
            printOnTFT("Failed Connection to BLE Server", TFT_WHITE, TFT_BLACK);
        }
        doConnect = false; // Ensure we don’t attempt to reconnect immediately
    }

    // If connected, perform BLE communication
    if (connected)
    {
        if (weAreInTheMainPage == true)
        {
            // updating wifi widget every 5 seconds only
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillisForWiFiStrengthCheck >= 5000)
            {
                previousMillisForWiFiStrengthCheck = currentMillis;
                Serial.print("Updating Wifi widget: ");
                Serial.println(WiFi.RSSI());
                updateWiFiWidget(60, 50, 70, 50, WiFi.RSSI());
            }

            // Check if still connected
            if (pTXCharacteristic == nullptr || !pTXCharacteristic->getRemoteService()->getClient()->isConnected())
            {
                Serial.println("Connection lost. Attempting to reconnect...");
                tft.fillScreen(TFT_BLACK); // Set background to black
                printOnTFT("Bluetooth connection lost!", TFT_WHITE, TFT_BLACK);
                printOnTFT("Attempting to reconnect...", TFT_WHITE, TFT_BLACK);
                // reconnectFlag = true; // to force VFO frequency display for unchanged frequency
                connected = false;
                doConnect = true; // Reset flags to trigger re-scan
                delay(1000);      // Delay before retrying
                return;
            }

            //  Continue normal communication if still connected
            //  Send the CIV command to get VFO frequency
            uint8_t CIV_frequency[] = {0xFE, 0xFE, radio_address, 0xE0, 0x03, 0xFD};
            pRXCharacteristic->writeValue(CIV_frequency, sizeof(CIV_frequency));
            delay(10);

            // Send the CIV command to get the PTT status
            uint8_t CIV_PTT_Status[] = {0xFE, 0xFE, radio_address, 0xE0, 0x1C, 0x00, 0xFD};
            pRXCharacteristic->writeValue(CIV_PTT_Status, sizeof(CIV_PTT_Status));
            delay(10);
            displayVFOfrequency(CurrentVFOFrequency, 160, 20, VFOFrequDisplayColor);
            getStepperPositionForCurrentVFOfrequency(CurrentVFOFrequency);

            // Scanning touchscreen to detect interaction
            currentMillis = millis(); // Get the current time

            uint16_t t_x = 9999, t_y = 9999; // To store the touch coordinates
            if (currentMillis - previousMillisForKeypadScan >= 50)
            {
                // Detect if there’s a valid touch
                bool pressed = tft.getTouch(&t_x, &t_y);

                if (pressed)
                {
                    // Serial.println("Pressed");
                    //  Check for specific touch regions
                    if (isTouchPointInRegion(t_x, t_y, 145, 213, 480, 320))
                    { // Bottom third of the screen
                        TuneOnDoublePTTclick();
                    }
                    if (isTouchPointInRegion(t_x, t_y, 0, 0, 100, 100)) // uppeer left corner for test
                    {                                                   // Bottom third of the screen
                        test();
                    }
                    else if (isTouchPointInRegion(t_x, t_y, 145, 0, 480, 213))
                    { // Top two-thirds of the screen
                        btn[1]->press(true);
                        btn[1]->pressAction();
                    }

                    // Loop through buttons to detect and trigger actions
                    for (uint8_t b = 0; b < buttonCount; b++)
                    {
                        if (btn[b]->contains(t_x, t_y))
                        {
                            btn[b]->press(true);
                            btn[b]->pressAction();
                        }
                    }
                }
                else
                {

                    // Serial.println("Not Pressed");

                    // Release all buttons if no touch is detected
                    for (uint8_t b = 0; b < buttonCount; b++)
                    {
                        btn[b]->press(false);
                        btn[b]->releaseAction();
                    }
                }

                previousMillisForKeypadScan = currentMillis;
            }

            if (doublePTTdetected == true)
            {
                doublePTTdetected = false;
                TuneOnDoublePTTclick();
            }
            if (PTTisPressed == true)
            {

                // Send the CIV command to read the selected band’s RF power
                uint8_t CIV_Read_RF_Power[] = {0xFE, 0xFE, radio_address, 0xE0, 0x14, 0x0A, 0xFD};
                pRXCharacteristic->writeValue(CIV_Read_RF_Power, sizeof(CIV_Read_RF_Power));
                delay(50);

                // Send the CI-V command to set the RF power level to 10% (0x26)
                uint8_t CIV_Set_RF_Power[] = {0xFE, 0xFE, radio_address, 0xE0, 0x14, 0x0A, 0x00, 0x26, 0xFD};
                pRXCharacteristic->writeValue(CIV_Set_RF_Power, sizeof(CIV_Set_RF_Power));
                delay(50); // Allow some time for the command to be sent

                // Send the CIV command to get the MODE and FILTER
                uint8_t CIV_Get_Mode_Filter[] = {0xFE, 0xFE, radio_address, 0xE0, 0x04, 0xFD};
                pRXCharacteristic->writeValue(CIV_Get_Mode_Filter, sizeof(CIV_Get_Mode_Filter));
                delay(10);

                // To set MODE to FM and FILTER to FIL1 (will revert when button is released)
                uint8_t CIV_Set_Mode_Filter[] = {0xFE, 0xFE, radio_address, 0xE0, 0x01, 0x05, 0x01, 0xFD};
                pRXCharacteristic->writeValue(CIV_Set_Mode_Filter, sizeof(CIV_Set_Mode_Filter));
                delay(10);

                // Send the CIV command to turn PTT ON (will tur OFF when button is released)
                uint8_t CIV_PTT_ON[] = {0xFE, 0xFE, radio_address, 0xE0, 0x1C, 0x00, 0x01, 0xFD};
                pRXCharacteristic->writeValue(CIV_PTT_ON, sizeof(CIV_PTT_ON));
                delay(10);
                unsigned long startTime = millis();
                while (millis() - startTime < PTTdurationForSWRmeterInMs)
                {
                    // Send the CIV command to get SWR
                    uint8_t CIV_Get_SWR[] = {0xFE, 0xFE, radio_address, 0xE0, 0x15, 0x12, 0xFD};
                    pRXCharacteristic->writeValue(CIV_Get_SWR, sizeof(CIV_Get_SWR));
                    delay(10); // Allow some time for the response

                    // swr is calculated in notifyCallback()
                    if (drawLinearSWRmeterInsteadOfLogarithmic)
                    {
                        SWR_drawLinearSWRbar(swr);
                    }
                    else
                    {
                        SWR_drawLogarithmicSWRbar(swr);
                    }

                    delay(25); // Control the update rate
                }
            }
        }
        if (weAreInTheMainPage == false)
        {
            checkKeypadPage();
        }
        if (freqIsOutOfRange == true)
        {
            allStepperInfotoBeRedrawn = true;
            displayOutofRangeMessage(180, 280);
        }
    }
}
// Function to draw the progress bar
void drawProgressBar(int x, int y, int givenWidth, int height, int duration)
{
    // Clear the area for the progress bar
    tft.fillRect(0, y, 480, 320 - y, TFT_BLACK);

    int gap = 2;       // Fixed width of the gap between rectangles
    int rectWidth = 4; // Fixed width of each rectangle

    int totalBarWidth = rectWidth + gap;            // Width occupied by one rectangle and one gap
    int numRectangles = givenWidth / totalBarWidth; // Total number of rectangles that fit

    int lastRectangleRightEdge = x + (numRectangles * totalBarWidth - gap);
    int frameWidth = lastRectangleRightEdge - x;

    // Draw the frame for the progress bar
    tft.drawRect(x, y, frameWidth, height, TFT_WHITE); // Draw the white frame

    // Helper function to interpolate color
    auto interpolateColor = [](int progress) -> uint16_t
    {
        int red, green, blue = 0; // Initialize color components

        if (progress <= 80)
        {
            // Red to Yellow transition (0% to 80%)
            red = 255;                     // Full red
            green = (progress * 255) / 80; // Gradually increase green to 255 at 80%
        }
        else
        {
            // Yellow to Green transition (80% to 100%)
            red = 255 - ((progress - 80) * 255) / 20; // Gradually decrease red to 0 at 100%
            green = 255;                              // Full green
        }

        return tft.color565(red, green, blue); // Return the RGB color in 565 format
    };

    unsigned long startTime = millis();
    unsigned long currentTime = startTime;

    // Draw each rectangle with interpolated color
    for (int i = 0; i < numRectangles; i++)
    {
        // Calculate the progress percentage
        int progress = (i * 100) / numRectangles;

        // Get the interpolated color for this progress
        uint16_t color = interpolateColor(progress);

        // Draw the rectangle with gradient color, offset by 1 pixel to avoid overlapping the left frame
        tft.fillRect(x + i * totalBarWidth + 1, y + 1, rectWidth - 1, height - 2, color);

        // Wait for the segment's time duration
        currentTime = millis();
        while (currentTime - startTime < duration / numRectangles)
        {
            currentTime = millis();
        }
        startTime = currentTime;
    }

    // Clear the area for the progress bar
    tft.fillRect(0, y, 480, 320 - y, TFT_BLACK);
    allStepperInfotoBeRedrawn = true;
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

void updateWiFiWidget(int x, int y, int radius, float sizePercentage, int rssi)
{
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
    tft.setTextFont(1);                     // Set small font size
    tft.setTextSize(2);                     // Set small font size
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background to clear previous text
    tft.setCursor(x - 23, y + 13);          // Position the text below the Wi-Fi icon

    tft.printf("%d", rssi);        // Print the RSSI value in dBm
    tft.setCursor(x - 20, y + 33); // Position the text below the Wi-Fi icon
    tft.printf("dBm");             // Print the RSSI value in dBm
}

// Function to display stepper information
void displayStepperInfo(int x, int y, int currentPos, int stepsToGo)
{
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
        tft.drawString("Current Pos: ", x, y);
        tft.drawString("Steps to Go: ", x, y + 30); // Adjust y position for spacing
        labelsDrawn = true;                         // Set the flag so labels are not drawn again
    }

    // Display formatted values
    String formattedPos = formatCurrentPos(currentPos); // Proper format for Current Pos
    tft.drawString(formattedPos, x + 175, y);           // Adjust x position for alignment

    String formattedSteps = formatStepsWithSign(stepsToGo); // Format with sign for Steps to Go
    tft.drawString(formattedSteps, x + 175, y + 30);        // Adjust x position for alignment
}

// ##############################################################################################################
// BUTTONS On MAIN PAGE
void buttonSETonMainPage_pressAction(void)
{
    if (buttonSETonMainPage.justPressed())
    {
        Serial.println("Set button just pressed");
        weAreInTheMainPage = false;
        tft.fillScreen(TFT_BLACK); // Set background to black

        // Draw keypad background
        tft.fillRect(0, 0, DISP_X + DISP_W + DISP_X, 320, TFT_DARKGREY);

        // Draw number display area and frame
        tft.fillRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_BLACK);

        tft.drawRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_WHITE);

        // Draw keypad
        drawKeypad();

        // Display the resonance frequency text
        displayInfoTextOnRightPane();

        // Draw the right pane button (e.g., "Save")
        tft.setFreeFont(buttonFont); // Numbers use buttonFont
        exitButton.initButton(&tft, 385, 280, 120, 50, TFT_WHITE, TFT_RED, TFT_WHITE, "Exit", 1);
        exitButton.drawButton();
        // buttonSETonMainPage.drawSmoothButton(true);
    }
}

void buttonSETonMainPage_releaseAction(void)
{
    // nothing to be done here
}

void buttonPTTonMainPage_pressAction(void)
{
    if (buttonPTTonMainPage.justPressed())
    {
        Serial.println("PTT button just pressed");
        // Draw the button as pressed (white background with black text)
        tft.setFreeFont(&FreeMonoBold12pt7b); // Set the FreeMonoBold12pt7b font from TFT_eSPI
        buttonPTTonMainPage.drawSmoothButton(true, 3, TFT_WHITE, "SWR");
        drawBlackRectangleToErasePortionOfScreen(0, 220, 480, 320); // to erase the bottom of the screen
        PTTisPressed = true;
    }
}

void buttonPTTonMainPage_releaseAction(void)
{
    if (buttonPTTonMainPage.justReleased())
    {
        Serial.println("PTT button just released");
        Serial.println("Setting MODE and FIL to initial values");

        // Send the CIV command to turn PTT OFF
        uint8_t CIV_PTT_OFF[] = {0xFE, 0xFE, radio_address, 0xE0, 0x1C, 0x00, 0x00, 0xFD};
        pRXCharacteristic->writeValue(CIV_PTT_OFF, sizeof(CIV_PTT_OFF));
        delay(100);

        // Send the CIV command to restore original Power Setting
        uint8_t highByte = (initialRFPower >> 8) & 0xFF; // Extract the high byte
        uint8_t lowByte = initialRFPower & 0xFF;         // Extract the low byte

        // Construct the CI-V command to set RF power back to initialRFPower
        uint8_t CIV_Set_RF_Power[] = {0xFE, 0xFE, radio_address, 0xE0, 0x14, 0x0A, highByte, lowByte, 0xFD};

        // Send the command
        pRXCharacteristic->writeValue(CIV_Set_RF_Power, sizeof(CIV_Set_RF_Power));
        delay(10); // Allow some time for the command to be sent

        // To set the MODE and FILTER back to currentMode and currentFilter
        uint8_t CIV_Set_Current_Mode_Filter[] = {0xFE, 0xFE, radio_address, 0xE0, 0x01, currentMode, currentFilter, 0xFD};
        pRXCharacteristic->writeValue(CIV_Set_Current_Mode_Filter, sizeof(CIV_Set_Current_Mode_Filter));
        delay(100);

        tft.setFreeFont(&FreeMonoBold12pt7b); // Set the FreeMonoBold12pt7b font from TFT_eSPI for the button o display correctly
        // Redraw the button in the unpressed state (black background with white text)
        buttonPTTonMainPage.drawSmoothButton(false, 2, TFT_BLACK, "SWR");
        allStepperInfotoBeRedrawn = true;
        drawBlackRectangleToErasePortionOfScreen(0, 220, 480, 320); // to erase SWR BAR
        displayStepperInfo(150, 250, currentStepperPosition, deltaSteps);
        allStepperInfotoBeRedrawn = false;
        // redraw the buttons
        buttonWSPRonMainPage.drawSmoothButton(false, 2, TFT_BLACK); // redraws the button
        buttonSETonMainPage.drawSmoothButton(false, 2, TFT_BLACK);  // redraws the button
        PTTisPressed = false;
        SWR_first_draw = true; // Track if this is the first call of SWR_drawLogarithmicSWRbar
    }
}

void buttonFT8onMainPage_pressAction(void)
{

    unsigned long freqToSet; // The frequency to be set based on conditions

    // Check if the button is already pressed to avoid unnecessary redraws
    if (!isFT8ButtonPressed)
    {
        Serial.println("FT8 button just pressed");

        // Check if CurrentVFOFrequency is within 40m band
        if (CurrentVFOFrequency >= LOWER_40M && CurrentVFOFrequency <= UPPER_40M)
        {
            freqToSet = FT8_40M; // Set frequency to 7074 kHz (FT8 frequency for 40m band)
        }
        // Check if CurrentVFOFrequency is within 20m band
        else if (CurrentVFOFrequency >= LOWER_20M && CurrentVFOFrequency <= UPPER_20M)
        {
            freqToSet = FT8_20M; // Set frequency to 14074 kHz (FT8 frequency for 20m band)
        }

        // Print the selected frequency to the serial monitor for debugging
        Serial.print("New freqToSet value: ");
        Serial.println(formatFrequencyForConsoleOutput(freqToSet));
        set_IC705_frequency(freqToSet);
        tft.setFreeFont(&FreeMonoBold12pt7b);                            // Set font
        buttonFT8onMainPage.drawSmoothButton(true, 2, TFT_WHITE, "FT8"); // Change button color when pressed
        isFT8ButtonPressed = true;                                       // Mark button as pressed
    }
}

void buttonFT8onMainPage_releaseAction(void)
{
    // Only revert the button if it was actually pressed, to avoid unnecessary redraws
    if (isFT8ButtonPressed)
    {
        Serial.println("FT8 button just released");
        tft.setFreeFont(&FreeMonoBold12pt7b);                             // Set font
        buttonFT8onMainPage.drawSmoothButton(false, 2, TFT_BLACK, "FT8"); // Revert button color to original state
        isFT8ButtonPressed = false;                                       // Mark button as released
    }
}

void buttonWSPRonMainPage_pressAction(void)
{
    unsigned long freqToSet; // The frequency to be set based on conditions

    // Check if the button is already pressed to avoid unnecessary redraws
    if (!isWSPRButtonPressed)
    {
        Serial.println("WSPR button just pressed");

        // Check if CurrentVFOFrequency is within 40m band
        if (CurrentVFOFrequency >= LOWER_40M && CurrentVFOFrequency <= UPPER_40M)
        {
            freqToSet = WSPR_40M; // Set frequency to WSPR for 40m band
        }
        // Check if CurrentVFOFrequency is within 20m band
        else if (CurrentVFOFrequency >= LOWER_20M && CurrentVFOFrequency <= UPPER_20M)
        {
            freqToSet = WSPR_20M; // Set frequency to WSPR for 20m band
        }

        // Print the selected WSPR frequency to the serial monitor for debugging
        Serial.print("New freqToSet value (WSPR): ");
        Serial.println(freqToSet);

        set_IC705_frequency(freqToSet);

        tft.setFreeFont(&FreeMonoBold12pt7b);                              // Set font
        buttonWSPRonMainPage.drawSmoothButton(true, 2, TFT_WHITE, "WSPR"); // Change button color when pressed
        isWSPRButtonPressed = true;                                        // Mark button as pressed
    }
}

void buttonWSPRonMainPage_releaseAction(void)
{
    // Only revert the button if it was actually pressed, to avoid unnecessary redraws
    if (isWSPRButtonPressed)
    {
        Serial.println("WSPR button just released");
        tft.setFreeFont(&FreeMonoBold12pt7b);                               // Set font
        buttonWSPRonMainPage.drawSmoothButton(false, 2, TFT_BLACK, "WSPR"); // Revert button color to original state
        isWSPRButtonPressed = false;                                        // Mark button as released
    }
}

void initButtonsOnMainPage()
{
    tft.setFreeFont(&FreeMonoBold12pt7b); // Set the FreeMonoBold12pt7b font from TFT_eSPI
    int x = 12;
    int y = 120;

    buttonPTTonMainPage.initButtonUL(x, y, 100, 40, TFT_WHITE, TFT_BLACK, TFT_WHITE, "SWR", 1);
    buttonPTTonMainPage.setPressAction(buttonPTTonMainPage_pressAction);
    buttonPTTonMainPage.setReleaseAction(buttonPTTonMainPage_releaseAction);
    buttonPTTonMainPage.drawSmoothButton(false, 3, TFT_BLACK); // 2 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing

    y = y + 52;
    buttonFT8onMainPage.initButtonUL(x, y, 100, 40, TFT_WHITE, TFT_BLACK, TFT_WHITE, "FT8", 1);
    buttonFT8onMainPage.setPressAction(buttonFT8onMainPage_pressAction);
    buttonFT8onMainPage.setReleaseAction(buttonFT8onMainPage_releaseAction);
    buttonFT8onMainPage.drawSmoothButton(false, 2, TFT_BLACK); // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing
    y = y + 52;
    buttonWSPRonMainPage.initButtonUL(x, y, 100, 40, TFT_WHITE, TFT_BLACK, TFT_WHITE, "WSPR", 1);
    buttonWSPRonMainPage.setPressAction(buttonWSPRonMainPage_pressAction);
    buttonWSPRonMainPage.setReleaseAction(buttonWSPRonMainPage_releaseAction);
    buttonWSPRonMainPage.drawSmoothButton(false, 2, TFT_BLACK); // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing
    y = y + 52;
    buttonSETonMainPage.initButtonUL(x, y, 100, 40, TFT_WHITE, TFT_BLACK, TFT_WHITE, "Set", 1);
    buttonSETonMainPage.setPressAction(buttonSETonMainPage_pressAction);
    buttonSETonMainPage.setReleaseAction(buttonSETonMainPage_releaseAction);
    buttonSETonMainPage.drawSmoothButton(false, 2, TFT_BLACK); // 3 is outline width, TFT_BLACK is the surrounding background colour for anti-aliasing
}

void calibrateTouchDisplay()
{
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // Check if the file system exists
    Serial.println("Initializing file system...");
    if (!LittleFS.begin())
    {
        Serial.println("File system not found. Formatting...");
        LittleFS.format();
        LittleFS.begin(); // Initialize the file system
    }

    // Check if the calibration file exists and if its size is correct
    if (LittleFS.exists(CALIBRATION_FILE))
    {
        Serial.println("Calibration file found. Checking validity...");
        if (REPEAT_CAL)
        {
            // If we want to re-calibrate, delete the existing calibration file
            Serial.println("Re-calibration requested. Deleting existing calibration file...");
            LittleFS.remove(CALIBRATION_FILE);
        }
        else
        {
            fs::File f = LittleFS.open(CALIBRATION_FILE, "r"); // Open the calibration file for reading
            if (f)
            {
                Serial.println("Reading calibration data...");
                // Attempt to read 14 bytes of calibration data
                if (f.readBytes((char *)calData, 14) == 14)
                {
                    calDataOK = 1; // Calibration data is valid
                    Serial.println("Calibration data read successfully.");
                }
                else
                {
                    Serial.println("Failed to read complete calibration data.");
                }
                f.close(); // Close the file
            }
            else
            {
                Serial.println("Failed to open calibration file for reading.");
            }
        }
    }

    if (calDataOK && !REPEAT_CAL)
    {
        // If calibration data is valid, set it to the touch screen
        Serial.println("Setting touch calibration data...");
        tft.setTouch(calData);
    }
    else
    {
        // If data is not valid, proceed to recalibrate
        Serial.println("Calibration data invalid or re-calibration requested. Starting calibration...");
        tft.fillScreen(TFT_BLACK); // Clear the screen

        // Set text font and size
        tft.setTextFont(4);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        // Display calibration instructions
        tft.setCursor(80, 50);
        tft.println("HB9IIU MLA CONTROLLER");

        tft.setCursor(120, 100);
        tft.println("Screen Calibration");

        tft.setCursor(75, 150);
        tft.println("Touch corners as indicated");

        tft.setCursor(35, 200);
        tft.println("This procedure is required only once");

        // Perform the touch calibration
        tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 20);
        Serial.println("Calibration process completed.");

        tft.fillScreen(TFT_BLACK); // Clear the screen

        // Indicate calibration completion
        tft.setCursor(120, 100);
        tft.println("Calibration complete!");
        Serial.println("Calibration complete! Storing data...");
        delay(2000);               // Pause to let the user see the message
        tft.fillScreen(TFT_BLACK); // Clear the screen again

        // Store the calibration data
        fs::File f = LittleFS.open(CALIBRATION_FILE, "w"); // Open the file for writing
        if (f)
        {
            f.write((const unsigned char *)calData, 14); // Write the calibration data
            Serial.println("Calibration data saved successfully.");
            f.close(); // Close the file
        }
        else
        {
            Serial.println("Failed to open calibration file for writing.");
        }
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

void establish_WIFI_connection_with_Slave()
{
    Serial.print("Establishing WiFi Connection with Slave...");
    printOnTFT("Connecting To Slave via WiFi...", TFT_WHITE, TFT_BLACK);

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
        int8_t rssi = WiFi.RSSI(); // Get WiFi RSSI (signal strength)
        String qualityDescription;
        uint16_t qualityColor;
        getLinkQuality(rssi, qualityDescription, qualityColor);
        Serial.println("\nSuccessful Wifi Connection to Slave");
        Serial.print("Signal Quality:");
        Serial.println(qualityDescription.c_str());
        String message = "Link Quality: " + qualityDescription;
        printOnTFT(message.c_str(), qualityColor, TFT_BLACK);
        // Initialize HTTPClient
        String url = String("http://") + slaveIP.toString() + "/command"; // Ensure slaveIP is defined
        http.begin(url);                                                  // Ensure http is defined and initialized
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
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

// Function to get link quality description and corresponding color
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

bool connectToServer(BLEAddress pAddress)
{
    Serial.print("Establishing a connection to device address: ");
    Serial.println(pAddress.toString().c_str());

    // Convert both to Arduino Strings
    String fullMessage = "Connecting to: " + String(pAddress.toString().c_str());

    // Pass the concatenated string to printOnTFT
    printOnTFT(fullMessage.c_str(), TFT_WHITE, TFT_BLACK);

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
    if (length == 8 &&
        pData[0] == 0xFE && pData[1] == 0xFE && // Start of the packet
        pData[2] == 0xE0 &&                     // Source address
        pData[3] == radio_address &&            // Verify radio address
        pData[4] == 0x1C &&                     // PTT command identifier
        pData[5] == 0x00)                       // Sub-command for PTT status
    {

        /*
        unsigned long currentPacketTime = millis(); // Get the current time in milliseconds

        // If this is not the first packet, calculate the interval
        if (packetCount > 0)
        {
            unsigned long interval = currentPacketTime - lastPacketTime; // Time difference since last packet
            intervalSum += interval;                                     // Add to the running sum of intervals
        }

        lastPacketTime = currentPacketTime; // Update the last packet time
        packetCount++;                      // Increment the packet count

        // Calculate the average interval (in milliseconds)
        float averageInterval = (packetCount > 1) ? (float)intervalSum / (packetCount - 1) : 0;

        // Optional: Print the average interval
        // Serial.print("Average Interval: ");
        // Serial.print(averageInterval);
        // Serial.println(" ms");
*/
        bool PTTStatus = (pData[6] == 0x01);  // PTT ON if 0x01, OFF if 0x00
        unsigned long currentTime = millis(); // Current time in ms

        checkPTT(PTTStatus);
    }

    // Check for selected band’s RF power response
    if (length == 9 &&
        pData[0] == 0xFE && pData[1] == 0xFE && // Start of the packet
        pData[2] == 0xE0 &&                     // Source address (IC-705)
        pData[3] == radio_address &&            // Verify radio address (e.g., 0xA4)
        pData[4] == 0x14 &&                     // Command for various settings
        pData[5] == 0x0A)                       // Sub-command for RF power
    {
        // Combine the high byte (pData[6]) and low byte (pData[7]) into a 16-bit value
        initialRFPower = (pData[6] << 8) | pData[7]; //
        Serial.println("");

        Serial.println("------------R------------");

        Serial.print(pData[6], HEX);
        Serial.print("  ");
        Serial.println(pData[7], HEX);
        Serial.println("-------------------------");

        // Calculate percentage based on the observed maximum range (597 in decimal)
        float RF_Power_Percentage = (initialRFPower / 597.0) * 100;
        Serial.print("RF Power Level: ");
        Serial.print(RF_Power_Percentage);
        Serial.println("%");

        /*
        Serial.print(pData[6], HEX);
        Serial.print("  ");
        Serial.println(pData[7], HEX); ////xxxx

        */
    }

    // Check for mode and filter status packet
    if (length == 8 &&
        pData[0] == 0xFE && pData[1] == 0xFE && // Start of the packet
        pData[2] == 0xE0 &&                     // Source address is always 0xE0 in this case (IC-705)
        pData[3] == radio_address &&            // Verify radio address (e.g., 0xA4)
        pData[4] == 0x04)                       // Command identifier for Mode/Filter
    {
        // Define the mode and filter mappings
        const char *modes[] = {"LSB", "USB", "AM", "CW", "RTTY", "FM", "WFM", "CW-R", "RTTY-R", "", "", "", "", "", "", "", "", "", "", "DV"};
        const char *filters[] = {"FIL1", "FIL2", "FIL3"};

        // Mode is likely in pData[5]
        currentMode = pData[5]; // Assign mode value to global variable
        // Serial.print("Mode: ");
        if (currentMode < sizeof(modes) / sizeof(modes[0]))
        {
            Serial.println(modes[currentMode]); // Map the mode to text
        }
        else
        {
            Serial.println("Unknown Mode");
        }

        // Filter status is likely in pData[6]
        currentFilter = pData[6]; // Assign filter value to global variable
        // Serial.print("Filter: ");

        if (currentFilter > 0 && (currentFilter - 1) < sizeof(filters) / sizeof(filters[0]))
        {
            Serial.println(filters[currentFilter - 1]); // Map the filter to text
        }
        else
        {
            Serial.println("Unknown Filter");
        }
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
        uint16_t swrByteValue = (swr_high << 8) | swr_low;

        // Calculate the actual SWR using the formula
        swr = 0.00696 * swrByteValue + 1.026;

        // Optional: Print the results for debugging
        /*
        Serial.print("SWR Byte Value: ");
        Serial.println(swrByteValue);
        Serial.print("Calculated SWR: ");
        Serial.println(swr);
*/
    }

    // Call the function to decode the frequency
    decodeFrequency(pData, length);
}

// Function to detect double PTT
void TuneOnDoublePTTclick()
{
    // Serial.println("Double Click Detected");
    // Serial.print("Steps to go:");
    // Serial.println(deltaSteps);

    if (deltaSteps == 0)

    {
        Serial.println("No Need To Tune; already on the right position");
        return;
    }
    // Check if the frequency is outside the defined ranges
    if (!((CurrentVFOFrequency >= LOWER_40M && CurrentVFOFrequency <= UPPER_40M) ||
          (CurrentVFOFrequency >= LOWER_20M && CurrentVFOFrequency <= UPPER_20M)))
    {
        Serial.println("Frequency is outside the defined ranges.");
        return; // Return if outside both ranges
    }
    drawBlackRectangleToErasePortionOfScreen(0, 220, 480, 320); // erase the bottom of the screen
    setNewPositionForCurrentVFOfrequency(CurrentVFOFrequency);
    GetTunedStatusFromSlave(); // just to get the new stepper position
    // cheating a bit because GetTunedStatusFromSlave will return a slightly different value of couple of Hz
    displayRESONANCEfrequency(CurrentVFOFrequency, 160, 130, ResonanceFrequDisplayColor);
    deltaSteps = 0;
    displayStepperInfo(150, 250, currentStepperPosition, deltaSteps);
    buttonFT8onMainPage.drawSmoothButton(false, 2, TFT_BLACK); // 2 is outline width
    buttonWSPRonMainPage.drawSmoothButton(false, 2, TFT_BLACK);
}
void decodeFrequency(uint8_t *pData, size_t length)
{
    // Check if the packet structure is valid for a frequency packet
    if (length != 11 ||              // Ensure valid length for frequency packet
        pData[0] != 0xFE ||          // Start of the packet (0xFE)
        pData[1] != 0xFE ||          // Second start marker (0xFE)
        pData[2] != 0xE0 ||          // Source address (0xE0 for IC-705)
        pData[3] != radio_address || // Verify radio address (e.g., 0xA4)
        pData[4] != 0x03 ||          // Command identifier for frequency
        pData[10] != 0xFD)           // End marker (0xFD)
    {
        return; // If any condition is not met, return (invalid packet)
    }

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

char *formatFrequencyForConsoleOutput(uint64_t vfo)
{

    static char vfo_str[20] = {""};
    uint32_t MHz = (vfo / 1000000 % 1000000);
    uint16_t Hz = (vfo % 1000);
    uint16_t KHz = ((vfo % 1000000) - Hz) / 1000;
    sprintf(vfo_str, "%lu.%03u.%03u", MHz, KHz, Hz);
    return vfo_str;
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

void getStepperPositionForCurrentVFOfrequency(uint32_t currentVFOfrequency)
{

    static uint64_t prev_freq = 0;

    if (currentVFOfrequency != prev_freq)

    {
        // force_getStepperPositionForCurrentVFOfrequency=false;

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
                    if ((currentVFOfrequency >= LOWER_40M && currentVFOfrequency <= UPPER_40M))
                    {
                        Serial.println("Frequency is within the 40m range.");

                        freqIsOutOfRange = false;
                        if (backInRange == false)
                        {
                            drawBlackRectangleToErasePortionOfScreen(150, 250, 480, 320);
                            allStepperInfotoBeRedrawn = true;

                            backInRange = true;
                        }
                        displayStepperInfo(150, 250, currentStepperPosition, deltaSteps);
                    }
                    else if ((currentVFOfrequency >= LOWER_20M && currentVFOfrequency <= UPPER_20M))
                    {
                        Serial.println("Frequency is within the 20m range.");
                        freqIsOutOfRange = false;

                        if (backInRange == false)
                        {

                            drawBlackRectangleToErasePortionOfScreen(150, 250, 480, 320);

                            backInRange = true;
                            allStepperInfotoBeRedrawn = true;
                        }

                        displayStepperInfo(150, 250, currentStepperPosition, deltaSteps);
                    }
                    else
                    {
                        Serial.println("Frequency is outside the defined ranges.");
                        freqIsOutOfRange = true;
                        backInRange = false;
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
                success = true; // Mark as successful
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
    drawProgressBar(20, 250, 435, 40, estimated_movement_duration_in_microseconds);

    GetTunedStatusFromSlave();
}

// ###########################################################################################################

void drawKeypad()
{

    tft.setFreeFont(buttonFont); // CLS and Send use LABEL1_FONT

    // Draw the keys in the desired layout
    for (uint8_t row = 0; row < 4; row++)
    {

        for (uint8_t col = 0; col < 3; col++)
        {
            uint8_t b = col + row * 3; // Calculate the key index based on row and column

            key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                              KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                              KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                              keyLabel[b], 1);
            key[b].drawButton();
        }
    }
}

void displayInfoTextOnRightPane()
{
    tft.setTextFont(2);                     // Set the smaller font
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text on black background
    tft.setTextDatum(TL_DATUM);             // Top-left datum (origin)
    // First part of the text
    const char *part1 = "Setting the current resonance frequency is only necessary if you have made modifications to the drive unit after the characterization process.";
    // Second part of the text (starting with "If needed...")
    const char *part2 = "If needed, connect your VNA to the antenna and enter the recorded resonance frequency here.";
    // Call the word wrap function for the first part of the text
    int newY = drawTextWithWordWrap(part1, 295, 30, 470, 20); // Start at x=280, y=10
    // Call the word wrap function for the second part of the text, using the returned y position
    drawTextWithWordWrap(part2, 295, newY, 470, 20); // Start below the first part
}

int drawTextWithWordWrap(const char *text, int x, int y, int maxX, int lineHeight)
{
    String currentLine = ""; // To store the current line of text
    String word = "";        // To store individual words
    int currentX = x;        // Start at the given x coordinate
    int currentY = y;        // Start at the given y coordinate

    while (*text != 0)
    {
        char c = *text++;

        // Check for space or end of word
        if (c == ' ' || c == 0)
        {
            // Get width of current line with the new word
            int wordWidth = tft.textWidth(currentLine + word);

            // Check if the word fits in the current line
            if (currentX + wordWidth > maxX)
            {
                // If the word doesn't fit, print the current line and move to the next line
                tft.drawString(currentLine, currentX, currentY);
                currentY += lineHeight;
                currentLine = word + " "; // Start the new line with the current word
            }
            else
            {
                // If the word fits, add it to the current line
                currentLine += word + " ";
            }

            // Clear the word buffer for the next word
            word = "";
        }
        else
        {
            // Add character to the current word
            word += c;
        }
    }

    // **New: Handle the last word** if there's any remaining text in the word buffer
    if (word.length() > 0)
    {
        int wordWidth = tft.textWidth(currentLine + word);
        if (currentX + wordWidth > maxX)
        {
            // If the word doesn't fit, print the current line and move to the next line
            tft.drawString(currentLine, currentX, currentY);
            currentY += lineHeight;
            currentLine = word; // Start the new line with the current word
        }
        else
        {
            currentLine += word; // Add the last word to the current line
        }
    }
    // Print the last line if it exists
    if (currentLine.length() > 0)
    {
        tft.drawString(currentLine, currentX, currentY);
        currentY += lineHeight; // Move to the next line after the last line
    }

    return currentY; // Return the y position after the text is drawn (for spacing)
}

// Function to right-align the number within 8 characters
void formatRightAlignedNumber(char *buffer, char *numberBuffer, int maxLength)
{
    int numLength = strlen(numberBuffer);     // Get the length of the current number
    int spacesNeeded = maxLength - numLength; // Calculate the number of spaces to pad

    // Fill the buffer with spaces
    memset(buffer, '-', spacesNeeded);

    // Copy the number to the end of the buffer (after the spaces)
    strcpy(buffer + spacesNeeded, numberBuffer);
}

// Function to convert the numberBuffer to an integer
unsigned long convertBufferToNumber(char *buffer)
{
    return strtoul(buffer, NULL, 10); // Convert the string to an unsigned long
}

void checkKeypadPage()
{

    uint16_t t_x = 0, t_y = 0; // To store the touch coordinates

    // Pressed will be set true if there is a valid touch on the screen
    bool pressed = tft.getTouch(&t_x, &t_y);

    // Check if the exitButton is pressed
    if (pressed && exitButton.contains(t_x, t_y))
    {
        Serial.println("Exit Button on keypad page Pressed");
        displayRebootingMessageScreen();
    }

    // Check if any key coordinate boxes contain the touch coordinates
    for (uint8_t b = 0; b < 12; b++)
    {
        if (pressed && key[b].contains(t_x, t_y))
        {
            key[b].press(true); // Tell the button it is pressed
        }
        else
        {
            key[b].press(false); // Tell the button it is NOT pressed
        }
    }

    // Check if any key has changed state
    for (uint8_t b = 0; b < 12; b++)
    {

        tft.setFreeFont(buttonFont);

        if (key[b].justReleased())
            key[b].drawButton(); // Draw normal

        if (key[b].justPressed())
        {
            key[b].drawButton(true); // Draw invert

            // If a numberpad button, append the relevant # to the numberBuffer
            if (b < 10)
            { // Numeric buttons
                if (numberIndex < NUM_LEN)
                {
                    numberBuffer[numberIndex] = keyLabel[b][0];
                    numberIndex++;
                    numberBuffer[numberIndex] = 0; // Zero terminate
                }
            }

            // CLS button pressed (10)
            if (b == 10)
            {                                  // CLS
                numberIndex = 0;               // Reset index to 0
                numberBuffer[numberIndex] = 0; // Place null in buffer
                // Clear the number display field
                tft.fillRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_BLACK);
                tft.drawRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_WHITE);
            }

            // Set button pressed (11)
            if (b == 11)
            { // Set
                // Convert the buffer to a number
                uint32_t number = convertBufferToNumber(numberBuffer);
                // Validate the number
                if (!((number >= LOWER_40M && number <= UPPER_40M) ||
                      (number >= LOWER_20M && number <= UPPER_20M)))
                {
                    // If invalid, display "ERROR"
                    tft.fillRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_BLACK); // Clear the display
                    tft.drawRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_WHITE); // Draw the frame

                    tft.setTextDatum(TL_DATUM); // Use top left corner as text coord datum
                    tft.setTextFont(4);

                    for (int i = 0; i < 4; i++)
                    {
                        tft.setTextColor(TFT_RED); // Set color to red for "ERROR"

                        // Display "ERROR"
                        tft.drawString("ERROR", DISP_X + 85, DISP_Y + 10);
                        tft.setTextColor(TFT_WHITE); // Set color to red for "ERROR"

                        // Display the message
                        tft.drawString("Check your frequency", DISP_X + 10, DISP_Y + 35);

                        numberIndex = 0;               // Reset index to 0
                        numberBuffer[numberIndex] = 0; // Place null in buffer

                        // Wait for 300 milliseconds)
                        delay(300);
                        // Clear the number display field
                        tft.fillRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_BLACK);
                        // Draw the round rectangle outline
                        tft.drawRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_WHITE);
                        // Wait again for 300 milliseconds)
                        delay(300);
                    }
                }
                else
                {
                    // If valid, proceed with further handling (currently just print the number to Serial)
                    Serial.print("Valid number: ");
                    Serial.println(number);
                    setNewStepperPosToSlavePreferencesForGivenFrequency(number);
                    // TODO -> Error checking
                    // Clear the number display field
                    tft.fillRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_BLACK);

                    // Draw the round rectangle outline
                    tft.drawRoundRect(DISP_X, DISP_Y, DISP_W, DISP_H, 10, TFT_WHITE);

                    tft.setTextColor(TFT_GREEN); // Set color to red for "ERROR"
                    tft.setTextFont(4);

                    // Display "SUCCESS"
                    tft.drawString("SUCCESS", DISP_X + 75, DISP_Y + 10);
                    tft.setTextColor(TFT_WHITE); // Set color to red for "ERROR"

                    // Display the message
                    tft.drawString("Slave Updated", DISP_X + 50, DISP_Y + 35);
                    delay(2000);
                    displayRebootingMessageScreen();
                }
            }

            // Displaying the right-aligned number
            char displayBuffer[NUM_LEN + 1]; // Buffer to hold the right-aligned number

            // Format the number to be right-aligned with leading "-"
            formatRightAlignedNumber(displayBuffer, numberBuffer, NUM_LEN);
            // Update the number display field
            tft.setTextDatum(TL_DATUM); // Use top left corner as text coord datum
            tft.setTextFont(7);         // Set font7 (LOADED_FONT7)

            // Loop through the displayBuffer and print each character
            int xPos = DISP_X + 8; // Start position for the first character

            // Clear the number display field
            tft.fillRoundRect(DISP_X + 2, DISP_Y + 2, DISP_W - 4, DISP_H - 4, 10, TFT_BLACK);

            for (int i = 0; i < NUM_LEN; i++)
            {
                Serial.print(displayBuffer[i]);
                Serial.print(" ");
                if (displayBuffer[i] == '-')
                {
                    tft.setTextColor(TFT_BLACK); // Set color to black for placeholder
                }
                else
                {
                    tft.setTextColor(TFT_GREEN); // Set color to green for digits
                }
                // Print each character one by one
                tft.drawChar(displayBuffer[i], xPos, DISP_Y + 12);

                // Move the position for the next character
                xPos += tft.textWidth("0"); // Advance position by the width of a digit
            }
            Serial.println();
            delay(5); // UI debouncing
        }
    }
}

void displayWelcomeScreen(int duration, const char *version, const char *date)
{

    // Center coordinates
    int centerX = tft.width() / 2;  // X coordinate of the center
    int centerY = tft.height() / 2; // Y coordinate of the center

    // Circle parameters
    const int INITIAL_RADIUS = 10;
    const int GAP = 25;
    const int MAX_RADIUS = centerX; // Max radius based on screen width
    const int CIRCLE_THICKNESS = 5; // Thickness of the circle

    // Text Y positions
    const int TEXT_Y_WELCOME = 80;
    const int TEXT_Y_CONTROLLER = 140;
    const int TEXT_Y_VERSION = 200;

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
    tft.setFreeFont(&FreeMonoBold18pt7b);
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

    // Set font for the version number and date
    tft.setFreeFont(&FreeMonoBold12pt7b);
    int xVersion = (tft.width() - tft.textWidth(versionText)) / 2;
    int xDate = (tft.width() - tft.textWidth(versionDate)) / 2;

    // Draw version and date text
    tft.setCursor(xVersion, TEXT_Y_VERSION);
    tft.print(versionText);
    tft.setCursor(xDate, TEXT_Y_VERSION + 50);
    tft.print(versionDate);

    // Initialize random seed
    randomSeed(analogRead(0)); // Use an analog pin to seed random for better randomness

    // Display concentric circles with animation
    for (int j = 0; j < 3; j++)
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
            // Redraw the splash screen text
            tft.setFreeFont(&FreeMonoBold18pt7b);
            tft.setCursor(xWelcome, TEXT_Y_WELCOME);
            tft.print("Welcome To");
            tft.setCursor(xController, TEXT_Y_CONTROLLER);
            tft.print("HB9IIU MLA CONTROLLER");
            tft.setFreeFont(&FreeMonoBold12pt7b);
            tft.setCursor(xVersion, TEXT_Y_VERSION);
            tft.print(versionText);
            tft.setCursor(xDate, TEXT_Y_VERSION + 50);
            tft.print(versionDate);
        }

        // Erase circles by drawing them in the erase color
        for (int i = 0; i < numCircles; i++)
        {
            Serial.println("Circle");
            delay(1000);
            for (int thickness = 0; thickness < CIRCLE_THICKNESS; thickness++)
            {
                tft.drawCircle(centerX, centerY, diameters[i] + thickness, eraseColor);
            }
            // Redraw the splash screen text
            tft.setFreeFont(&FreeMonoBold18pt7b);
            tft.setCursor(xWelcome, TEXT_Y_WELCOME);
            tft.print("Welcome To");
            tft.setCursor(xController, TEXT_Y_CONTROLLER);
            tft.print("HB9IIU MLA CONTROLLER");
            tft.setFreeFont(&FreeMonoBold12pt7b);
            tft.setCursor(xVersion, TEXT_Y_VERSION);
            tft.print(versionText);
            tft.setCursor(xDate, TEXT_Y_VERSION + 50);
            tft.print(versionDate);
        }
    }
    // Redraw the splash screen text
    tft.setFreeFont(&FreeMonoBold18pt7b);
    tft.setCursor(xWelcome, TEXT_Y_WELCOME);
    tft.print("Welcome To");
    tft.setCursor(xController, TEXT_Y_CONTROLLER);
    tft.print("HB9IIU MLA CONTROLLER");
    tft.setFreeFont(&FreeMonoBold12pt7b);
    tft.setCursor(xVersion, TEXT_Y_VERSION);
    tft.print(versionText);
    tft.setCursor(xDate, TEXT_Y_VERSION + 50);
    tft.print(versionDate);

    // Delay before clearing the screen
    delay(1500);
    tft.fillScreen(TFT_BLACK); // Clear the screen
}

void displayWelcomeScreenSimple(int duration, const char *version, const char *date)
{

    // Text Y positions
    const int TEXT_Y_WELCOME = 80;
    const int TEXT_Y_CONTROLLER = 140;
    const int TEXT_Y_VERSION = 200;

    // Draw the splash screen text
    tft.setFreeFont(&FreeMonoBold18pt7b);
    tft.setTextColor(TFT_GOLD);
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

    // Set font for the version number and date
    tft.setFreeFont(&FreeMonoBold12pt7b);
    int xVersion = (tft.width() - tft.textWidth(versionText)) / 2;
    int xDate = (tft.width() - tft.textWidth(versionDate)) / 2;

    // Draw version and date text
    tft.setCursor(xVersion, TEXT_Y_VERSION);
    tft.print(versionText);
    tft.setCursor(xDate, TEXT_Y_VERSION + 50);
    tft.print(versionDate);

    // Delay before clearing the screen
    delay(1500);
    tft.fillScreen(TFT_BLACK); // Clear the screen
}

void setNewStepperPosToSlavePreferencesForGivenFrequency(uint32_t newVFOFrequency)
{
    const int maxRetries = 5;    // Maximum number of retries
    const int retryDelay = 1000; // Delay between retries in milliseconds
    int attempts = 0;
    bool success = false;

    while (attempts < maxRetries && !success)
    {
        attempts++;
        Serial.printf("\nAttempt %d: Sending command to Slave to store new reference point\n", attempts);
        String response = sendCommandToSlave("setTunedFrequToPreferenceOnSlave", String(newVFOFrequency));

        if (response.length() > 0)
        {
            // Parse the response to extract stepper position and vfoFrequency
            int separatorIndex = response.indexOf(',');
            if (separatorIndex != -1)
            {
                String stepperPositionStr = response.substring(0, separatorIndex);
                String frequencyStr = response.substring(separatorIndex + 1);
                currentStepperPosition = stepperPositionStr.toInt();
                theoreticalResonanceFrequency = frequencyStr.toInt();
                Serial.println("Response from slave): ");

                Serial.print("Current Stepper Position: ");
                Serial.println(formatStepperPosForConsoleOutput(currentStepperPosition));
                Serial.print("Resulting Lookup Resonance Frequency:  ");
                Serial.println(formatFrequencyForConsoleOutput(theoreticalResonanceFrequency));
                Serial.println("");
                success = true; // Mark as successful
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
        // displayErrorOnLCD("setTunedFrequToPreferenceOnSlave()");
    }
}

void displayRebootingMessageScreen()
{

    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(&FreeMonoBold18pt7b); // Set the custom font
    // Calculate the x coordinates to center the text
    int16_t x1 = (tft.width() - tft.textWidth("HB9IIU MLA CONTROLLER")) / 2;
    int16_t x2 = (tft.width() - tft.textWidth("Rebooting")) / 2;
    tft.setCursor(x1, 80);
    tft.print("HB9IIU MLA CONTROLLER");
    tft.setCursor(x2, 140);
    tft.print("Rebooting");
    esp_restart();
}

//-------- SWR ---------------------------------------------------------------------------------

// Helper function to map float values
float SWR_mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

// Function to draw tick marks and labels for SWR values (1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0)
void SWR_drawTicksAndLabels()
{
    // Tick values and corresponding positions on the bar
    float tick_values[] = {1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0};
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
        tft.drawString(String(tick_values[i], 1), tick_x - 10, SWR_BAR_Y + SWR_BAR_HEIGHT + SWR_TICK_HEIGHT + 5);
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

// Out of range message
void displayOutofRangeMessage(int x, int y)
{
    static int blinkState = 0; // Static variable to retain state

    // Set the free font
    tft.setFreeFont(&FreeSans18pt7b);
    // Get the height of the currently selected font
    int textHeight = tft.fontHeight();

    // Erase the previous message
    drawBlackRectangleToErasePortionOfScreen(x - 65, y - textHeight, 480, 360);

    // Toggle text color
    if (blinkState == 0)
    {
        tft.setTextColor(TFT_RED, TFT_BLACK); // Set text color to green
    }
    else
    {
        tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set text color to black (invisible)
    }
    // Update the blink state
    blinkState = 1 - blinkState; // Toggle between 0 and 1
    // Position the cursor for text
    tft.setCursor(x, y);
    tft.print("Out of Range !!!"); // Print some text using the selected font
    delay(200);
}

void drawBlackRectangleToErasePortionOfScreen(int x1, int y1, int x2, int y2)
{
    // Calculate width and height from the corner coordinates
    int width = x2 - x1;
    int height = y2 - y1;

    // Fill the rectangle with black color
    // tft.fillRect(x1, y1, width, height, TFT_RED); // Change to black to erase properly
    tft.fillRect(x1, y1, width, height, TFT_BLACK); // Change to black to erase properly
}

// Function to set the IC-705 frequency in Hz (e.g., 14201000 for 14.201 MHz)
// Helper function to convert a frequency into BCD format
void set_IC705_frequency(uint32_t frequency)
{
    frequency = frequency * 10;
    // Convert the frequency to a BCD encoded form for each byte
    uint8_t byte_1 = (frequency / 100000000) % 10 * 16 + (frequency / 10000000) % 10; // Extract first 2 digits
    uint8_t byte_2 = (frequency / 1000000) % 10 * 16 + (frequency / 100000) % 10;     // Extract next 2 digits
    uint8_t byte_3 = (frequency / 10000) % 10 * 16 + (frequency / 1000) % 10;         // Extract next 2 digits
    uint8_t byte_4 = (frequency / 100) % 10 * 16 + (frequency / 10) % 10;             // Extract next 2 digits
    uint8_t byte_5 = (frequency % 10) * 16;                                           // Last digit with BCD 0 padding

    // Construct the command with preamble and end byte
    uint8_t command[] = {
        0xFE, 0xFE, 0xA4, 0xE0,                 // Command preamble
        byte_5, byte_4, byte_3, byte_2, byte_1, // Frequency bytes in BCD format
        0x00, 0xFD                              // End of command
    };

    // Send the command via BLE (assuming pRXCharacteristic is properly initialized)
    pRXCharacteristic->writeValue(command, sizeof(command));

    // Optionally, print the command for debugging
    Serial.print("Sent command: ");
    for (int i = 0; i < sizeof(command); i++)
    {
        Serial.printf("0x%02X ", command[i]);
    }
    Serial.println();
}

bool isSlaveConnected()
{
    // Function to check if the slave is connected
    WiFiClient client;
    if (client.connect(slaveIP, 80))
    { // Assuming slave listens on port 80
        client.stop();
        return true;
    }
    return false;
}
// Function to check if tft was touched is within a rectangular region
bool isTouchPointInRegion(int t_x, int t_y, int x1, int y1, int x2, int y2)
{
    // Ensure x1 is the leftmost and x2 is the rightmost
    int left = min(x1, x2);
    int right = max(x1, x2);

    // Ensure y1 is the topmost and y2 is the bottommost
    int top = min(y1, y2);
    int bottom = max(y1, y2);

    // Check if the point (t_x, t_y) is within the rectangle
    return (t_x >= left && t_x <= right && t_y >= top && t_y <= bottom);
}
// function to detect fast double PTT
void checkPTT(bool currentPTTState)
{
    unsigned long currentTime = millis();

    // Detect a rising edge (OFF -> ON)
    if (currentPTTState && !lastPTTState)
    {
        // Check if the time since the last release is within double-click interval
        if (possibleDoubleClick && (currentTime - lastReleaseTime <= DOUBLE_CLICK_MAX_INTERVAL))
        {
            Serial.println("Double PTT detected");
            doublePTTdetected = true;
            possibleDoubleClick = false; // Reset the double-click flag
        }
        else
        {
            // Otherwise, start tracking a potential double-click
            possibleDoubleClick = true;
            lastPressTime = currentTime;
        }
    }

    // Detect a falling edge (ON -> OFF)
    if (!currentPTTState && lastPTTState)
    {
        // Check if the press duration was not a long press
        if ((currentTime - lastPressTime) < LONG_PRESS_THRESHOLD)
        {
            lastReleaseTime = currentTime; // Update the last release time
        }
        else
        {
            // If it was a long press, reset the double-click flag
            possibleDoubleClick = false;
        }
    }

    // Update the last PTT state
    lastPTTState = currentPTTState;
}
// Function to draw the SWR meter with log scale
void SWR_drawLogarithmicSWRbar(float swr)
{
    // Helper function for color interpolation
    auto interpolateColor = [](uint16_t color1, uint16_t color2, float factor) -> uint16_t
    {
        uint8_t r1 = (color1 >> 11) & 0x1F;
        uint8_t g1 = (color1 >> 5) & 0x3F;
        uint8_t b1 = color1 & 0x1F;

        uint8_t r2 = (color2 >> 11) & 0x1F;
        uint8_t g2 = (color2 >> 5) & 0x3F;
        uint8_t b2 = color2 & 0x1F;

        uint8_t r = r1 + (r2 - r1) * factor;
        uint8_t g = g1 + (g2 - g1) * factor;
        uint8_t b = b1 + (b2 - b1) * factor;

        return (r << 11) | (g << 5) | b;
    };

    // Draw the background and ticks once
    if (SWR_first_draw)
    {
        tft.fillRect(0, SWR_BAR_Y, 480, 320 - SWR_BAR_Y, SWR_COLOR_BG); // Background
        SWR_drawTicksAndLabels();                                       // Draw ticks and labels
        SWR_first_draw = false;
    }

    // Map SWR values to bar lengths
    int current_length = SWR_mapSWRToBarLength(swr);
    int previous_length = SWR_mapSWRToBarLength(previous_swr);
    int green_length = SWR_mapSWRToBarLength(2.0);

    // Define gradient color points
    uint16_t colorStart = SWR_COLOR_GOOD; // Green (safe)
    uint16_t colorMid = COLOR_WARNING;    // Yellow (moderate)
    uint16_t colorEnd = COLOR_DANGER;     // Red (high SWR)

    // Determine if we're increasing or decreasing SWR
    if (swr > previous_swr) // SWR increases
    {
        // Draw segments with gradient effect
        for (int x = SWR_BAR_X; x < SWR_BAR_X + current_length; x += SEGMENT_WIDTH)
        {
            float positionFactor = (float)(x - SWR_BAR_X) / SWR_BAR_WIDTH;

            uint16_t color;
            if (positionFactor <= 0.5)
            {
                color = interpolateColor(colorStart, colorMid, positionFactor * 2); // Green to yellow
            }
            else
            {
                color = interpolateColor(colorMid, colorEnd, (positionFactor - 0.5) * 2); // Yellow to red
            }

            tft.fillRect(x, SWR_BAR_Y, SEGMENT_WIDTH, SWR_BAR_HEIGHT, color);
        }
    }
    else if (swr < previous_swr) // SWR decreases
    {
        // Erase excess portions when SWR decreases
        SWR_eraseBar(SWR_BAR_X + current_length, SWR_BAR_X + previous_length);

        // Erase any segments beyond the current length
        for (int x = SWR_BAR_X + current_length + SEGMENT_WIDTH; x <= SWR_BAR_X + previous_length; x += SEGMENT_WIDTH)
        {
            SWR_eraseBar(x, x + SEGMENT_WIDTH); // Clear segments that should be removed
        }
    }

    // Draw the outline of the bar
    tft.drawRect(SWR_BAR_X, SWR_BAR_Y, SWR_BAR_WIDTH, SWR_BAR_HEIGHT, TFT_WHITE);

    // Erase previous SWR value by overwriting with background color
    tft.setFreeFont(&FreeMonoBold18pt7b);
    String prevSWRText = String(previous_swr, 1);
    int text_x = SWR_BAR_X + SWR_BAR_WIDTH + 10;
    int text_y = SWR_BAR_Y + 8;
    tft.setTextColor(SWR_COLOR_BG, TFT_BLACK);
    tft.drawString(prevSWRText, text_x, text_y);

    // Display new SWR value
    String swrText = String(swr, 1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(swrText, text_x, text_y);

    // Update previous SWR value for next cycle
    previous_swr = swr;

    // Apply the segment effect in the filled area if needed
    SWR_drawSegmentsInFilledArea(current_length, green_length);
}
// Function to draw the SWR meter with linear scale
void SWR_drawLinearSWRbar(float swrValue)
{
    // Helper function for color interpolation
    auto interpolateColor = [](uint16_t color1, uint16_t color2, float factor) -> uint16_t
    {
        uint8_t r1 = (color1 >> 11) & 0x1F;
        uint8_t g1 = (color1 >> 5) & 0x3F;
        uint8_t b1 = color1 & 0x1F;

        uint8_t r2 = (color2 >> 11) & 0x1F;
        uint8_t g2 = (color2 >> 5) & 0x3F;
        uint8_t b2 = color2 & 0x1F;

        uint8_t r = r1 + (r2 - r1) * factor;
        uint8_t g = g1 + (g2 - g1) * factor;
        uint8_t b = b1 + (b2 - b1) * factor;

        return (r << 11) | (g << 5) | b;
    };

    // Calculate Y position and dimensions
    int meterY = 320 - segmentHeight - tickLength - 28;
    int fullWidth = numSegments * segmentWidth + (numSegments - 1) * segmentGap;
    int activeSegments = min((int)((swrValue - 1.0) * 10), numSegments);
    int valueX = meterX + fullWidth + 10;
    int valueY = meterY + 8;

    // Draw initial static elements only once
    if (SWR_first_draw)
    {
        tft.drawRect(meterX - 1, meterY - 1, fullWidth + 2, segmentHeight + 2, TFT_WHITE);

        // Draw tick marks and labels
        float tickValues[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
        int tickCount = sizeof(tickValues) / sizeof(tickValues[0]);
        for (int i = 0; i < tickCount; i++)
        {
            int tickPosition = meterX + (int)((tickValues[i] - 1.0) * 10 * (segmentWidth + segmentGap));
            tft.drawFastVLine(tickPosition, meterY + segmentHeight + 1, tickLength, TFT_WHITE);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setFreeFont(&FreeMonoBold9pt7b);
            char label[4];
            snprintf(label, sizeof(label), "%.1f", tickValues[i]);
            tft.drawString(label, tickPosition - 17, meterY + segmentHeight + tickLength + 5);
        }
        SWR_first_draw = false;
    }

    // Define gradient color points
    uint16_t colorStart = TFT_GREEN; // Start (safe) color
    uint16_t colorMid = TFT_YELLOW;  // Middle (caution) color
    uint16_t colorEnd = TFT_RED;     // End (warning) color

    // Draw segments with gradient
    for (int i = 0; i < numSegments; i++)
    {
        int x = meterX + i * (segmentWidth + segmentGap);

        // Determine color using gradient interpolation
        float factor;
        uint16_t color;
        if (i < numSegments / 2)
        { // Green to yellow
            factor = (float)i / (numSegments / 2);
            color = interpolateColor(colorStart, colorMid, factor);
        }
        else
        { // Yellow to red
            factor = (float)(i - numSegments / 2) / (numSegments / 2);
            color = interpolateColor(colorMid, colorEnd, factor);
        }

        // Fill or clear segment
        if (i < activeSegments)
        {
            tft.fillRect(x, meterY, segmentWidth, segmentHeight, color);
        }
        else
        {
            tft.fillRect(x, meterY, segmentWidth, segmentHeight, TFT_BLACK);
        }
    }

    // Display the SWR value with color-coded text
    if (previousSWRValue >= 1.0 && abs(previousSWRValue - swrValue) > 0.1)
    {
        tft.setTextColor(TFT_BLACK, TFT_BLACK);
        tft.setFreeFont(&FreeMonoBold18pt7b);
        char previousValueLabel[5];
        snprintf(previousValueLabel, sizeof(previousValueLabel), "%.1f", previousSWRValue);
        tft.drawString(previousValueLabel, valueX, valueY);
    }

    uint16_t swrColor = (swrValue < 2.0) ? TFT_GREEN : (swrValue < 3.0) ? TFT_YELLOW
                                                                        : TFT_RED;
    if (previousSWRValue != swrValue)
    {
        tft.setTextColor(swrColor, TFT_BLACK);
        tft.setFreeFont(&FreeMonoBold18pt7b);
        char currentValueLabel[5];
        snprintf(currentValueLabel, sizeof(currentValueLabel), "%.1f", swrValue);
        tft.drawString(currentValueLabel, valueX, valueY);
    }
    previousSWRValue = swrValue;
}
// function for testing / debugging
void test()
{
    Serial.println("");
    Serial.println("Touched Test Area");
    Serial.println("");
    // SWR_drawLinearSWRbar(4);
    // SWR_drawLogarithmicSWRbar(4);
}
