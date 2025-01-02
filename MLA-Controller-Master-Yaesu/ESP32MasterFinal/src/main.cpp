#include <myConfig.h>
#include <Arduino.h>
#include "BLEDevice.h"
#include "bluetoothManagement.h"
#include "FS.h"
#include <TFT_eSPI.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "Digital_7Mono40pt7b.h"
#include "arial10pt7b.h"
#include "technology_bold18pt7b.h"

int xRectangle; // TO BE REPLACED LATER WITH NUMBERS
int wRectangle; // TO BE REPLACED LATER WITH NUMBERS
// Display Related
TFT_eSPI tft = TFT_eSPI();
#define TFT_CALIBRATION_FILE "/calibration.dat"
// WIFI RElated
#include <WiFi.h>
#define SSID "HB9IIU-MLA"          // WiFi SSID to connect to
IPAddress slaveIP(192, 168, 4, 1); // IP address of the SLAVE device to send HTTP commands to
HTTPClient http;
//------------------------------------------------------------------------------
// String currentModeName; // Will store the current mode value
// volatile uint8_t setRFPower;
// volatile unsigned long CurrentVFOFrequency = 0; // VFO frequency in Hz
volatile unsigned long previousVFOFrequency;
// volatile float currentSWR;
// volatile bool PTTstatus = false;

uint32_t currentStepperPosition = 0;       // current stepper position
int64_t theoreticalResonanceFrequency = 0; // theoretical resonance
uint32_t predictedStepperPosition = 0;     // predicted stepper position from lookup table
uint32_t deltaSteps = 0;                   // diff between current stepper position and predicted stepper position

unsigned long lastVFOfrequencyChangeTime;
bool allStepperInfotoBeRedrawn = false;
bool forceRedwrawOutofRangeWarning = true;
bool SWRtest = false;
// long estimated_movement_duration_in_microseconds;

// MILLIS related
// unsigned long lastVFOfrequencyChangeTime;
//------------------------------------------------------------------------------
// Prototype Declarations
String getModeCodeFromName(const String &modeName);
// Wifi Related
void establishWIFIconnectionWithSlave();
void getWIFIinkQuality(int8_t rssi, String &description, uint16_t &color);
// TFT display related
void initTFTscreen();
bool initializeSPIFFS();
void calibrateTFTscreen();
void checkAndApplyTFTCalibrationData(bool recalibrate);
void printOnTFT(const char *text, uint16_t textColor, uint16_t backgroundColor);
String formatStepperPosForConsoleOutput(uint32_t value);
char *formatFrequencyForConsoleOutput(uint64_t vfo);
void GetTunedStatusFromSlave();
void drawVariableCapacitor(int x, int y, float scale, uint16_t color);
String formatStepsToGoForConsole(int32_t value);
void setNewPositionForCurrentVFOfrequency(uint32_t targetFrequency);
bool VFOfrequencyIsInAdmissibleRange(unsigned long frequency);
bool redraw_swr = false;
// bool freqIsOutOfRange = false;
// bool freqBackInRange = true;
// bool allStepperInfotoBeRedrawn = false;
//--------------------------------------------------------------------------------
//  TFT Display related
void initTFTscreen()
{
    tft.init();
    pinMode(TFT_BLP, OUTPUT);
    digitalWrite(TFT_BLP, HIGH); // Turn on backlight
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
}

bool initializeSPIFFS()
{
    if (!SPIFFS.begin())
    {
        Serial.println("SPIFFS mount failed. Formatting SPIFFS...");
        if (SPIFFS.format())
        {
            Serial.println("SPIFFS formatted successfully. Re-initializing...");
            if (SPIFFS.begin())
            {
                Serial.println("SPIFFS mounted successfully.");
                return true;
            }
            else
            {
                Serial.println("SPIFFS re-initialization failed.");
                return false;
            }
        }
        else
        {
            Serial.println("SPIFFS formatting failed!");
            return false;
        }
    }
    Serial.println("SPIFFS mounted successfully.");
    return true;
}

void calibrateTFTscreen()
{
    uint16_t calibrationData[5];

    // Display recalibration message
    tft.fillScreen(TFT_BLACK);             // Clear screen
    tft.setTextColor(TFT_BLACK, TFT_GOLD); // Set text color (white on black)
    tft.setFreeFont(&FreeSansBold12pt7b);  // Use custom free font for better visibility
    tft.setCursor(10, 22);
    tft.fillRect(0, 0, 480, 30, TFT_GOLD);
    tft.print("   TFT TOUCHSCREEN CALIBRATION");
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color (white on black)

    tft.setFreeFont(&FreeSans12pt7b); // Use custom free font for better visibility

    // Corrected and split text to display
    String line1 = "Welcome to the initial touchscreen calibration.";
    String line2 = "This procedure will only be required once.";
    String line3 = "On the next screen you will see arrows";
    String line4 = "appearing one after the other at each corner.";
    String line5 = "Just tap them until completion.";
    String line6 = "";                                    // Empty line
    String line7 = "Tap anywhere on the screen to begin"; // Final line in green

    // Variables to manage line positions
    int16_t yPos = 100; // Vertically center the text block

    // Print each line, center it horizontally
    String lines[] = {line1, line2, line3, line4, line5, line6, line7}; // Array to hold the lines
    for (int i = 0; i < 6; i++)
    { // Print lines 1-6 in white
        int16_t xPos = (tft.width() - tft.textWidth(lines[i])) / 2;
        tft.setCursor(xPos, yPos);
        tft.print(lines[i]);
        yPos += tft.fontHeight();
    }

    // Print the final line (line 7) in TFT_GREEN
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    int16_t xPos = (tft.width() - tft.textWidth(line7)) / 2;
    tft.setCursor(xPos, yPos);
    tft.print(line7);

    // Wait for touch input before continuing
    while (true)
    {
        uint16_t x, y;
        if (tft.getTouch(&x, &y))
        {
            break; // Exit the loop on touch
        }
    }
    // Erase the screen
    tft.fillScreen((TFT_BLACK)); // Fill the screen

    // Perform the calibration process
    tft.calibrateTouch(calibrationData, TFT_GREEN, TFT_BLACK, 12);

    // Store the calibration data in SPIFFS
    File f = SPIFFS.open(TFT_CALIBRATION_FILE, "w");
    if (f)
    {
        f.write((const unsigned char *)calibrationData, 14);
        f.close();
    }

    uint16_t x, y;
    int16_t lastX = -1;     // Last x position of the text
    int16_t lastY = -1;     // Last y position of the text
    String lastResult = ""; // Last printed result string

    // Draw a dynamically sized exit button
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(4);
    String Text = "Tap anywhere to test";
    tft.setCursor((tft.width() - tft.textWidth(Text)) / 2, 8);
    tft.print(Text);

    Text = "Click Here to Exit";
    int16_t btn_w = tft.textWidth(Text) + 20;  // Dynamic button width
    int16_t btn_h = tft.fontHeight() + 12;     // Dynamic button height
    int16_t btn_x = (tft.width() - btn_w) / 2; // Center the button
    int16_t btn_y = 280;                       // Fixed vertical position
    int16_t btn_r = 10;                        // Corner radius

    tft.fillRoundRect(btn_x, btn_y, btn_w, btn_h, btn_r, TFT_BLUE);
    tft.drawRoundRect(btn_x, btn_y, btn_w, btn_h, btn_r, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setCursor((tft.width() - tft.textWidth(Text)) / 2, btn_y + 10);
    tft.print(Text);

    while (true)
    {

        if (tft.getTouch(&x, &y))
        {
            // Generate the new result string
            String result = "x=" + String(x) + "  " + "y=" + String(y);

            // Calculate text width and height
            int16_t textWidth = tft.textWidth(result);
            int16_t textHeight = tft.fontHeight();

            // Calculate the x and y positions to center the text
            int16_t xPos = (tft.width() - textWidth) / 2;
            int16_t yPos = (tft.height() - textHeight) / 2;

            // Set the text color to black to "erase" previous text
            tft.setTextColor(TFT_BLACK, TFT_BLACK);
            tft.setCursor(lastX, lastY);
            tft.print(lastResult); // Redraw the old result in black

            // Set the cursor to the calculated position
            tft.setCursor(xPos, yPos);

            // Print the new result
            tft.setTextColor(TFT_GREEN, TFT_BLACK); // Set text color for new text (e.g., white)
            tft.print(result);

            // Update the last printed position and result
            lastX = xPos;
            lastY = yPos;
            lastResult = result;

            // Draw a filled circle at the touch position
            tft.fillCircle(x, y, 2, TFT_RED);

            // Check if touch is within the exit button area
            if (x > btn_x && x < (btn_x + btn_w) && y > btn_y && y < (btn_y + btn_h))
            {
                Serial.println("Exit Touched");
                break;
            }
        }
    }
    tft.fillScreen(TFT_BLACK); // Clear screen
    printOnTFT("TFT display calibration applied", TFT_GREEN, TFT_BLACK);
}

void checkAndApplyTFTCalibrationData(bool recalibrate)
{

    if (recalibrate == true)
    {
        calibrateTFTscreen();
    }
    File f = SPIFFS.open(TFT_CALIBRATION_FILE, "r");
    uint16_t calibrationData[5];
    if (f && f.readBytes((char *)calibrationData, 14) == 14)
    {
        tft.setTouch(calibrationData);
        Serial.println("Calibration data applied.");
        printOnTFT("TFT display calibration applied", TFT_GREEN, TFT_BLACK);
        f.close();
    }
    else
    {
        Serial.println("Invalid calibration data. Recalibrating...");
        calibrateTFTscreen();
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

// Wifi related
void establishWIFIconnectionWithSlave()
{
    Serial.print("Establishing WiFi Connection with Slave...");
    printOnTFT("Connecting To Slave via WiFi...", TFT_YELLOW, TFT_BLACK);

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
        getWIFIinkQuality(rssi, qualityDescription, qualityColor);
        Serial.println("\nSuccessful Wifi Connection to Slave");
        printOnTFT("Successful Connection to Slave", TFT_GREEN, TFT_BLACK);
        Serial.print("Signal Quality:");
        Serial.println(qualityDescription.c_str());
        String message = "Link Quality: " + qualityDescription;
        printOnTFT(message.c_str(), qualityColor, TFT_BLACK);
        // Initialize HTTPClient
        String url = String("http://") + slaveIP.toString() + "/command";
        http.begin(url);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    }
    else
    {
        Serial.println("\nConnection failed, rebooting in 3 seconds");
        printOnTFT("No WiFi Connection to Slave", TFT_RED, TFT_BLACK);
        printOnTFT("Please check your Slave", TFT_YELLOW, TFT_BLACK);
        printOnTFT("", TFT_YELLOW, TFT_BLACK);
        printOnTFT("Re-Booting in 3 seconds", TFT_RED, TFT_BLACK);
        delay(3000);
        ESP.restart();
    }
}

void getWIFIinkQuality(int8_t rssi, String &description, uint16_t &color)
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

//-------------------- TFT DISPLAY FUNCTION ----------------------------------------------

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

void updateWiFiWidget(int x, int y, int radius, float sizePercentage, int rssi, bool forceredraw)
{
    static int previousRSSILevel = -1; // Track the previous RSSI level to avoid unnecessary redraws
                                       /* if (frequencyDisplaystoBeRedrawn)
                                        { // in case of a reconnection
                                          previousRSSILevel = -1;
                                        }
                                        */
    // Function to convert degrees to radians
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
    tft.setFreeFont(&arial10pt7b);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // White text with black background to clear previous text
    tft.setCursor(x - 19, y + 24);          // Position the text below the Wi-Fi icon
    tft.fillRect(x - 31, y + 9, 45, 20, TFT_BLACK);
    tft.printf("%d", rssi);        // Print the RSSI value in dBm
    tft.setCursor(x - 23, y + 45); // Position the text below the Wi-Fi icon
    tft.printf("dBm");             // Print the RSSI value in dBm
}

void displayVFOfrequency(unsigned long frequency, int startX, int yPosition, bool forceRedraw)
{
    constexpr int numDigits = 8;
    static char previousFrequArr[9] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    char frequArr[numDigits + 1] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    static bool isInitiated = false;
    static int xPositions[8];
    constexpr int rectOffset = 63;
    constexpr int rectHeight = 79;
    constexpr int dialColor = TFT_BLACK;
    constexpr int digitColor = TFT_GREEN;
    constexpr const char *dialLabel = "  VFO Frequency  ";
    constexpr int deadSegmentColor = 0x4208; // RGB(66, 66, 66) - Medium dark gray  0x630C
    // constexpr int darkGrey = 0x4208; // RGB(33, 33, 33) - A very dark gray
    constexpr int darkGrey = 0x528A; // RGB(50, 50, 50) - Slightly lighter dark gray
                                     // constexpr int darkGrey = 0x8410; // RGB(100, 100, 100) - The lightest of the dark grays
    // constexpr int darkGrey = 0x738E; // RGB(83, 83, 83) - Lighter medium gray

    tft.setFreeFont(&Digital_7Mono40pt7b);
    if (!isInitiated || forceRedraw)
    {

        int charWidth = tft.textWidth("8") + 5;
        int dotWidth = tft.textWidth(".");
        xPositions[0] = startX;

        for (int i = 1; i < numDigits; i++)
        {
            xPositions[i] = xPositions[i - 1] + charWidth + ((i == 2 || i == 5) ? dotWidth : 0);
        }

        int xRect = startX - charWidth / 2 + 8;
        int yRect = yPosition - rectOffset;
        int wRect = xPositions[7] + 2 * charWidth - startX - 16;
        xRectangle = xRect;
        wRectangle = wRect;

        tft.drawRoundRect(xRect - 1, yRect - 1, wRect + 2, rectHeight + 2, 11, TFT_LIGHTGREY);
        tft.drawRoundRect(xRect - 2, yRect - 2, wRect + 4, rectHeight + 4, 12, TFT_LIGHTGREY);

        tft.setTextColor(digitColor);
        tft.setCursor(xPositions[1] + 28, yPosition);
        tft.print(".");
        tft.setCursor(xPositions[4] + 28, yPosition);
        tft.print(".");

        tft.setTextFont(4);
        tft.setTextColor(digitColor, dialColor);
        tft.setCursor(xRect + wRect / 2 - tft.textWidth(dialLabel) / 2, yRect + rectHeight - 7);
        tft.print(dialLabel);

        tft.setFreeFont(&Digital_7Mono40pt7b);
        tft.setTextColor(deadSegmentColor);

        for (int i = 0; i < numDigits; i++)
        {
            tft.setCursor(xPositions[i], yPosition);
            tft.print("8"); // Draw the switched OFF digit
        }
        isInitiated = true;
    }

    // Extract digits from most significant to least significant
    unsigned long tempFreq = frequency;
    for (int i = numDigits - 1; i >= 0; i--)
    {
        frequArr[i] = (tempFreq % 10) + '0';
        tempFreq /= 10;
    }
    // Suppress leading zeros
    bool leadingZero = true;
    for (int i = 0; i < numDigits; i++)
    {
        if (leadingZero && frequArr[i] == '0')
        {
            frequArr[i] = ' '; // Replace leading zero with space
        }
        else
        {
            leadingZero = false;
        }
    }
    // Update only the digits that have changed
    for (int i = 0; i < numDigits; i++)
    {
        if (frequArr[i] != previousFrequArr[i])
        {
            // Erase old digit
            tft.setCursor(xPositions[i], yPosition);
            tft.setTextColor(dialColor); // Use background color to "erase"
            tft.print(previousFrequArr[i]);

            // Draw new digit
            tft.setCursor(xPositions[i], yPosition);
            tft.setTextColor(digitColor);
            tft.print(frequArr[i]);

            // Draw the switched OFF digit
            tft.setTextColor(darkGrey);

            tft.setCursor(xPositions[i], yPosition);
            switch (frequArr[i])
            {
            case '1':
                tft.print('A');
                break;
            case '2':
                tft.print('B');
                break;
            case '3':
                tft.print('C');
                break;
            case '4':
                tft.print('D');
                break;
            case '5':
                tft.print('E');
                break;
            case '6':
                tft.print('F');
                break;
            case '7':
                tft.print('G');
                break;
            case '8':
                // tft.print('H');
                break;
            case '9':
                tft.print('I');
                break;
            case '0':
                tft.print('@');
                break;
            }

            // Update previous digit array
            previousFrequArr[i] = frequArr[i];
        }
        if (frequency < 10000000)
        {
            // Draw the 1st switched OFF digit
            tft.setTextColor(darkGrey);
            tft.setCursor(xPositions[0], yPosition);
            tft.print('8');
        }
    }
}

void displayRESONANCEfrequency(unsigned long frequency, int startX, int yPosition, bool forceRedraw)
{
    constexpr int numDigits = 8;
    static char previousFrequArr[9] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    char frequArr[numDigits + 1] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', '\0'};
    static bool isInitiated = false;
    static int xPositions[8];
    constexpr int rectOffset = 63;
    constexpr int rectHeight = 79;
    constexpr int dialColor = TFT_BLACK;
    constexpr int digitColor = TFT_CYAN;
    constexpr const char *dialLabel = "  Resonance Frequency  ";
    constexpr int deadSegmentColor = 0x4208; // RGB(66, 66, 66) - Medium dark gray  0x630C
    // constexpr int darkGrey = 0x4208;         // RGB(33, 33, 33) - A very dark gray
    constexpr int darkGrey = 0x528A; // RGB(50, 50, 50) - Slightly lighter dark gray
    // constexpr int darkGrey = 0x8410; // RGB(100, 100, 100) - The lightest of the dark grays
    // constexpr int darkGrey = 0x738E; // RGB(83, 83, 83) - Lighter medium gray

    tft.setFreeFont(&Digital_7Mono40pt7b);

    if (!isInitiated || forceRedraw)
    {
        int charWidth = tft.textWidth("8") + 5;
        int dotWidth = tft.textWidth(".");
        xPositions[0] = startX;

        for (int i = 1; i < numDigits; i++)
        {
            xPositions[i] = xPositions[i - 1] + charWidth + ((i == 2 || i == 5) ? dotWidth : 0);
        }

        int xRect = startX - charWidth / 2 + 8;
        int yRect = yPosition - rectOffset;
        int wRect = xPositions[7] + 2 * charWidth - startX - 16;

        tft.drawRoundRect(xRect - 1, yRect - 1, wRect + 2, rectHeight + 2, 11, TFT_LIGHTGREY);
        tft.drawRoundRect(xRect - 2, yRect - 2, wRect + 4, rectHeight + 4, 12, TFT_LIGHTGREY);

        tft.setTextColor(digitColor);
        tft.setCursor(xPositions[1] + 28, yPosition);
        tft.print(".");
        tft.setCursor(xPositions[4] + 28, yPosition);
        tft.print(".");

        tft.setTextFont(4);
        tft.setTextColor(digitColor, dialColor);
        tft.setCursor(xRect + wRect / 2 - tft.textWidth(dialLabel) / 2, yRect + rectHeight - 7);
        tft.print(dialLabel);

        tft.setFreeFont(&Digital_7Mono40pt7b);
        tft.setTextColor(deadSegmentColor);

        for (int i = 0; i < numDigits; i++)
        {
            tft.setCursor(xPositions[i], yPosition);
            tft.print("8"); // Draw the switched OFF digit
        }
        isInitiated = true;
    }

    // Extract digits from most significant to least significant
    unsigned long tempFreq = frequency;
    for (int i = numDigits - 1; i >= 0; i--)
    {
        frequArr[i] = (tempFreq % 10) + '0';
        tempFreq /= 10;
    }
    // Suppress leading zeros
    bool leadingZero = true;
    for (int i = 0; i < numDigits; i++)
    {
        if (leadingZero && frequArr[i] == '0')
        {
            frequArr[i] = ' '; // Replace leading zero with space
        }
        else
        {
            leadingZero = false;
        }
    }
    // Update only the digits that have changed
    for (int i = 0; i < numDigits; i++)
    {
        if (frequArr[i] != previousFrequArr[i])
        {
            // Erase old digit
            tft.setCursor(xPositions[i], yPosition);
            tft.setTextColor(dialColor); // Use background color to "erase"
            tft.print(previousFrequArr[i]);

            // Draw new digit
            tft.setCursor(xPositions[i], yPosition);
            tft.setTextColor(digitColor);
            tft.print(frequArr[i]);

            // Draw the switched OFF digit
            tft.setTextColor(darkGrey);

            tft.setCursor(xPositions[i], yPosition);
            switch (frequArr[i])
            {
            case '1':
                tft.print('A');
                break;
            case '2':
                tft.print('B');
                break;
            case '3':
                tft.print('C');
                break;
            case '4':
                tft.print('D');
                break;
            case '5':
                tft.print('E');
                break;
            case '6':
                tft.print('F');
                break;
            case '7':
                tft.print('G');
                break;
            case '8':
                // tft.print('H');
                break;
            case '9':
                tft.print('I');
                break;
            case '0':
                tft.print('@');
                break;
            }

            // Update previous digit array
            previousFrequArr[i] = frequArr[i];
        }
        if (frequency < 10000000)
        {
            // Draw the 1st switched OFF digit
            tft.setTextColor(darkGrey);
            tft.setCursor(xPositions[0], yPosition);
            tft.print('8');
        }
    }
}

void displaySetMode(String mode, int x, int y, uint16_t fontColor, uint16_t rectColor, uint8_t rectLineWidth, bool inverted, bool redraw)
{

    static String lastCurrentModeName; // Will store the current mode value

    // skip if same
    if (currentModeName == lastCurrentModeName && redraw == false)
    {
        return;
    }
    lastCurrentModeName = currentModeName;

    int fontSize = 4; // Set font size
    tft.setTextFont(fontSize);

    // Split the string at the first hyphen
    int hyphenIndex = mode.indexOf('-');
    String line1, line2;

    if (hyphenIndex != -1)
    {
        // If hyphen is found, split into two parts
        line1 = mode.substring(0, hyphenIndex);  // Part before hyphen
        line2 = mode.substring(hyphenIndex + 1); // Part after hyphen
        if (line2 == "U")
        {
            line2 = "USB";
        }
        if (line2 == "L")
        {
            line2 = "LSB";
        }
    }
    else
    {
        // If no hyphen, treat as a single-line mode
        line1 = mode;
        line2 = "";
    }

    // Calculate the width and height for each line
    int textWidth1 = tft.textWidth(line1, fontSize);
    int textWidth2 = tft.textWidth(line2, fontSize);
    int textHeight = tft.fontHeight(fontSize);

    // Determine rectangle dimensions
    int rectWidth = 86;                                                       // Fixed width for both single and multi-line modes
    int rectHeight = line2 == "" ? (textHeight + 10) : (textHeight * 2 + 10); // Adjust height for single or multi-line
    int rectEraseheight = textHeight * 2 + 10 + 2;
    int rectX = x - rectWidth / 2; // Top-left x-coordinate
    int rectY = y - 5;             // Top-left y-coordinate, adding padding

    // Clear the area where the text will be printed
    tft.fillRect(rectX - 5, rectY - 2, rectWidth + 10, rectEraseheight, TFT_BLACK);

    if (inverted)
    {
        // Inverted mode: Filled rectangle with `rectColor` and black text
        tft.fillRoundRect(rectX - 3, rectY - 4, rectWidth + 6, rectHeight + 1, 5, rectColor);

        // Draw outline if rectLineWidth > 1
        for (int i = 0; i < rectLineWidth; i++)
        {
            tft.drawRoundRect(rectX - 3 - i, rectY - 4 - i, rectWidth + 6 + 2 * i, rectHeight + 1 + 2 * i, 5, fontColor);
        }

        // Set the font color to black for inverted text
        tft.setTextColor(TFT_BLACK);
    }
    else
    {
        // Normal mode: Rectangle outline with `rectColor` and colored text
        for (int i = 0; i < rectLineWidth; i++)
        {
            tft.drawRoundRect(rectX - 3 - i, rectY - 4 - i, rectWidth + 6 + 2 * i, rectHeight + 1 + 2 * i, 5, rectColor);
        }

        // Set the font color to the specified value for normal text
        tft.setTextColor(fontColor);
    }

    // Print the first line (centered horizontally)
    tft.setCursor(x - textWidth1 / 2, y);
    tft.print(line1);

    // Print the second line if applicable (centered horizontally and below the first line)
    if (line2 != "")
    {
        tft.setCursor(x - textWidth2 / 2, y + textHeight);
        tft.print(line2);
    }
}

void displaySetRFpower(int power, int x, int y, uint16_t fontColor, uint16_t rectColor, uint8_t rectLineWidth, bool inverted)
{
    static uint8_t lastsetRFPower;
    // skip if same
    if (power == lastsetRFPower)
    {
        return;
    }
    lastsetRFPower = power;

    int fontSize = 4; // Set font size
    tft.setTextFont(fontSize);

    // Format power as "XX W"
    String powerText = String(power) + " W";

    // Calculate the width and height of the rectangle
    int rectWidth = 86;                             // Fixed width to fit "100 W"
    int rectHeight = tft.fontHeight(fontSize) + 10; // Height for one line + padding
    int rectX = x - rectWidth / 2;                  // Top-left x-coordinate
    int rectY = y - 5;                              // Top-left y-coordinate, adding padding

    // Calculate the width of the text
    int textWidth = tft.textWidth(powerText, fontSize);

    // Clear the area where the text will be printed
    tft.fillRect(rectX - 5, rectY - 2, rectWidth + 10, rectHeight + 4, TFT_BLACK);

    if (inverted)
    {
        // Inverted mode: Filled rectangle with `rectColor` and black text
        tft.fillRoundRect(rectX - 3, rectY - 4, rectWidth + 6, rectHeight + 1, 5, rectColor);

        // Draw outline if rectLineWidth > 1
        for (int i = 0; i < rectLineWidth; i++)
        {
            tft.drawRoundRect(rectX - 3 - i, rectY - 4 - i, rectWidth + 6 + 2 * i, rectHeight + 1 + 2 * i, 5, fontColor);
        }

        // Set the font color to black for inverted text
        tft.setTextColor(TFT_BLACK);
    }
    else
    {
        // Normal mode: Rectangle outline with `rectColor` and colored text
        for (int i = 0; i < rectLineWidth; i++)
        {
            tft.drawRoundRect(rectX - 3 - i, rectY - 4 - i, rectWidth + 6 + 2 * i, rectHeight + 1 + 2 * i, 5, rectColor);
        }

        // Set the font color to the specified value for normal text
        tft.setTextColor(fontColor);
    }

    // Print the power text (centered horizontally)
    tft.setCursor(x - textWidth / 2, y);
    tft.print(powerText);
}

void displayRXTXstatus(int x, int y, bool pttStatus)
{
    static bool lastPTTstatus = true;
    // skip if same
    if (pttStatus == lastPTTstatus)
    {
        return;
    }
    lastPTTstatus = pttStatus;

    tft.setTextFont(4);

    // Calculate the width and height of the rectangle
    int rectWidth = 86;                      // Fixed width to fit "100 W"
    int rectHeight = tft.fontHeight(4) + 10; // Height for one line + padding
    int rectX = x - rectWidth / 2;           // Top-left x-coordinate
    int rectY = y - 5;                       // Top-left y-coordinate, adding padding

    // Calculate the width of the text
    int textWidth = tft.textWidth("TX OFF", 4);

    // Clear the area where the text will be printed
    tft.fillRect(rectX - 5, rectY - 2, rectWidth + 10, rectHeight + 4, TFT_BLACK);

    // Normal mode: Rectangle outline with `rectColor` and colored text
    int PTTcolor = TFT_GREEN;
    if (pttStatus)
    {
        PTTcolor = TFT_RED;
    }
    for (int i = 0; i < 2; i++)
    {
        tft.drawRoundRect(rectX - 3 - i, rectY - 4 - i, rectWidth + 6 + 2 * i, rectHeight + 1 + 2 * i, 5, PTTcolor);
    }

    // Set the font color to the specified value for normal text
    String text = "TX ON";
    tft.setCursor(x - textWidth / 2 + 5, y);
    tft.setTextColor(PTTcolor);
    if (!pttStatus)
    {

        textWidth = tft.textWidth("TX OFF", 4);
        text = "TX OFF";
        tft.setCursor(x - textWidth / 2, y);
    }
    // Print the power text (centered horizontally)

    tft.print(text);
}

void displayStepperInfo(int x, int y, long currentPos, long stepsToGo, bool forceRedwraw)
{
    // Nested function for formatting numbers
    auto formatNumber = [](long number, char *output)
    {
        memset(output, ' ', 10);
        output[10] = '\0'; // Null-terminate the string

        char sign = (number < 0) ? '-' : '+';
        unsigned long absNumber = abs(number);

        char tempBuffer[11];
        int tempIndex = 10;
        tempBuffer[tempIndex--] = '\0';

        int digitCount = 0;
        do
        {
            if (digitCount > 0 && digitCount % 3 == 0)
            {
                tempBuffer[tempIndex--] = '.'; // Add a thousand separator
            }
            tempBuffer[tempIndex--] = '0' + (absNumber % 10);
            absNumber /= 10;
            digitCount++;
        } while (absNumber > 0);

        tempBuffer[tempIndex--] = sign;

        const char *start = &tempBuffer[tempIndex + 1];
        int formattedLength = strlen(start);
        strncpy(&output[10 - formattedLength], start, formattedLength);
    };

    // Static variables for state management
    static char previousStepperPositionArray[11] = "          "; // Initialize with spaces
    static char previousStepsToGoArray[11] = "          ";       // Initialize with spaces

    char currentStepperPositionArray[11];
    char currentStepsToGoArray[11];

    int ygap = 35;
    static int xPositions[10];
    int digitWidth = 18;
    int gap = 8;
    static bool isInitiated = false;
    int digitColor = TFT_ORANGE;
    int bgColor = TFT_BLACK;
    if (isInitiated == false || forceRedwraw || allStepperInfotoBeRedrawn)
    {
        forceRedwrawOutofRangeWarning = true;

        memset(previousStepperPositionArray, ' ', sizeof(previousStepperPositionArray) - 1);
        previousStepperPositionArray[sizeof(previousStepperPositionArray) - 1] = '\0'; // to reset

        memset(previousStepsToGoArray, ' ', sizeof(previousStepsToGoArray) - 1);
        previousStepsToGoArray[sizeof(previousStepsToGoArray) - 1] = '\0'; // to reset

        xPositions[0] = x + 150;
        xPositions[1] = xPositions[0] + digitWidth;
        xPositions[2] = xPositions[1] + digitWidth;
        xPositions[3] = xPositions[2] + digitWidth - gap;
        xPositions[4] = xPositions[3] + digitWidth;
        xPositions[5] = xPositions[4] + digitWidth;
        xPositions[6] = xPositions[5] + digitWidth;
        xPositions[7] = xPositions[6] + digitWidth - gap;
        xPositions[8] = xPositions[7] + digitWidth;
        xPositions[9] = xPositions[8] + digitWidth;

        // clear the area first
        tft.fillRoundRect(xRectangle - 1, y - 35, wRectangle + 2, 85, 11, TFT_BLACK);

        tft.setTextColor(digitColor, bgColor);
        tft.setTextDatum(TL_DATUM); // Reset datum to the default Top-Left
        tft.setFreeFont(&arial10pt7b);
        tft.drawString("Current Pos: ", x + 50, y - 15);
        tft.drawString("Steps to Go: ", x + 50, y + ygap - 15);

        // tft.drawRoundRect(xRect - 1, yRect - 1, wRect + 2, rectHeight + 2, 11, TFT_LIGHTGREY);

        tft.drawRoundRect(xRectangle - 1, y - 35, wRectangle + 2, 85, 11, TFT_LIGHTGREY);
        tft.drawRoundRect(xRectangle - 2, y - 35 - 1, wRectangle + 4, 85, 11, TFT_LIGHTGREY);
        // Draw the capacitor at position (10, 10) with scale 1.6
        drawVariableCapacitor(x - 20, y - 20, 1.6, digitColor);
        isInitiated = true;
        allStepperInfotoBeRedrawn = false;
    }

    // Format the current position
    formatNumber(currentPos, currentStepperPositionArray);
    tft.setFreeFont(&technology_bold18pt7b);

    // Compare and update current position
    for (int i = 0; i < 11 && currentStepperPositionArray[i] != '\0'; i++)
    {
        if (currentStepperPositionArray[i] != previousStepperPositionArray[i])
        {
            tft.setCursor(xPositions[i], y);
            tft.setTextColor(bgColor, bgColor);
            tft.print(previousStepperPositionArray[i]);
            tft.setCursor(xPositions[i], y);
            tft.setTextColor(digitColor, bgColor);
            tft.print(currentStepperPositionArray[i]);
        }
    }

    strncpy(previousStepperPositionArray, currentStepperPositionArray, 11);

    // Debugging output
    // Serial.println(currentStepperPositionArray);

    //----------------------------------------------------

    // Format the steps to go
    formatNumber(stepsToGo, currentStepsToGoArray);

    // Compare and update steps to go
    for (int i = 0; i < 11 && currentStepsToGoArray[i] != '\0'; i++)
    {
        if (currentStepsToGoArray[i] != previousStepsToGoArray[i])
        {
            tft.setCursor(xPositions[i], y + ygap);
            tft.setTextColor(bgColor, bgColor);
            tft.print(previousStepsToGoArray[i]);
            tft.setCursor(xPositions[i], y + ygap);
            tft.setTextColor(digitColor, bgColor);
            tft.print(currentStepsToGoArray[i]);
        }
    }
    strncpy(previousStepsToGoArray, currentStepsToGoArray, 11);
}

void displayOutOfRangeWarning(int x, int y)
{
    static bool isInitiated = false;

    // Initial drawing or forced redraw
    if (!isInitiated || forceRedwrawOutofRangeWarning)
    {
        // Clear the area first
        tft.fillRoundRect(xRectangle + 5, y - 25, wRectangle - 14, 70, 11, TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextDatum(TL_DATUM); // Reset datum to the default Top-Left

        forceRedwrawOutofRangeWarning = false; // Reset redraw flag
        allStepperInfotoBeRedrawn = true;
        isInitiated = true;
    }

    // Blinking logic
    static unsigned long previousMillis = 0; // Tracks the last time text visibility changed
    static bool textVisible = true;          // Toggles text visibility

    unsigned long currentMillis = millis();
    tft.setFreeFont(&FreeSansBold18pt7b);

    // Check if 500ms have passed
    if (currentMillis - previousMillis >= 300)
    {
        previousMillis = currentMillis; // Update the time
        textVisible = !textVisible;     // Toggle text visibility

        if (textVisible)
        {
            // Draw the text
            tft.setTextColor(TFT_RED);
            tft.drawString("Out of Range !!!", x + 20, y - 10);
        }
        else
        {
            // Draw the text
            tft.setTextColor(TFT_BLACK);
            tft.drawString("Out of Range !!!", x + 20, y - 10);
        }
    }
}

void displayProgressBar(int x, int y, int givenWidth, int height, int duration)
{
    // Track the total start time
    unsigned long overallStartTime = millis();

    // Clear the area for the progress bar
    tft.fillRect(0, 220, 480, 320 - 200, TFT_BLACK);

    // clearBottomOfDisplay();

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

    unsigned long startTime = overallStartTime;
    unsigned long currentTime = startTime;
    unsigned long lastBlinkTime = startTime; // Track time for blinking
    bool blinkState = true;                  // Toggle for blinking text

    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(&arial10pt7b);

    // Draw each rectangle with interpolated color
    for (int i = 0; i < numRectangles; i++)
    {
        // Calculate the progress percentage
        int progress = (i * 100) / numRectangles;

        // Get the interpolated color for this progress
        uint16_t color = interpolateColor(progress);

        // Draw the rectangle with gradient color, offset by 1 pixel to avoid overlapping the left frame
        tft.fillRect(x + i * totalBarWidth + 1, y + 1, rectWidth - 1, height - 2, color);

        // Check if it's time to blink the text
        currentTime = millis();
        if (currentTime - lastBlinkTime >= 400)
        {
            lastBlinkTime = currentTime;
            blinkState = !blinkState; // Toggle blink state

            // Draw the text in the current blink state
            tft.setTextColor(blinkState ? TFT_LIGHTGREY : TFT_BLACK, TFT_BLACK);
            tft.drawString("Stepper Moving Capacitor To New Position", 240, y - 20);
        }

        // Wait for the segment's time duration
        while (millis() - startTime < duration / numRectangles)
        {
            // Keep the loop responsive
            yield();
        }
        startTime = millis();
    }

    // Calculate the total elapsed time
    unsigned long totalElapsedTime = millis() - overallStartTime;

    // Print the timing details to the console
    Serial.println();
    Serial.print("Duration argument: ");
    Serial.print(duration);
    Serial.println(" ms");

    Serial.print("Real duration: ");
    Serial.print(totalElapsedTime);
    Serial.println(" ms");

    Serial.print("Difference: ");
    Serial.print((long)totalElapsedTime - duration);
    Serial.println(" ms");

    // Clear the area for the progress bar
    tft.fillRect(0, 220, 480, 320 - 200, TFT_BLACK);
}
void displayLinearSWRMeter(float swr_value)
{
    static bool first_draw = true;                                                                 // Ensure initial setup happens only once
    static const int SEGMENT_WIDTH = 14;                                                           // Width of each segment
    static const int SEGMENT_GAP = 1;                                                              // Gap between segments
    static const int SEGMENT_HEIGHT = 40;                                                          // Height of each segment
    static const int METER_X = 20;                                                                 // X position of the top-left corner of the meter
    static const int TICK_LENGTH = 15;                                                             // Adjustable length of tick marks
    static const int NUM_SEGMENTS = 25;                                                            // Number of segments in the SWR meter
    static const int METER_Y = 320 - SEGMENT_HEIGHT - TICK_LENGTH - 38;                            // Y position dynamically calculated
    static const int FULL_WIDTH = NUM_SEGMENTS * SEGMENT_WIDTH + (NUM_SEGMENTS - 1) * SEGMENT_GAP; // Total width of the meter
    static float previous_swr_value = -1.0;                                                        // Store the previous SWR value for optimization
    static const uint16_t SWR_COLOR_BG = TFT_BLACK;
    tft.setTextDatum(TL_DATUM); // Set to top-left (default, equivalent to 0)

    // Helper function for gradient color interpolation
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

    // Initial static elements
    if (first_draw || redraw_swr)
    {
        tft.fillRect(0, METER_Y - 5, 480, 320 - METER_Y, SWR_COLOR_BG); // Clear area

        tft.drawRect(METER_X - 1, METER_Y - 1, FULL_WIDTH + 2, SEGMENT_HEIGHT + 2, TFT_WHITE);

        // Draw tick marks and labels
        float tick_values[] = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
        int tick_count = sizeof(tick_values) / sizeof(tick_values[0]);
        for (int i = 0; i < tick_count; i++)
        {
            int tick_position = METER_X + (int)((tick_values[i] - 1.0) * 10 * (SEGMENT_WIDTH + SEGMENT_GAP));
            tft.drawFastVLine(tick_position, METER_Y + SEGMENT_HEIGHT + 1, TICK_LENGTH, TFT_WHITE);
            tft.setTextColor(TFT_WHITE, SWR_COLOR_BG);
            tft.setFreeFont(&FreeMonoBold9pt7b);
            tft.drawString(String(tick_values[i], 1), tick_position - 17, METER_Y + SEGMENT_HEIGHT + TICK_LENGTH + 5);
        }
        first_draw = false;
        redraw_swr = false;
    }

    // Calculate active segments based on SWR value
    int active_segments = min((int)((swr_value - 1.0) * 10), NUM_SEGMENTS);

    // Gradient color points
    uint16_t color_start = TFT_GREEN; // Safe range
    uint16_t color_mid = TFT_YELLOW;  // Warning range
    uint16_t color_end = TFT_RED;     // Danger range

    // Draw the segments with gradient colors
    for (int i = 0; i < NUM_SEGMENTS; i++)
    {
        int x = METER_X + i * (SEGMENT_WIDTH + SEGMENT_GAP);

        // Determine color based on gradient
        float factor;
        uint16_t color;
        if (i < NUM_SEGMENTS / 2)
        { // Green to Yellow
            factor = (float)i / (NUM_SEGMENTS / 2);
            color = interpolateColor(color_start, color_mid, factor);
        }
        else
        { // Yellow to Red
            factor = (float)(i - NUM_SEGMENTS / 2) / (NUM_SEGMENTS / 2);
            color = interpolateColor(color_mid, color_end, factor);
        }

        // Draw or clear the segment
        if (i < active_segments)
        {
            tft.fillRect(x, METER_Y, SEGMENT_WIDTH, SEGMENT_HEIGHT, color);
        }
        else
        {
            tft.fillRect(x, METER_Y, SEGMENT_WIDTH, SEGMENT_HEIGHT, SWR_COLOR_BG);
        }
    }

    // Display the SWR value as dynamic text
    int value_x = METER_X + FULL_WIDTH + 10;
    int value_y = METER_Y + 8;

    // Erase the previous SWR value
    tft.setTextColor(SWR_COLOR_BG, SWR_COLOR_BG);
    tft.setFreeFont(&FreeMonoBold18pt7b);
    tft.drawString(String(previous_swr_value, 1), value_x, value_y);

    // Determine text color based on SWR value
    uint16_t swr_color = (swr_value < 2.0) ? TFT_GREEN : (swr_value < 3.0) ? TFT_YELLOW
                                                                           : TFT_RED;

    // Draw the new SWR value
    tft.setTextColor(swr_color, SWR_COLOR_BG);
    tft.drawString(String(swr_value, 1), value_x, value_y);

    // Update the previous SWR value
    previous_swr_value = swr_value;
}

void displayLogarithmicSWRMeter(float swr_value)
{
    static bool first_draw = true;
    static const int SWR_BAR_X = 20;       // X position of the bar
    static const int SWR_BAR_Y = 228;      // Y position of the bar
    static const int SWR_BAR_WIDTH = 380;  // Width of the bar
    static const int SWR_BAR_HEIGHT = 40;  // Height of the bar
    static const int SWR_TICK_HEIGHT = 10; // Height of tick marks
    static const int SEGMENT_WIDTH = 8;    // Width of the segments

    static const uint16_t SWR_COLOR_GOOD = TFT_GREENYELLOW;
    static const uint16_t COLOR_WARNING = TFT_ORANGE;
    static const uint16_t COLOR_DANGER = TFT_RED;
    static const uint16_t SWR_COLOR_BG = TFT_BLACK;
    static const uint16_t SWR_COLOR_TICK = TFT_WHITE;
    static const uint16_t SEGMENT_COLOR = TFT_WHITE;
    static const uint16_t SEGMENT_COLOR_BLACK = TFT_BLACK;

    static float previous_swr_value = -1.0;
    tft.setTextDatum(TL_DATUM); // Set to top-left (default, equivalent to 0)

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

    // Helper function to apply logarithmic scaling
    auto applyLogarithmicScale = [](float swr_value) -> float
    {
        if (swr_value > 4.0)
            swr_value = 4.0;
        if (swr_value < 1.0)
            swr_value = 1.0;
        float log_swr_min = log10(1.0);
        float log_swr_max = log10(4.0);
        return (log10(swr_value) - log_swr_min) / (log_swr_max - log_swr_min);
    };

    // Helper function to map SWR to bar length
    auto mapSWRToBarLength = [&](float swr_value) -> int
    {
        float log_swr_scaled = applyLogarithmicScale(swr_value);
        return log_swr_scaled * SWR_BAR_WIDTH;
    };

    if (first_draw || redraw_swr)
    {
        // erase area
        tft.fillRect(0, SWR_BAR_Y - 5, 480, 320 - SWR_BAR_Y + 5, SWR_COLOR_BG);

        float tick_values[] = {1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0};
        int num_ticks = sizeof(tick_values) / sizeof(tick_values[0]);

        for (int i = 0; i < num_ticks; i++)
        {
            float log_swr_scaled = applyLogarithmicScale(tick_values[i]);
            int tick_x = SWR_BAR_X + (log_swr_scaled * SWR_BAR_WIDTH);
            tft.drawLine(tick_x, SWR_BAR_Y + SWR_BAR_HEIGHT, tick_x, SWR_BAR_Y + SWR_BAR_HEIGHT + SWR_TICK_HEIGHT, SWR_COLOR_TICK);
            tft.setFreeFont(&FreeMonoBold9pt7b);
            tft.setTextColor(SWR_COLOR_TICK, SWR_COLOR_BG);
            tft.drawString(String(tick_values[i], 1), tick_x - 10, SWR_BAR_Y + SWR_BAR_HEIGHT + SWR_TICK_HEIGHT + 5);
        }

        first_draw = false;
        redraw_swr = false;
    }

    int current_length = min(mapSWRToBarLength(swr_value), SWR_BAR_WIDTH);
    int previous_length = min(mapSWRToBarLength(previous_swr_value), SWR_BAR_WIDTH);
    int green_length = mapSWRToBarLength(2.0);

    uint16_t color_start = SWR_COLOR_GOOD;
    uint16_t color_mid = COLOR_WARNING;
    uint16_t color_end = COLOR_DANGER;

    float position_factor = (swr_value - 1.0) / (4.0 - 1.0);
    uint16_t text_color = (position_factor <= 0.5)
                              ? interpolateColor(color_start, color_mid, position_factor * 2)
                              : interpolateColor(color_mid, color_end, (position_factor - 0.5) * 2);

    tft.setFreeFont(&FreeMonoBold18pt7b);
    tft.setTextColor(TFT_BLACK, TFT_BLACK);
    tft.drawString(String(previous_swr_value, 1), SWR_BAR_X + SWR_BAR_WIDTH + 10, SWR_BAR_Y + 8);

    if (swr_value > previous_swr_value)
    {
        for (int x = SWR_BAR_X; x < SWR_BAR_X + current_length; x += SEGMENT_WIDTH)
        {
            if (x >= SWR_BAR_X + SWR_BAR_WIDTH)
                break;

            float segment_factor = (float)(x - SWR_BAR_X) / SWR_BAR_WIDTH;
            uint16_t color = (segment_factor <= 0.5)
                                 ? interpolateColor(color_start, color_mid, segment_factor * 2)
                                 : interpolateColor(color_mid, color_end, (segment_factor - 0.5) * 2);
            tft.fillRect(x, SWR_BAR_Y, SEGMENT_WIDTH, SWR_BAR_HEIGHT, color);

            if (x <= SWR_BAR_X + green_length)
            {
                tft.drawLine(x, SWR_BAR_Y, x, SWR_BAR_Y + SWR_BAR_HEIGHT, SEGMENT_COLOR_BLACK);
            }
            else
            {
                tft.drawLine(x, SWR_BAR_Y, x, SWR_BAR_Y + SWR_BAR_HEIGHT, SEGMENT_COLOR);
            }
        }
    }
    else if (swr_value < previous_swr_value)
    {
        tft.fillRect(SWR_BAR_X + current_length, SWR_BAR_Y, previous_length - current_length, SWR_BAR_HEIGHT, SWR_COLOR_BG);

        for (int x = SWR_BAR_X + current_length + SEGMENT_WIDTH; x <= SWR_BAR_X + previous_length; x += SEGMENT_WIDTH)
        {
            if (x >= SWR_BAR_X + SWR_BAR_WIDTH)
                break;
            tft.fillRect(x, SWR_BAR_Y, SEGMENT_WIDTH, SWR_BAR_HEIGHT, SWR_COLOR_BG);
        }

        if (previous_length % SEGMENT_WIDTH != 0)
        {
            int last_segment_start = SWR_BAR_X + ((previous_length / SEGMENT_WIDTH) * SEGMENT_WIDTH);
            if (last_segment_start + SEGMENT_WIDTH > SWR_BAR_X + current_length)
            {
                tft.fillRect(last_segment_start, SWR_BAR_Y, SEGMENT_WIDTH, SWR_BAR_HEIGHT, SWR_COLOR_BG);
            }
        }
    }

    tft.drawRect(SWR_BAR_X, SWR_BAR_Y, SWR_BAR_WIDTH, SWR_BAR_HEIGHT, TFT_WHITE);

    tft.setTextColor(text_color, TFT_BLACK);
    tft.drawString(String(swr_value, 1), SWR_BAR_X + SWR_BAR_WIDTH + 10, SWR_BAR_Y + 8);

    previous_swr_value = swr_value;
}
/*
void displayOutofRangeMessage(int x, int y)
{
    static int blinkState = 0; // Static variable to retain state

    // Set the free font
    tft.setFreeFont(&FreeSansBold18pt7b);
    // Get the height of the currently selected font
    int textHeight = tft.fontHeight();

    // Erase the previous message
    tft.fillRoundRect(x - 65, y - textHeight - 5, 355, textHeight + 25, 5, TFT_BLACK);

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
    tft.setCursor(x - 12, y);
    tft.print("Out of Range !!!"); // Print some text using the selected font
    delay(150);
}
*/
//-------------------- SLAVE COMMAND FUNCTION ----------------------------------------------

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

void getStepperPositionForCurrentVFOfrequency(uint32_t currentVFOfrequency)
{

    static uint64_t prev_freq;

    if (currentVFOfrequency != prev_freq)
    {
        prev_freq = currentVFOfrequency;

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
                        displayStepperInfo(140, 260, currentStepperPosition, deltaSteps, false);
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
        }
    }
}

void TuneOnClick()
{
    if (deltaSteps == 0)
    {
        Serial.println("No Need To Tune; already on the right position");
        return;
    }
    // Check if the frequency is outside the defined ranges
    if (VFOfrequencyIsInAdmissibleRange(CurrentVFOFrequency) == false)
    {
        Serial.println("Frequency is outside the defined ranges.");
        return; // Return if outside both ranges
    }

    setNewPositionForCurrentVFOfrequency(CurrentVFOFrequency);
    // redraw mode
    Serial.print("I am here:");
    Serial.println(currentModeName);
    GetTunedStatusFromSlave(); // just to get the new stepper position
                               //  cheating a bit because GetTunedStatusFromSlave will return a slightly different value of couple of Hz

    displayRESONANCEfrequency(CurrentVFOFrequency, 121, 176, false);

    // deltaSteps = 0;
    displayStepperInfo(150, 260, currentStepperPosition, 0, true);
    displaySetMode(currentModeName, 52, 236, TFT_WHITE, TFT_LIGHTGREY, 2, false, true);
}
void setNewPositionForCurrentVFOfrequency(uint32_t targetFrequency)
{

    Serial.print("Sending command to Slave to move stepper to position for frequency: ");
    Serial.println(formatFrequencyForConsoleOutput(targetFrequency));

    String response = sendCommandToSlave("SetNewPositionForCurrentVFOfrequency", String(targetFrequency));

    // Parse the response to extract estimated duration
    int separatorIndex = response.indexOf(',');

    uint32_t targetStepperPosition = response.substring(0, separatorIndex).toInt();
    long estimated_movement_duration_in_microseconds;

    estimated_movement_duration_in_microseconds = response.substring(separatorIndex + 1).toInt();

    Serial.print("Received Target Position:");
    Serial.println(targetStepperPosition);

    Serial.print("Estimated movement duration: ");
    Serial.print(estimated_movement_duration_in_microseconds);
    Serial.println(" microseconds");
    Serial.println("Animating Progress Bar");
    displayProgressBar(20, 250, 435, 50, estimated_movement_duration_in_microseconds);
}
//--------------------------- HELPER FUNCTIONS ------------------------------------
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

String getModeCodeFromName(const String &modeName)
{
    // Mapping of mode names to mode codes
    const char *modeMapping[][2] = {
        {"LSB", "1"}, {"USB", "2"}, {"CW-U", "3"}, {"FM", "4"}, {"AM", "5"}, {"RTTY-L", "6"}, {"CW-L", "7"}, {"DATA-L", "8"}, {"RTTY-U", "9"}, {"DATA-FM", "A"}, {"FM-N", "B"}, {"DATA-U", "C"}, {"AM-N", "D"}, {"PSK", "E"}, {"DATA-FM-N", "F"}};

    for (size_t i = 0; i < sizeof(modeMapping) / sizeof(modeMapping[0]); i++)
    {
        if (modeName.equalsIgnoreCase(modeMapping[i][0])) // Case-insensitive comparison
        {
            return modeMapping[i][1]; // Return the corresponding mode code
        }
    }

    return ""; // Return an empty string if no match is found
}

void drawVariableCapacitor(int x, int y, float scale, uint16_t color)
{
    // Define line thickness for bold effect
    const int LINE_THICKNESS = 1; // Reduced thickness for a less bold appearance

    // Helper function to draw scaled and offset lines
    auto drawBoldLine = [&](int x0, int y0, int x1, int y1)
    {
        tft.drawLine(x0 * scale + x, y0 * scale + y,
                     x1 * scale + x, y1 * scale + y, color);
        for (int i = 1; i <= LINE_THICKNESS; i++)
        {
            tft.drawLine(x0 * scale + x, y0 * scale + y + i,
                         x1 * scale + x, y1 * scale + y + i, color);
            tft.drawLine(x0 * scale + x + i, y0 * scale + y,
                         x1 * scale + x + i, y1 * scale + y, color);
        }
    };

    // Helper function to draw scaled and offset triangles
    auto drawBoldTriangle = [&](int x0, int y0, int x1, int y1, int x2, int y2)
    {
        for (int i = 0; i < LINE_THICKNESS; i++)
        {
            tft.fillTriangle(x0 * scale + x + i, y0 * scale + y + i,
                             x1 * scale + x + i, y1 * scale + y + i,
                             x2 * scale + x + i, y2 * scale + y + i, color);
        }
    };

    // Draw the paths
    drawBoldLine(16, 5, 16, 27);  // Vertical line (center)
    drawBoldLine(19, 27, 19, 5);  // Vertical line (right)
    drawBoldLine(0, 16, 16, 16);  // Horizontal line (left)
    drawBoldLine(19, 16, 35, 16); // Horizontal line (right)

    // Draw the curved line (approximation of the bezier path)
    drawBoldLine(7, 27, 28, 5); // Diagonal curved line

    // Draw the arrowhead
    drawBoldTriangle(24, 5, 29, 3, 28, 8); // Arrowhead
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

bool VFOfrequencyIsInAdmissibleRange(unsigned long frequency)
{
    // Define frequency ranges for 40m and 20m bands
    const unsigned long LOWER_40M = 7000000;  // Replace with actual lower bound of 40m band
    const unsigned long UPPER_40M = 7200000;  // Replace with actual upper bound of 40m band
    const unsigned long LOWER_20M = 14000000; // Replace with actual lower bound of 20m band
    const unsigned long UPPER_20M = 14350000; // Replace with actual upper bound of 20m band

    // Check if frequency is within 40m or 20m range
    if ((frequency >= LOWER_40M && frequency <= UPPER_40M) ||
        (frequency >= LOWER_20M && frequency <= UPPER_20M))
    {
        return true; // Frequency is in an admissible range
    }

    return false; // Frequency is outside of admissible ranges
}

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

//------------------------- END OF HELPER FUNCTIONS ------------------------------------

void setup()
{
    Serial.begin(115200); // Start Serial communication for debugging
    delay(500);
    Serial.println("HB9IIU MLA Controller Starting");
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

    // Initialize SPIFFS used here to store/retrieve calibration data
    if (!initializeSPIFFS())
    {
        Serial.println("SPIFFS initialization failed. Check your flash memory.");
        while (1)
            ; // Halt or handle error
    }

    // Initialize TFT screen
    initTFTscreen();
    // displayWelcomeScreen(1500, VERSION, RELEASE_DATE);
    displayWelcomeScreenSimple(2500, VERSION, RELEASE_DATE);
    // Check and apply TFT display calibration data
    checkAndApplyTFTCalibrationData(false);

    establishWIFIconnectionWithSlave();
    // Start Bluetooth
    printOnTFT("Searching for BLE Cat Server....", TFT_YELLOW, TFT_BLACK);
    BLEDevice::init("");
    if (!resetScan(5000)) // Timeout after 5000 ms
    {
        Serial.println("No server found. Rebooting ESP...");
        printOnTFT("BLE CAT Server Not Found", TFT_RED, TFT_BLACK);
        delay(2000);
        printOnTFT("", TFT_GREEN, TFT_BLACK);
        printOnTFT("        Rebooting !!!", TFT_YELLOW, TFT_BLACK);
        delay(1000);
        ESP.restart(); // Reboot the ESP
    }
    printOnTFT("Successful Connection to Server", TFT_GREEN, TFT_BLACK);
    printOnTFT("", TFT_GREEN, TFT_BLACK);
    printOnTFT("       WE CAN START !!!", TFT_GREEN, TFT_BLACK);

    // Clear display
    delay(2000);
    tft.fillScreen(TFT_BLACK);

    updateWiFiWidget(53, 45, 72, 50, WiFi.RSSI(), false);
    GetTunedStatusFromSlave(); // to get current stepper pos and theoretical resonance freq
}

void loop()
{
    if (doConnect)
    {
        if (connectToServer())
        {
            Serial.println("Connected to server successfully.");
        }
        else
        {
            Serial.println("Connection failed. Restarting scan...");
            resetScan(5000);
        }
        doConnect = false;
    }

    // Handle reconnection if flagged
    if (reconnect && !connected)
    {
        Serial.println("Attempting to reconnect to server...");
        resetScan(5000);
        reconnect = false; // Reset flag after starting scan
    }

    // Periodically update Wifi widget
    static unsigned long lastWiFiWidgetUpdate = 0;
    if (millis() - lastWiFiWidgetUpdate > 15000)
    {
        Serial.print("Updating Wifi widget: ");
        updateWiFiWidget(53, 45, 72, 50, WiFi.RSSI(), false);
        lastWiFiWidgetUpdate = millis();
    }

    displayVFOfrequency(CurrentVFOFrequency, 121, 68, false);
    displaySetMode(currentModeName, 52, 236, TFT_WHITE, TFT_LIGHTGREY, 2, false, false);
    displaySetRFpower(setRFPower, 52, 166, TFT_WHITE, TFT_LIGHTGREY, 2, false);
    displayRXTXstatus(52, 120, PTTstatus);
    displayRESONANCEfrequency(theoreticalResonanceFrequency, 121, 176, false);

    // we use this to control the requests to the slave to not trigger while rotating the VFO knob
    if (previousVFOFrequency != CurrentVFOFrequency)
    {
        lastVFOfrequencyChangeTime = millis();
        previousVFOFrequency = CurrentVFOFrequency;
    }
    if (VFOfrequencyIsInAdmissibleRange(CurrentVFOFrequency) == false)
    {
        displayOutOfRangeWarning(140, 260);
    }

    if ((millis() - lastVFOfrequencyChangeTime) >= 300 || allStepperInfotoBeRedrawn)
    {
        if (VFOfrequencyIsInAdmissibleRange(CurrentVFOFrequency))
        {
            getStepperPositionForCurrentVFOfrequency(CurrentVFOFrequency); // Reset to prevent repetitive triggering
        }
    }

    static unsigned long previousMillisForKeypadScan = 0;
    uint16_t t_x = 9999, t_y = 9999; // To store the touch coordinate

    if (millis() - previousMillisForKeypadScan > 100)
    {
        // Detect if there’s a valid touch
        if (tft.getTouch(&t_x, &t_y))
        {
            //  Check for specific touch regions
            /*
            Serial.print("Touch detected at x");
            Serial.print(t_x);
            Serial.print(" y");
            Serial.println(t_y);
            */
            // Bottom Section
            if (isTouchPointInRegion(t_x, t_y, 145, 213, 480, 320))
            {
                Serial.print("Touch detected at bottom third");
                TuneOnClick();
            }
            // Upper Section
            if (isTouchPointInRegion(t_x, t_y, 145, 0, 480, 213))
            {
                Serial.print("Touch detected at upper third");

                SWRtest = true;

                // SWRtest = true;
            }
            // WiFi Widget
            if (isTouchPointInRegion(t_x, t_y, 0, 0, 100, 100))
            { // Bottom third of the screen
                Serial.print("Touch detected on WifiIcon");
            }
        }
        previousMillisForKeypadScan = millis();
    }
    if (SWRtest)
    {
        unsigned long SWRtestStartTime = millis();
        uint16_t SWRtestDuration = 2000;
        int RFpowerNow = setRFPower;
        String ModeNameNow = currentModeName;

        setRFpower(5);
        delay(200);
        setMode("FM");
        delay(100);
        displaySetRFpower(setRFPower, 52, 166, TFT_WHITE, TFT_LIGHTGREY, 2, false);
        setTXon();
        while (millis() < SWRtestStartTime + SWRtestDuration)
        {
            displayLogarithmicSWRMeter(currentSWR);
            // Serial.println(currentSWR);
            delay(100);
        }
        setTXoff();
        Serial.print("Setting Mode back to: ");
        Serial.println(ModeNameNow);
        setMode(ModeNameNow);
        Serial.print("Setting RF Power back to: ");
        Serial.print(RFpowerNow);
        Serial.println(" W");
        setRFpower(RFpowerNow);
        SWRtest = false;
        redraw_swr = true;
        delay(200);
        tft.fillRect(0, 228 - 5, 480, 320 - 228 + 5, TFT_BLACK);
        allStepperInfotoBeRedrawn = true;
        displayStepperInfo(150, 260, currentStepperPosition, 0, true);
        displaySetMode(currentModeName, 52, 236, TFT_WHITE, TFT_LIGHTGREY, 2, false, true);
    }
}