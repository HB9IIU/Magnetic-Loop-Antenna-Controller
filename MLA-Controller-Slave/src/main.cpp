#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <SPIFFS.h>


#define MAX_LOOKUP_SIZE 10000 // 2 x 5000 data sets
uint32_t lookupTable[MAX_LOOKUP_SIZE * 2];
size_t tableSize = 0;

// Driver DRV8825 at the back
#define DRM542_PUL_Pin 23 // orange
#define DRM542_DIR_Pin 22 // yellow
#define DRM542_EN_Pin 21  // green

// WiFi credentials
const char *ssid = "HB9IIU-MLA";

// Global variables (Checked)
long currentStepperPosition = 0;
long targetStepperPosition = 0;
unsigned long estimated_duration = 0;

// Global constants (Checked)
const unsigned int MIN_ACCEL_STEPS = 10000;   // Minimum steps for acceleration and deceleration
const unsigned int FINAL_DELAY = 25;         // Minimum delay for max speed
const unsigned int INITIAL_DELAY = 200;      // Maximum delay for initial speed (slow)
const long BACKLASH_CORRECTION_STEPS =22000; // Example value, adjust as needed

/////////////////////////////////////////////////////////////////////

bool debugReceivedCommands;

bool backlashCorrection = false; // Corrected variable name
int DIR = 0;                     // Direction: +1 or -1
bool moving = false;
// bool positionReached = false;   // Flag to indicate if target position is reached
unsigned long lastStepTime = 0; // To track the time between steps
float dynamicDelay = 0;         // Current delay between steps
long accelerationSteps = 0, constantSpeedSteps = 0, decelerationSteps = 0;
long stepsCompleted = 0;     // Tracks the number of steps completed
uint32_t tunedfrequency = 0; // frequency for currentStepperPosition from lookup table
uint32_t target_frequency = 0;

// Create an AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create a Preferences object
Preferences preferences;

// Function prototypes
void listFilesInSPIFFS();
String formatNumberWithSeparator(long value);
uint32_t findStepperPositionFromFrequency(uint32_t frequency);
uint32_t findFrequencyFromStepperPosition(uint32_t stepperPosition);
void handleStepperMovement();
void printLookupTable(); // Function prototype for printing lookup table
char *formatVFO(uint64_t vfo);
bool loadLookupTable();
void moveStepper(unsigned long stepsToMove, int direction);
unsigned long estimateTotalMovementDuration(long targetPosition);
unsigned long estimateSinglelMovementDuration(long totalSteps);

void setup()
{
    Serial.begin(115200);
    Serial.println("\nHB9IIU MLA SLAVE Controller Started");

    // Display initial memory usage
    Serial.print("Initial Free Heap: ");
    Serial.println(formatNumberWithSeparator(ESP.getFreeHeap()));

    // For ESP32, you might also use:
    Serial.print("Max Allocatable Heap: ");
    Serial.println(formatNumberWithSeparator(ESP.getMaxAllocHeap()));

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("An error occurred while mounting SPIFFS");
        return;
    }
    else
    {
        Serial.println("SPIFFS mounted successfully");
        listFilesInSPIFFS();
    }

    // Display memory usage after SPIFFS mounting
    Serial.print("Free Heap after mounting SPIFFS: ");
    Serial.println(formatNumberWithSeparator(ESP.getFreeHeap()));

    // Load the lookup table
    if (!loadLookupTable())
    {
        Serial.println("Table could not be loaded");
    }
    ///printLookupTable();
    //   Display memory usage after loading the lookup table
    Serial.print("Free Heap after loading lookup table: ");
    Serial.println(formatNumberWithSeparator(ESP.getFreeHeap()));

    // Initialize preferences
    preferences.begin("HB9IIUmla", false); // 'HB9IIUmla' is the storage group name
    currentStepperPosition = preferences.getLong("currentPos", 0);

    // Display memory usage after loading preferences
    Serial.print("Free Heap after loading preferences: ");
    Serial.println(formatNumberWithSeparator(ESP.getFreeHeap()));

    Serial.print("Max Allocatable Heap: ");
    Serial.println(formatNumberWithSeparator(ESP.getMaxAllocHeap()));

    Serial.print("\nRetrieved current stepper position from non-volatile memory: ");
    Serial.print(formatNumberWithSeparator(currentStepperPosition));
    tunedfrequency = findFrequencyFromStepperPosition(currentStepperPosition);
    pinMode(DRM542_EN_Pin, OUTPUT);
    pinMode(DRM542_PUL_Pin, OUTPUT);
    pinMode(DRM542_DIR_Pin, OUTPUT);
    digitalWrite(DRM542_EN_Pin, LOW); // Enable the stepper driver
    Serial.println("\nSetting up WiFi AP...");
    WiFi.softAP(ssid);
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    // HTTP Server endpoints
    // Add this inside your setup function
    server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "OK"); });

    server.on("/getLookupTable", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  // Default values for start and count
                  int start = 0;
                  int count = 1000;

                  // Check if "start" parameter is provided
                  if (request->hasParam("start"))
                  {
                      start = request->getParam("start")->value().toInt();
                  }

                  // Check if "count" parameter is provided
                  if (request->hasParam("count"))
                  {
                      count = request->getParam("count")->value().toInt();
                  }

                  String response = "";
                  size_t end = min(start + count, (int)tableSize);

                  for (size_t i = start; i < end; i++)
                  {
                      uint32_t frequency = lookupTable[i * 2];
                      uint32_t stepperPosition = lookupTable[i * 2 + 1];
                      response += String(frequency) + "," + String(stepperPosition) + "\n";
                  }

                  request->send(200, "text/plain", response);
                  // Serial.print("Sent data from index ");
                  // Serial.print(start);
                  // Serial.print(" to ");
                  // Serial.println(end);
              });

    server.on("/uploadLookupTable", HTTP_POST, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Upload initiated"); }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
              {
        if (index == 0) {
            Serial.println("Starting file write");
            SPIFFS.remove("/lookupTable.txt"); // Remove the existing table
            File file = SPIFFS.open("/lookupTable.txt", FILE_WRITE);
            if (!file) {
                Serial.println("Failed to open file for writing");
                return;
            }
            file.close();
        }
        
        // Append data to the file
        File file = SPIFFS.open("/lookupTable.txt", FILE_APPEND);
        if (file) {
            file.write(data, len);  // Write received data
            file.close();
        } else {
            Serial.println("Failed to open file for appending");
        }

        if (index + len == total) {
            Serial.println("File upload completed");
            if (loadLookupTable()) {
                Serial.println("New lookup table loaded successfully");
            } else {
                Serial.println("Failed to load the new lookup table");
            }
        } });

    server.on("/setStepperPosition", HTTP_POST, [](AsyncWebServerRequest *request)
              {
        if (request->hasParam("position", true)) {
            String positionStr = request->getParam("position", true)->value();
            long newStepperPosition = positionStr.toInt();

            preferences.putLong("currentPos", newStepperPosition); // Store the new position in preferences
            currentStepperPosition = newStepperPosition;

            Serial.print("New stepper position set: ");
            Serial.println(formatNumberWithSeparator(currentStepperPosition));

            request->send(200, "text/plain", "Stepper position updated successfully");
        } else {
            request->send(400, "text/plain", "Error: 'position' parameter is missing");
        } });

    server.on("/getCurrentStepperPosition", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String response = String(currentStepperPosition);
        //Serial.print("Current stepper position requested. Returning: ");
        //Serial.println(response);
        request->send(200, "text/plain", response); });

    server.on("/command", HTTP_POST, [](AsyncWebServerRequest *request)
              {
    if (request->hasParam("command", true))
    {
        String command = request->getParam("command", true)->value();
        if (debugReceivedCommands)
        {
            Serial.print("\nCommand received: ");
            Serial.println(command);
        }

        if (command == "GetTunedStatusFromSlave")
        {
                tunedfrequency = findFrequencyFromStepperPosition(currentStepperPosition);

            String response = String(currentStepperPosition) + "," + String(tunedfrequency);
            Serial.print("Returning response (current stepper position and predicted frequency): ");
            Serial.println(response);
            request->send(200, "text/plain", response);
        }
        else if (command == "moveBySteps")
        {
            String argument = request->getParam("argument", true)->value();
            long numberOfStep = argument.toInt();

            Serial.print("(1) Get request to move by ");
            Serial.println(formatNumberWithSeparator(numberOfStep));

            Serial.print("(2) Current Stepper Position: ");
            Serial.println(formatNumberWithSeparator(currentStepperPosition));

            targetStepperPosition = currentStepperPosition + numberOfStep;
            Serial.print("Final target Stepper Position:");
            Serial.println(formatNumberWithSeparator(targetStepperPosition));

            estimated_duration = estimateTotalMovementDuration(targetStepperPosition);

            // Reply to MASTER
            String response = String(targetStepperPosition) + "," + String(estimated_duration);
            Serial.print("Returning response (targetStepperPosition,estimated_duration): ");
            Serial.println(response);
            request->send(200, "text/plain", response);
            moving = true;

            if (estimated_duration == 0)
            {
                return;
            }

            moving = true;
        }
        else if (command == "setTunedFrequToPreferenceOnSlave")
        {
            String argument = request->getParam("argument", true)->value();
            uint32_t lookup_frequency = argument.toInt();
            Serial.print("\n(1) Got request to find & store stepper position for frequency ");
            Serial.println(formatVFO(lookup_frequency));
            Serial.print("(2) ");
            uint32_t PositionForGivenFrequency = findStepperPositionFromFrequency(lookup_frequency);
            preferences.putLong("currentPos", PositionForGivenFrequency); // Store the new position in preferences
            Serial.println("\n(4) Saved value to non-volatile memory");
            tunedfrequency = lookup_frequency;
            currentStepperPosition = PositionForGivenFrequency;
            String response = String(PositionForGivenFrequency) + "," + String(tunedfrequency);
            Serial.print("(5) Returning response (PositionForGivenFrequency,tunedfrequency): ");
            Serial.println(response);
            request->send(200, "text/plain", response);
        }
        else if (command == "getStepperPositionForCurrentVFOfrequency")
        {
            String argument = request->getParam("argument", true)->value();
            uint32_t lookup_frequency = argument.toInt();
            Serial.print("\n(1) Get request to return stepper position for frequency ");
            Serial.println(formatVFO(lookup_frequency));
            Serial.print("(2) ");
            uint32_t PositionForGivenFrequency = findStepperPositionFromFrequency(lookup_frequency);
            String response = String(PositionForGivenFrequency) + "," + String(currentStepperPosition);
            Serial.print("\n(4) Returning response (predicted & current positions): ");
            Serial.println(response);
            request->send(200, "text/plain", response);
        }
        else if (command == "SetNewPositionForCurrentVFOfrequency")
        {
            String argument = request->getParam("argument", true)->value();
            target_frequency = argument.toInt();
            Serial.print("(1) ");
            Serial.print("SetNewPosition argument received: ");
            Serial.println(formatVFO(target_frequency));
            Serial.print("(2) ");
            targetStepperPosition = findStepperPositionFromFrequency(target_frequency);
            Serial.print("\n(4) Stepper current position: ");
            Serial.println(formatNumberWithSeparator(currentStepperPosition));
            //stepsToGo = (targetStepperPosition > currentStepperPosition) ? (targetStepperPosition - currentStepperPosition) : (currentStepperPosition - targetStepperPosition);
            estimated_duration = estimateTotalMovementDuration(targetStepperPosition);
            Serial.print("(5) Estimated duration: ");
            Serial.print(estimated_duration);
            Serial.println(" ms");
            // replying to MASTER
            String response = String(targetStepperPosition) + "," + String(estimated_duration);
            Serial.print("Returning response (targetStepperPosition,estimated_duration): ");
            Serial.println(response);
            request->send(200, "text/plain", response);
            // if estimated duration = 0 then nothing else to be done
            if (estimated_duration == 0)
            {
                return;
            }

               moving = true;
        }
    } });

    server.begin();
    Serial.println("HTTP server started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
}

void loop()
{
    /* uncomment to test stepper motor

    // Delay for 2 seconds
    delay(2000);

    // Generate a random target position between 50,000 and 150,000
    targetStepperPosition = random(50000, 150000);
    estimated_duration = estimateTotalMovementDuration(targetStepperPosition);
    // Set moving flag to true
    moving = true;
*/
    // Handle the stepper movement
    handleStepperMovement();
}

void handleStepperMovement()
{
    if (!moving)
        return;
    Serial.print("\nSTEPPER MOVEMENT: ");
    Serial.print("New Target Position: ");
    //digitalWrite(DRM542_EN_Pin, LOW); // Enable the stepper driver

    Serial.println(formatNumberWithSeparator(targetStepperPosition));

    estimated_duration = estimateTotalMovementDuration(targetStepperPosition);

    int direction = (targetStepperPosition > currentStepperPosition) ? 1 : -1;
    unsigned long stepsToGo = abs(targetStepperPosition - currentStepperPosition);

    // Record start time
    unsigned long startTime = millis();

    // Backlash compensation
    if (direction == -1)
    {
        // Move backward
        unsigned long preTargetSteps = BACKLASH_CORRECTION_STEPS + stepsToGo;
        Serial.println("Initiating 'backlash correction movement'");
        Serial.print("Total number of steps to target: ");
        Serial.println(formatNumberWithSeparator(stepsToGo));
        Serial.print("Moving backwards by: ");
        Serial.print(formatNumberWithSeparator(preTargetSteps));
        Serial.println(" steps");
        moveStepper(preTargetSteps, direction);
        Serial.print("Moving forward by ");
        Serial.print(formatNumberWithSeparator(BACKLASH_CORRECTION_STEPS));
        Serial.println(" steps");
        direction = 1;
        moveStepper(BACKLASH_CORRECTION_STEPS, direction);
    }
    else
    {
        // Move directly to the target position
        Serial.print("\nInitiating 'normal' movement");
        moveStepper(stepsToGo, direction);
        Serial.println("");
    }

    // Record end time
    unsigned long endTime = millis();
    unsigned long elapsedTime = endTime - startTime;

    // Movement completed, update state
    moving = false;
    //digitalWrite(DRM542_EN_Pin, HIGH); // Disable the stepper driver

    preferences.putLong("currentPos", currentStepperPosition); // Save the new position to non-volatile memory
    Serial.print("Target position reached. New Position saved to volatile memory: ");

    Serial.println(formatNumberWithSeparator(currentStepperPosition));
    Serial.print("Estimated Duration: ");
    Serial.print(formatNumberWithSeparator(estimated_duration));
    Serial.println(" ms");

    // Print elapsed time
    Serial.print("Movement duration: ");
    Serial.print(formatNumberWithSeparator(elapsedTime));
    Serial.println(" ms");
    long deltaT = elapsedTime - estimated_duration;
    float percentDelta = (float)deltaT / estimated_duration * 100; // Use float for accurate division
    Serial.print("Diff. in %: ");
    Serial.print(percentDelta, 2); // Print with 2 decimal places
    Serial.println("%");
    Serial.println("");
}

void moveStepper(unsigned long stepsToMove, int direction)
{
    // Set the direction pin before moving
    digitalWrite(DRM542_DIR_Pin, (direction == 1) ? HIGH : LOW); // Set direction pin

    unsigned long accelerationSteps = (stepsToMove < 2 * MIN_ACCEL_STEPS) ? stepsToMove / 2 : MIN_ACCEL_STEPS;
    unsigned long constantSpeedSteps = (stepsToMove < 2 * MIN_ACCEL_STEPS) ? 0 : (stepsToMove - 2 * MIN_ACCEL_STEPS);
    unsigned long decelerationSteps = accelerationSteps;
    float dynamicDelay;

    // Acceleration phase
    for (unsigned long i = 0; i < accelerationSteps && stepsToMove > 0; i++)
    {
        dynamicDelay = INITIAL_DELAY - ((INITIAL_DELAY - FINAL_DELAY) * ((float)i / (float)accelerationSteps));
        if (dynamicDelay < 0)
            dynamicDelay = 0; // Ensure delay is non-negative
        digitalWrite(DRM542_PUL_Pin, HIGH);
        delayMicroseconds((unsigned int)dynamicDelay);
        digitalWrite(DRM542_PUL_Pin, LOW);
        delayMicroseconds((unsigned int)dynamicDelay);
        currentStepperPosition += direction;
        stepsToMove--;
        stepsCompleted++;
    }

    // Constant speed phase
    for (unsigned long i = 0; i < constantSpeedSteps && stepsToMove > 0; i++)
    {
        digitalWrite(DRM542_PUL_Pin, HIGH);
        delayMicroseconds((unsigned int)FINAL_DELAY);
        digitalWrite(DRM542_PUL_Pin, LOW);
        delayMicroseconds((unsigned int)FINAL_DELAY);
        currentStepperPosition += direction;
        stepsToMove--;
        stepsCompleted++;
    }

    // Deceleration phase
    for (unsigned long i = 0; i < decelerationSteps && stepsToMove > 0; i++)
    {
        dynamicDelay = FINAL_DELAY + ((INITIAL_DELAY - FINAL_DELAY) * ((float)i / (float)decelerationSteps));
        if (dynamicDelay < 0)
            dynamicDelay = 0; // Ensure delay is non-negative
        digitalWrite(DRM542_PUL_Pin, HIGH);
        delayMicroseconds((unsigned int)dynamicDelay);
        digitalWrite(DRM542_PUL_Pin, LOW);
        delayMicroseconds((unsigned int)dynamicDelay);
        currentStepperPosition += direction;
        stepsToMove--;
        stepsCompleted++;
    }
}

unsigned long estimateTotalMovementDuration(long targetPosition)
{
    // Calculate total steps including backlash compensation if moving backward
    long stepsToGo = abs(targetPosition - currentStepperPosition);
    if (stepsToGo==0){
        return 0;
 }
    if (targetPosition > currentStepperPosition)
    {
        // Add 5% margin to the estimated duration by multiplying by 1.05
        return (unsigned long)(estimateSinglelMovementDuration(stepsToGo) * 1.05);
    }
    else
    {
        long totaDuration = estimateSinglelMovementDuration(stepsToGo + BACKLASH_CORRECTION_STEPS);
        totaDuration = totaDuration + estimateSinglelMovementDuration(BACKLASH_CORRECTION_STEPS);
        // Add 5% margin to the estimated duration by multiplying by 1.05
        return (unsigned long)(totaDuration * 1.05);
    }
}

unsigned long estimateSinglelMovementDuration(long totalSteps)
{
    // Calculate acceleration and deceleration steps
    unsigned long accelerationSteps = (totalSteps < 2 * MIN_ACCEL_STEPS) ? totalSteps / 2 : MIN_ACCEL_STEPS;
    unsigned long constantSpeedSteps = (totalSteps < 2 * MIN_ACCEL_STEPS) ? 0 : (totalSteps - 2 * MIN_ACCEL_STEPS);
    unsigned long decelerationSteps = accelerationSteps;

    float dynamicDelay;
    float accelerationDuration = 0;
    float constantSpeedDuration = 0;
    float decelerationDuration = 0;

    // Acceleration phase duration
    for (unsigned long i = 0; i < accelerationSteps; i++)
    {
        dynamicDelay = INITIAL_DELAY - ((INITIAL_DELAY - FINAL_DELAY) * ((float)i / (float)accelerationSteps));
        if (dynamicDelay < 0)
            dynamicDelay = 0;
        accelerationDuration += 2 * dynamicDelay; // HIGH and LOW states
    }

    // Constant speed phase duration
    constantSpeedDuration = constantSpeedSteps * 2 * FINAL_DELAY; // HIGH and LOW states

    // Deceleration phase duration
    for (unsigned long i = 0; i < decelerationSteps; i++)
    {
        dynamicDelay = FINAL_DELAY + ((INITIAL_DELAY - FINAL_DELAY) * ((float)i / (float)decelerationSteps));
        if (dynamicDelay < 0)
            dynamicDelay = 0;
        decelerationDuration += 2 * dynamicDelay; // HIGH and LOW states
    }

    // Return total duration
    return (unsigned long)(accelerationDuration + constantSpeedDuration + decelerationDuration) / 1000;
}

String formatNumberWithSeparator(long value)
{
    String result = String(value);
    String formattedResult = "";
    int len = result.length();

    int startIdx = 0;
    if (value < 0)
    {
        formattedResult += '-';
        startIdx = 1;
    }

    int numLength = len - startIdx;
    int insertPosition = numLength % 3;
    for (int i = startIdx; i < len; i++)
    {
        formattedResult += result[i];
        if ((i - startIdx - insertPosition + 1) % 3 == 0 && i != len - 1)
        {
            formattedResult += "'";
        }
    }
    return formattedResult;
}
uint32_t findStepperPositionFromFrequency(uint32_t frequency)
{
    Serial.print("Searching stepper position for frequency: ");
    Serial.println(formatVFO(frequency));

    // Check if the frequency is out of bounds
    if (frequency < lookupTable[0] || frequency > lookupTable[2 * (tableSize - 1)])
    {
        Serial.println("Frequency out of bounds");
        return 0;
    }

    for (size_t i = 0; i < tableSize - 1; ++i)
    {
        uint32_t freq1 = lookupTable[2 * i];
        uint32_t position1 = lookupTable[2 * i + 1];
        uint32_t freq2 = lookupTable[2 * (i + 1)];
        uint32_t position2 = lookupTable[2 * (i + 1) + 1];

        if (frequency == freq1)
        {
            Serial.print("Exact match found: ");
            Serial.print(formatNumberWithSeparator(position1));
            return position1;
        }
        else if (frequency == freq2)
        {
            Serial.print("Exact match found: ");
            Serial.print(formatNumberWithSeparator(position2));
            return position2;
        }
        else if (frequency > freq1 && frequency < freq2)
        {
            // Perform linear interpolation
            float ratio = (float)(frequency - freq1) / (freq2 - freq1);
            uint32_t interpolatedPosition = (uint32_t)(position1 + ratio * (position2 - position1));

            // Ensure the interpolated position is within bounds
            if (interpolatedPosition < min(position1, position2) || interpolatedPosition > max(position1, position2))
            {
                Serial.println("Interpolated position out of bounds, adjusting to nearest bound.");
                interpolatedPosition = min(max(interpolatedPosition, min(position1, position2)), max(position1, position2));
            }

            Serial.print("Interpolated position: ");
            Serial.print(formatNumberWithSeparator(interpolatedPosition));
            return interpolatedPosition;
        }
    }

    Serial.println("No suitable range found");
    return 0;
}

uint32_t findFrequencyFromStepperPosition(uint32_t stepperPosition)
{
    Serial.print("\nSearching frequency for stepper position from lookup table: ");
    Serial.println(formatNumberWithSeparator(stepperPosition));

    uint32_t minPosition = lookupTable[1];
    uint32_t maxPosition = lookupTable[2 * (tableSize - 1) + 1];

    if (stepperPosition < minPosition || stepperPosition > maxPosition)
    {
        Serial.println("Stepper position out of bounds");
        return 0;
    }

    for (size_t i = 0; i < tableSize - 1; ++i)
    {
        uint32_t freq1 = lookupTable[2 * i];
        uint32_t position1 = lookupTable[2 * i + 1];
        uint32_t freq2 = lookupTable[2 * (i + 1)];
        uint32_t position2 = lookupTable[2 * (i + 1) + 1];

        if (stepperPosition == position1)
        {
            Serial.print("Exact match found: ");
            Serial.println(formatVFO(freq1));
            return freq1;
        }
        else if (stepperPosition == position2)
        {
            Serial.print("Exact match found: ");
            Serial.println(formatVFO(freq2));
            return freq2;
        }
        else if (stepperPosition > position1 && stepperPosition < position2)
        {
            float ratio = (float)(stepperPosition - position1) / (position2 - position1);
            uint32_t interpolatedFrequency = freq1 + (uint32_t)(ratio * (freq2 - freq1));
            Serial.print("Interpolated frequency: ");
            Serial.println(formatVFO(interpolatedFrequency));
            return interpolatedFrequency;
        }
    }
    Serial.println("No suitable range found");
    return 0;
}

char *formatVFO(uint64_t vfo)
{
    static char vfo_str[20] = {""};
    uint32_t MHz = (vfo / 1000000 % 1000000);
    uint16_t Hz = (vfo % 1000);
    uint16_t KHz = ((vfo % 1000000) - Hz) / 1000;
    sprintf(vfo_str, "%lu.%03u.%03u", MHz, KHz, Hz);
    return vfo_str;
}

bool loadLookupTable()
{
    File file = SPIFFS.open("/lookupTable.txt", "r");
    if (!file)
    {
        Serial.println("Failed to open lookup table file");
        return false;
    }

    size_t index = 0;
    while (file.available() && index < MAX_LOOKUP_SIZE * 2)
    {
        String line = file.readStringUntil('\n');
        line.trim();
        int separatorIndex = line.indexOf(',');
        if (separatorIndex > 0)
        {
            String freqString = line.substring(0, separatorIndex);
            String stepperString = line.substring(separatorIndex + 1);
            uint32_t frequency = freqString.toInt();
            uint32_t stepperPosition = stepperString.toInt();
            lookupTable[index++] = frequency;
            lookupTable[index++] = stepperPosition;
        }
        else
        {
            Serial.println("No valid separator found in line");
        }
    }

    tableSize = index / 2; // Number of entries
        

    file.close();

    Serial.println("Lookup table loaded successfully");
    return true;
}

void listFilesInSPIFFS()
{
    Serial.println("Listing files in SPIFFS:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();

    while (file)
    {
        Serial.print("FILE: ");
        Serial.println(file.name());
        file = root.openNextFile();
    }
}

void printLookupTable()
{
    Serial.println("Printing Lookup Table:");
    if (tableSize == 0)
    {
        Serial.println("Lookup table is empty.");
        return;
    }
    for (size_t i = 0; i < tableSize; i++)
    {
        uint32_t frequency = lookupTable[i * 2];
        uint32_t stepperPosition = lookupTable[i * 2 + 1];
        Serial.print(i);
        Serial.print(" Frequency: ");
        Serial.print(frequency);
        Serial.print(", Stepper Position: ");
        Serial.println(stepperPosition);
    }
    Serial.println("End of Lookup Table.");
}
