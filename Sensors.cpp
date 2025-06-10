#include <array>
#include <string>
#include <string.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BH1750.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "src/MotionSensor.h"
#include "src/LightSensor.h"
#include "src/UltrasonicSensor.h"
#include "src/VibrationSensor.h"
#include "src/SoundSensor.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

unsigned long previousMillis = 0;
const long interval = 1000; // Interval for logging data (1 second)

// These are the handles to the separate threads. Each individual thread will run one of the sensors
TaskHandle_t vibrationThread;
TaskHandle_t motionThread;
TaskHandle_t proximityThread;
TaskHandle_t lightThread;
TaskHandle_t DataloggingThread;

////////////////////////////////////////////////////////////////////////////////////////
// Bluetooth Parameters, macros for sensor delay from active to nonactive
////////////////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SUBSYSTEM_CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define DESCRIPTOR_UUID "00002902-0000-1000-8000-00805f9b34fb"
#define SUBSYSTEM_DESCRIPTOR "3f0d768b-ddfb-4b46-acdb-e10b904ed065"

#define MOTION_BUFFER_SIZE 500
#define MOTION_BUFFER_CUTOFF 480

// Emergency alert time in minutes
// Every n minutes, if there has been no movement detected, an emergency alert will be sent
// Ex. If EMERGENCY_ALERT_TIME == 2, then every 2 minutes, if motion hasn't been detected, an alert notification will be sent to the Android device
#define EMERGENCY_ALERT_TIME 2

BluetoothSerial bluetooth;
BLECharacteristic *characteristic; // This is a handle to the main system's main data BLE characteristic that the Android app will retrieve it's data from

// This is a handle to the BLE characteristic that the subsystem will write the room's occupation status to
// The main system will use this characteristic in combination with time since last detection to determine if an emergency alert should be sent
BLECharacteristic *subsystemCharacteristic;

// Pointer to BLEAdvertising object that will be used to start and stop the advertising of the main system's BLE service and characteristics as needed
BLEAdvertising *advertising;

// Tracks the number of devices connected to this main system (via BLE)
// By default, there should at least be one BLE connection at all times (the subsystem) and the main system should not be advertising
// When an emergency alert needs to be sent, the main system will start advertising, and the Android device should pick up the main system on it's scan and connect
int connectedDevices = 0;

////////////////////////////////////////////////////////////////////////////////////////
// Pin setup for sensors
////////////////////////////////////////////////////////////////////////////////////////
const int motionSensorInputPin = 14;
const int trigPin = 2;  // ultrasonic sensor trigger pin
const int echoPin = 15; // ultrasonic sensor echo pin
const int vibrationInputPin = 36;

// Right now, the sound sensor isn't used in the main system
const int soundInputPin = 26;
const int soundGatePin = 27;

// Implementing class for BLEServerCallbacks
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server) override;
    void onDisconnect(BLEServer *server) override;
};

// After the subsystem successfully establishes a connection, the main system will stop advertising to prevent any other connections
// If for some reason the subsystem is disconnected, begin advertising again to allow it to reconnect
void MyServerCallbacks::onConnect(BLEServer *server)
{
    connectedDevices++;
    if (advertising != nullptr)
    {
        Serial.println("A device connected");
        if (connectedDevices >= 1)
        {
            Serial.println("All devices connected. Stopping BLE advertise...");
            server->getAdvertising()->stop();
        }
        else
        {
            server->getAdvertising()->start();
        }
        Serial.print("Connected devices: ");
        Serial.println(connectedDevices);
    }
}

// On subsystem disconnect, if there are no connected devices, begin advertising
void MyServerCallbacks::onDisconnect(BLEServer *server)
{
    connectedDevices--;
    Serial.println("Device disconnected");
    Serial.print("Connected devices: ");
    Serial.println(connectedDevices);
    if (connectedDevices < 1)
    {
        server->getAdvertising()->start();
    }
}

////////////////////////////////////////////////////////////////////////////////////////
// calibration time (in seconds) for the sensors to be calibrated
////////////////////////////////////////////////////////////////////////////////////////
const int calibrationTime = 30;

////////////////////////////////////////////////////////////////////////////////////////
// Dynamically allocated MotionSensor object which will be used for interacting with the motion sensor
////////////////////////////////////////////////////////////////////////////////////////
MotionSensor *motionSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic sensor objects for interacting with the ultrasonic sensors
////////////////////////////////////////////////////////////////////////////////////////
UltrasonicSensor *ultrasonicSensor;
UltrasonicSensor *secondUltrasonicSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Vibration sensor object for interacting with the vibration sensor
////////////////////////////////////////////////////////////////////////////////////////
VibrationSensor *vibrationSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Light sensor object
////////////////////////////////////////////////////////////////////////////////////////
LightSensor *lightSensor;

////////////////////////////////////////////////////////////////////////////////////////
// Sound sensor object
////////////////////////////////////////////////////////////////////////////////////////
SoundSensor *soundSensor;

////////////////////////////////////////////////////////////////////////////////////////
// String variable to keep track of previous detection state
////////////////////////////////////////////////////////////////////////////////////////
String previousMessage = "";

////////////////////////////////////////////////////////////////////////////////////////
// keep track of current time in ms when one of the sensors read HIGH
////////////////////////////////////////////////////////////////////////////////////////
unsigned int timeOfDetection = 0;
unsigned int lastDetected = 0; // time since last detection in seconds
unsigned int minutes = 0;      // minutes that have passed since last detection
unsigned int alertMinutes = 0; // this will determine when an alert will be sent (if it's equal to or greater than EMERGENCY_ALERT_TIME

////////////////////////////////////////////////////////////////////////////////////////
// JSON Object we will send to the Bluetooth app. It keeps track of the status as well as time since last detection
////////////////////////////////////////////////////////////////////////////////////////
StaticJsonDocument<300> jsonData;
String stringData;

////////////////////////////////////////////////////////////////////////////////////////
// Motion Buffer class to store last 500 sensor readings from the motion sensor (work in progress)
////////////////////////////////////////////////////////////////////////////////////////

enum class BufferResult
{
    NO_ALERT,
    SEND_ALERT
};

class MotionBuffer
{
private:
    int buffer[MOTION_BUFFER_SIZE];
    int current_index = 0;
    bool full = false;

public:
    void appendReading(int value);
    void clear();
    bool isFull() const;
    BufferResult processBuffer();
};

void MotionBuffer::appendReading(int value)
{
    buffer[current_index] = value;
    current_index++;
    if (current_index >= MOTION_BUFFER_SIZE)
    {
        full = true;
    }
}

void MotionBuffer::clear()
{
    for (int i = 0; i < MOTION_BUFFER_SIZE; i++)
    {
        buffer[i] = int();
    }
    current_index = 0;
}

bool MotionBuffer::isFull() const { return full; }

BufferResult MotionBuffer::processBuffer()
{
    int num_high_readings = 0;
    for (int i = 0; i < MOTION_BUFFER_SIZE; i++)
    {
        switch (buffer[i])
        {
        case 0:
        {
            break;
        }
        case 1:
        {
            num_high_readings++;
            break;
        }
        }
    }
    if (num_high_readings < MOTION_BUFFER_CUTOFF)
    {
        return BufferResult::SEND_ALERT;
    }
    else
    {
        return BufferResult::NO_ALERT;
    }
    clear();
}

MotionBuffer motion_buffer;

////////////////////////////////////////////////////////////////////////////////////////
// BLE Response Class for parsing the response from the subsystem characteristic and setting room occupied status
////////////////////////////////////////////////////////////////////////////////////////
class BLEResponse
{
private:
    std::string previousResponse = "";
    boolean isOccupied = false;

public:
    void parseResponse(std::string response)
    {
        if (response == std::string("Occupied"))
        {
            isOccupied = true;
        }
        else
        {
            isOccupied = false;
        }
        // When the room occupation status transitions, we want to reset the minutes counter
        if (response != previousResponse)
        {
            minutes = 0;
            previousResponse = response;
        }
    }
    bool getOccupied() { return isOccupied; }
};

BLEResponse parser;

////////////////////////////////////////////////////////////////////////////////////////
// TempAdvertise interface that can be implemented to determine what the behavior of starting BLE advertising should be
////////////////////////////////////////////////////////////////////////////////////////
class TempAdvertise
{
public:
    virtual void onTempAdvertising(unsigned long milliseconds) = 0;
};

// Class that implements TempAdvertise
class MyTempAdvertise : public TempAdvertise
{
public:
    void onTempAdvertising(unsigned long milliseconds) override;
};

// Start advertising, wait for the specified number of milliseconds, then stop advertising
void MyTempAdvertise::onTempAdvertising(unsigned long milliseconds)
{
    advertising->start();
    delay(milliseconds);
    advertising->stop();
}

void startTempAdvertising(TempAdvertise *tempAdvertise)
{
    tempAdvertise->onTempAdvertising(60000); // begin BLE advertising for a full minute
}

MyTempAdvertise *tempAdvertiseCallback = new MyTempAdvertise();

// Helper method that takes a String message to set the main system's characteristic's value
// We convert the message into an array of bytes before setting the value, which will be translated to proper characters by the ESP32 via UTF-8
void sendBluetoothMessage(String message, BLECharacteristic *characteristic)
{
    uint8_t data[message.length() + 1];
    memcpy(data, message.c_str(), message.length());
    characteristic->setValue(data, message.length());
    characteristic->notify();
}

// Tasks/functions for multithreading sensors
void lightTask(void *parameter)
{
    while (true)
    {
        lightSensor->start();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void soundTask(void *parameter)
{
    while (true)
    {
        soundSensor->start();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void vibrationTask(void *parameter)
{
    while (true)
    {
        vibrationSensor->start();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//////////////////////// SD Card Functions ////////////////////////

// List the files in the SD card
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.path(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

// Create a new directory
void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

// Remove a directory
void removeDir(fs::FS &fs, const char *path)
{
    Serial.printf("Removing Dir: %s\n", path);
    if (fs.rmdir(path))
    {
        Serial.println("Dir removed");
    }
    else
    {
        Serial.println("rmdir failed");
    }
}

// Read a file and print its contents
void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
    }
    file.close();
}

// Write to a file
void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

// Append to a file
void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

// Rename a file
void renameFile(fs::FS &fs, const char *path1, const char *path2)
{
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2))
    {
        Serial.println("File renamed");
    }
    else
    {
        Serial.println("Rename failed");
    }
}

// Delete a file
void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
        Serial.println("File deleted");
    }
    else
    {
        Serial.println("Delete failed");
    }
}
// Aggregation variables for decision tree 
int    aggregationCounter = 0;
double sumDistance       = 0.0;
double sumMotion         = 0.0;
double sumLightIntensity = 0.0;
int    sumOccupiedCount  = 0;     // ← NEW: count how many times occupied==true
bool   occupied_ml       = false; 

// Helper function to log the relevant sensor data to the SD card
void logData() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Get current time
        String time = String(currentMillis / 1000);

        // Retrieve raw sensor data
        bool motion       = motionSensor->getRawOutput();
        float distance    = ultrasonicSensor->distance();
        float lightIntensity = lightSensor->lightValue();
        bool occupied     = parser.getOccupied();

        // Accumulate data for aggregation
        sumDistance       += distance;
        sumMotion         += motion        ? 1.0 : 0.0; // Convert bool → 0.0/1.0
        sumLightIntensity += lightIntensity;
        sumOccupiedCount  += occupied      ? 1   : 0;   // NEW
        aggregationCounter++;

        // Every 5 seconds, run the decision tree model
        if (aggregationCounter >= 5) {
            // 1) Calculate averages
            double avgDistance = sumDistance       / 5.0;
            double avgMotion   = sumMotion         / 5.0;
            double avgLight    = sumLightIntensity / 5.0;

            // 2) Compute mode for occupied: majority vote (>=3 of 5 → 1)
            bool   modeOccBool   = (sumOccupiedCount >= 3);
            double modeOccDouble = modeOccBool ? 1.0 : 0.0;

            // 3) Prepare 4-feature input for the model
            double input[4]  = { avgDistance,
                                 avgMotion,
                                 avgLight,
                                 modeOccDouble };
            double output[2];
            score(input, output); // your existing tree function

            // 4) Determine occupancy prediction
            occupied_ml = (output[1] > 0.5);

            // 5) DEBUG: print exactly what the model saw
            Serial.printf("MODEL IN→ dist=%.2f mot=%.2f light=%.2f occMode=%d\n",
                          avgDistance, avgMotion, avgLight, modeOccBool);

            // Reset aggregation for next window
            sumDistance        = 0.0;
            sumMotion          = 0.0;
            sumLightIntensity  = 0.0;
            sumOccupiedCount   = 0;
            aggregationCounter = 0;
        }

        // Format the data string with ML prediction
        String dataString = time + " -> distance: "    + String(distance) + 
                            ", motion: "              + String(motion) +
                            ", lightIntensity: "      + String(lightIntensity) +
                            ", occupied: "            + String(occupied) +
                            ", occupied_ml: "         + String(occupied_ml) + "\n";

        // Append to log.txt (same file)
        appendFile(SD, "/log.txt", dataString.c_str());
    }
}

// data logging task
void DataloggingTask(void *parameter)
{
    while (true)
    {
        logData();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void score(double *input, double *output) {
    double var0[2];

    if (input[0] <= 106.15739059448242) {
        if (input[2] <= 5.582000017166138) {
            if (input[3] <= 0.5) {
                memcpy(var0, (double[]){0.915834312573443, 0.08416568742655699}, 2 * sizeof(double));
            } else {
                memcpy(var0, (double[]){0.9902862985685071, 0.009713701431492843}, 2 * sizeof(double));
            }
        } else {
            if (input[0] <= 85.9000015258789) {
                memcpy(var0, (double[]){0.8835705045278137, 0.11642949547218628}, 2 * sizeof(double));
            } else {
                memcpy(var0, (double[]){0.450920245398773, 0.549079754601227}, 2 * sizeof(double));
            }
        }
    } else {
        if (input[2] <= 5.746999979019165) {
            if (input[2] <= 1.334241509437561) {
                memcpy(var0, (double[]){0.30695683196575096, 0.693043168034249}, 2 * sizeof(double));
            } else {
                memcpy(var0, (double[]){0.9450998898851659, 0.05490011011483404}, 2 * sizeof(double));
            }
        } else {
            if (input[0] <= 275.3000030517578) {
                memcpy(var0, (double[]){0.06493669299235301, 0.935063307007647}, 2 * sizeof(double));
            } else {
                memcpy(var0, (double[]){0.17463235294117646, 0.8253676470588235}, 2 * sizeof(double));
            }
        }
    }

    memcpy(output, var0, 2 * sizeof(double));
}

void setup()
{
    Serial.begin(115200);

    // Initializing SD card
    SPI.begin();

    // output to serial monitor if SD card is connected or not
    if (!SD.begin())
    {
        Serial.println("Card failed, or not present");
        return;
    }
    Serial.println("card initialized.");

    uint8_t cardType = SD.cardType();

    // output SD card information
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC)
    {
        Serial.println("MMC");
    }
    else if (cardType == CARD_SD)
    {
        Serial.println("SDSC");
    }
    else if (cardType == CARD_SDHC)
    {
        Serial.println("SDHC");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    motionSensor = new MotionSensor(motionSensorInputPin);
    ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
    lightSensor = new LightSensor;
    vibrationSensor = new VibrationSensor(vibrationInputPin);

    lightSensor->setup();
    vibrationSensor->setup();
    light_module.begin();

    // Initializing I2C (the vibration and light sensor uses I2C)
    Wire.begin();

    Serial.println("Starting BLE setup...");

    // Initialize BLE for the main system with the name "ESP32"
    BLEDevice::init("ESP32");

    // Creating the BLEServer that will hold all of the necessary BLE services and characteristics
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new MyServerCallbacks());
    BLEService *service = server->createService(SERVICE_UUID);

    // Creating the main characteristic that will hold all of the sensor data
    characteristic = service->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

    // Creating the characteristic for the subsystem that will hold the current room's occupation status
    subsystemCharacteristic = service->createCharacteristic(SUBSYSTEM_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    subsystemCharacteristic->setValue("Occupied");

    characteristic->setValue("Test");
    BLEDescriptor descriptor(DESCRIPTOR_UUID);
    uint8_t descriptorValue[] = {0x00, 0x01};
    BLEDescriptor subsystemDescriptor(SUBSYSTEM_DESCRIPTOR);
    uint8_t subsystemDescriptorValue[] = {0x00, 0x01};
    descriptor.setValue(descriptorValue, 2);
    subsystemDescriptor.setValue(subsystemDescriptorValue, 2);
    subsystemCharacteristic->addDescriptor(&subsystemDescriptor);
    characteristic->addDescriptor(&descriptor);

    // Starting the BLE service that holds the two BLECharacteristics
    service->start();

    // Obtain a handle to BLEAdvertising here and start advertising
    advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE setup complete!\n");

    Serial.print("calibrating sensor ");
    for (int i = 0; i < calibrationTime; i++)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");

    // Start the data logging task on core 0
    xTaskCreatePinnedToCore(DataloggingTask, "DataloggingTask", 10000, NULL, 0, NULL, 0);

    // Start the light and vibration sensors on a separate thread on core 0
    xTaskCreatePinnedToCore(lightTask, "LightTask", 10000, NULL, 0, NULL, 0);
    // xTaskCreatePinnedToCore(soundTask, "SoundTask", 10000, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(vibrationTask, "VibrationTask", 10000, NULL, 0, NULL, 0);
}

void loop()
{

    // Motion and ultrasonic sensor are running on the main thread (on core 1)
    motionSensor->start();
    ultrasonicSensor->start();

    // Obtain the number of currently "active" sensors
    // Active means that the sensor has individually concluded that there is human presence/motion within the room
    int activeSensors = motionSensor->isActive() + ultrasonicSensor->isActive() + lightSensor->isActive();

    // Read the subsystem characteristic value every loop. The String value could potentially change after a write from the subsystem
    parser.parseResponse(subsystemCharacteristic->getValue().c_str());

    // Serial.print("Light Sensor Baseline: ");
    // Serial.println(lightSensor->baseline());

    // output if the room is occupied or not to the serial monitor
    if (parser.getOccupied())
    {
        Serial.println("Room is occupied");
    }
    else
    {
        Serial.println("Room is not occupied");
    }

    // Setting the key value pairs for the JSON that will be sent to the main sensor data BLE characteristic
    // Boolean Data
    jsonData["motion"] = motionSensor->isActive();
    jsonData["proximity"] = ultrasonicSensor->isActive();
    jsonData["light"] = lightSensor->isActive();
    jsonData["occupied"] = parser.getOccupied();

    // Numerical Data
    jsonData["lightIntensity"] = lightSensor->lightValue();
    jsonData["distance"] = ultrasonicSensor->distance();
    jsonData["vibrationIntensity"] = vibrationSensor->intensity();
    jsonData["vibrationBaseline"] = vibrationSensor->baseline();
    jsonData["proximityBaseline"] = ultrasonicSensor->baseline();
    jsonData["lightBaseline"] = lightSensor->baseline();
    jsonData["lightOffBaseline"] = lightSensor->baseline_off();

    jsonData["connected_devices"] = connectedDevices;

    if (lightSensor->lightValue() >= 120)
    {
        jsonData["lightingStatus"] = "Lights on";
    }
    else
    {
        jsonData["lightingStatus"] = "Lights off";
    }

    // log the sensor data to the SD card

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    // Two or more of the sensors are reading high (person is present)
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    if (activeSensors >= 2)
    {
        timeOfDetection = millis();
        minutes = 0;
        alertMinutes = 0;

        jsonData["movement_status"] = "Person is present";
        jsonData["lastDetected"] = 0;
        serializeJson(jsonData, stringData);
        sendBluetoothMessage(stringData, characteristic);
        previousMessage = "Person is present";

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // All sensors read low (person is absent or not being detected for an extended period of time)
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        lastDetected = ((millis() - timeOfDetection) / 1000); // keeps track of time since last detection in seconds
        jsonData["movement_status"] = "Not present";

        jsonData["lastDetected"] = minutes;
        serializeJson(jsonData, stringData);
        sendBluetoothMessage(stringData, characteristic);
        previousMessage = "Not present";

        // If person isn't detected, keep track of last detection in minutes
        if (lastDetected >= 60)
        {
            lastDetected = 0;
            timeOfDetection = millis();
            minutes++;
            alertMinutes++;
            jsonData["lastDetected"] = minutes;

            // If this statement evaluates to true, an emergency alert notification will be sent to the Android device
            if (minutes % EMERGENCY_ALERT_TIME == 0 && parser.getOccupied())
            {
                // Start advertising and send an alert to the client device
                alertMinutes = 0;
                startTempAdvertising(tempAdvertiseCallback);
            }

            serializeJson(jsonData, stringData);
            sendBluetoothMessage(stringData, characteristic);
        }
    }
    stringData = "";
}