/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define FTMS_UUID "00001826-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"
#define LED_BUILTIN 2

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
    setupBluetoothServer();
    setupHalSensor();
}

BLECharacteristic *pCharacteristic;
void setupBluetoothServer()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    BLEDevice::init("IC Bike");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(FTMS_UUID);
    pCharacteristic = pService->createCharacteristic(
        INDOOR_BIKE_DATA_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setValue("Characteristic configured"); // Used for demonstration purposes.
    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // add this for backwards compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FTMS_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

int analogPin = 18;
int digitalPin = 19;

//NOTE: ISR is triggered too frequently to be relied upon, even with RISING. RPMs are in the thousands.
// This variable is used for syncronisation
// We use it to ensure that the ISR and the loop
// do not try to access the 'rev' variable
// at the same time (Causes a Guru Meditation Error)
//portMUX_TYPE revMux = portMUX_INITIALIZER_UNLOCKED;
//
//int rev = 0;
//void IRAM_ATTR ISR() //interrupt service routine
//{
//    portENTER_CRITICAL_ISR(&revMux);
//    rev++;
//    Serial.printf("rev: %d \n", rev);
//    portEXIT_CRITICAL_ISR(&revMux);
//}

void setupHalSensor()
{
    //    attachInterrupt(digitalPin, ISR, RISING); //attaching the interrupt
    pinMode(analogPin, INPUT);
    pinMode(digitalPin, INPUT);
    Serial.begin(9600);
}

bool passedMagnet = true;
int rev = 0;
void incrementRevolutions(bool passingMagnet)
{
    // Serial.printf("passedMagnet: %d ,  passingMagnet %d , rev: %d \n", passedMagnet, passingMagnet, rev);
    if (passedMagnet && passingMagnet)
    { //Started a new pass of the magnet
        passedMagnet = false;
        rev++;
    }
    else if (!passedMagnet && !passingMagnet)
    { //The new pass of the magnet is complete
        passedMagnet = true;
    }
}

unsigned long oldTime = 0;
unsigned long revTime = 0;
int oldRpm = 0;
int calculateRpmFromRevolutions()
{
    revTime = millis() - oldTime;
    if (revTime > 1000) // One second
    {
        oldRpm = rev * 60000 / revTime; //calculates rpm
        Serial.printf("RPM: %d, rev %d , revTime: %d , oldTime: %d \n", rev, oldRpm, revTime, oldTime);
        oldTime = millis(); //saves the current time (in relation to program start)
        rev = 0;
    }
    return oldRpm;
}

void indicateRpmWithLight(int rpm)
{
    if (rpm > 1)
    {
        digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    }
    else
    {
        digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
    }
}

void transmitRpmOverBle(int rpm)
{
    pCharacteristic->setValue(rpm);
}

void loop()
{
    incrementRevolutions(digitalRead(digitalPin));
    int rpm = calculateRpmFromRevolutions();
    indicateRpmWithLight(rpm);
    transmitRpmOverBle(rpm);
}
