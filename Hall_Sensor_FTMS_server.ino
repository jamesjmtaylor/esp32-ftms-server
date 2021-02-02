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

#define FTMS_UUID        "00001826-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"
#define LED_BUILTIN 2

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  setupBluetoothServer();
  setupHalSensor();  
}

BLECharacteristic *pCharacteristic;
void setupBluetoothServer() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  BLEDevice::init("IC Bike");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(FTMS_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         INDOOR_BIKE_DATA_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );



  pCharacteristic->setValue("Characteristic configured");  // Used for demonstration purposes.
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // add this for backwards compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(FTMS_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

int digitalPin = 34; // for ESP32 microcontroller
void setupHalSensor() {
  pinMode(digitalPin, INPUT); 
  Serial.begin(9600);
}

bool passingMagnet = false;
void loop() {
  int digitalVal = digitalRead(digitalPin);
  Serial.println(digitalVal);
  if (digitalVal == 1) {
    passingMagnet = true;
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    pCharacteristic->setValue("passingMagnet is true");
  } else if (passingMagnet){ //passed magnet, count a rotation
    passingMagnet = false;
    pCharacteristic->setValue("passingMagnet is false");
  } else {
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
  delay(10);  //At 25 mph a rotation of the trainer wheel occurs 54/sec.  
}
