
/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define FTMS_UUID "00001826-0000-1000-8000-00805f9b34fb"
#define FITNESS_MACHINE_FEATURES_UUID "00002acc-0000-1000-8000-00805f9b34fb"
#define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"
#define LED_BUILTIN 2

bool deviceConnected = false;
bool oldDeviceConnected = false;
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
    setupBluetoothServer();
    setupHalSensor();
}

BLECharacteristic *fitnessMachineFeaturesCharacteristic = NULL;
BLECharacteristic *indoorBikeDataCharacteristic = NULL;
BLEServer *pServer = NULL;
void setupBluetoothServer()
{
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
    BLEDevice::init("IC Bike");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(FTMS_UUID);
    fitnessMachineFeaturesCharacteristic = pService->createCharacteristic(
        FITNESS_MACHINE_FEATURES_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);
    indoorBikeDataCharacteristic = pService->createCharacteristic(
        INDOOR_BIKE_DATA_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);
    //  BLE2803 and BLE2902 are the UUIDs for Characteristic Declaration (0x2803) and Descriptor Declaration (0x2902).
    fitnessMachineFeaturesCharacteristic->addDescriptor(new BLE2902());
    indoorBikeDataCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // add this for backwards compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(FTMS_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Waiting for a client connection to notify...");
}

int digitalPin = 18;
int analogPin = 19;
bool magStateOld;
void setupHalSensor()
{
    pinMode(analogPin, INPUT);
    pinMode(digitalPin, INPUT);
    Serial.begin(9600);
    magStateOld = digitalRead(digitalPin);
}

//incrementRevolutions() used to synchronously update rev rather than using an ISR.
inline bool positiveEdge(bool state, bool &oldState)
{
    bool result = (state && !oldState);//latch logic
    oldState = state;
    return result;
}

double calculateRpmFromRevolutions(int revolutions, unsigned long revolutionsTime)
{
    double ROAD_WHEEL_TO_TACH_WHEEL_RATIO = 6.8;
    double instantaneousRpm = revolutions * 60000 / revolutionsTime / ROAD_WHEEL_TO_TACH_WHEEL_RATIO;
    //    Serial.printf("revolutionsTime: %d, rev: %d , instantaneousRpm: %2.9f \n",
    //                    revolutionsTime, revolutions, instantaneousRpm);
    return instantaneousRpm;
}

double calculateKphFromRpm(double rpm)
{
    double WHEEL_RADIUS = 0.00034; // in km
    double KM_TO_MI = 0.621371;

    double circumfrence = 2 * PI * WHEEL_RADIUS;
    double metricDistance = rpm * circumfrence;
    double kph = metricDistance * 60;
    double mph = kph * KM_TO_MI; 
                                        //    Serial.printf("rpm: %2.2f, circumfrence: %2.2f, metricDistance %2.5f , imperialDistance: %2.5f, mph: %2.2f \n",
                                        //                   rpm, circumfrence, metricDistance, imperialDistance, mph);
    return kph;
}

unsigned long distanceTime = 0;
double calculateDistanceFromKph(unsigned long distanceTimeSpan, double kph)
{
    double incrementalDistance = distanceTimeSpan * kph / 60 / 60 / 1000;
    //    Serial.printf("mph: %2.2f, distanceTimeSpan %d , incrementalDistance: %2.9f \n",
    //                   mph, distanceTimeSpan, incrementalDistance);
    return incrementalDistance;
}

double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;
double calculateCaloriesFromKph(unsigned long caloriesTimeSpan, double kph)
{
    //double velocity = mph * 0.44704; // translates to meters/second
    double velocity = kph * 0.277778; // translates to meters/second
    double riderWeight = 72.6;       //165 lbs
    double bikeWeight = 11.1;        //Cannondale road bike
    int theTire = 0;                 //Clinchers
    double rollingRes = tireValues[theTire];
    int theAero = 1; //Bartops
    double frontalArea = aeroValues[theAero];
    double grade = 0;
    double headwind = 0;        // converted to m/s
    double temperaturev = 15.6; // 60 degrees farenheit
    double elevation = 100;     // Meters
    double transv = 0.95;       // no one knows what this is, so why bother presenting a choice?

    /* Common calculations */
    double density = (1.293 - 0.00426 * temperaturev) * exp(-elevation / 7000.0);
    double twt = 9.8 * (riderWeight + bikeWeight); // total weight in newtons
    double A2 = 0.5 * frontalArea * density;       // full air resistance parameter
    double tres = twt * (grade + rollingRes);      // gravity and rolling resistance

    // we calculate power from velocity
    double tv = velocity + headwind;      //terminal velocity
    double A2Eff = (tv > 0.0) ? A2 : -A2; // wind in face so you must reverse effect
    double powerv = (velocity * tres + velocity * tv * tv * A2Eff) / transv;

    /* Common calculations */
    double incrementalCalories = caloriesTimeSpan * powerv * 0.24 / 60000; // simplified
    double wl = incrementalCalories / 32318.0;                             // comes from 1 lb = 3500 Calories
    return incrementalCalories;
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

//Used to convert from little-endian (ESP32) to big-endian (ARM)
/* Detailed breakdown of the math
  + lookup reverse of bottom nibble
  |       + grab bottom nibble
  |       |        + move bottom result into top nibble
  |       |        |     + combine the bottom and top results 
  |       |        |     | + lookup reverse of top nibble
  |       |        |     | |       + grab top nibble
  V       V        V     V V       V
 (lookup[n&0b1111] << 4) | lookup[n>>4]
*/
static unsigned char lookup[16] = {
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };
byte reverseBits(byte n) 
{  
   return (lookup[n&0b1111] << 4) | lookup[n>>4];
}

void littleEndianToBigEndian(byte arr[], int end)
{
    int start = 0;
    while (start < end)
    {
        byte temp = reverseBits(arr[start]); 
        arr[start] = reverseBits(arr[end]);
        arr[end] = temp;
        start++;
        end--;
    } 
}    

/*Used for debugging, i.e.
    Serial.print("Before reverseBits: ");
    printArray(bikeData, sizeof(bikeData));
    littleEndianToBigEndian(bikeData,sizeof(bikeDataures)-1);
    Serial.print("\nAfter reverseBits: ");
    printArray(bikeData, sizeof(bikeData));
*/
void printArray(byte input[], int sizeOfArray)
{
    for (size_t i = 0; i < sizeOfArray; i++) 
    {
        Serial.print(input[i]);
        Serial.print(' ');
    }
}

/* TODO: use https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.indoor_bike_data.xml
 *  to convert data to bytes in conjunction with logic below:
UINT16 value = 0xAAFF;
UINT8 array[2];
array[0]=value & 0xff;
array[1]=(value >> 8);
 */

byte features[] = {0x00,0x00,0x48,0xe0}; //Corresponds to avgSpeed (0), cadence (1), total distance (2), expended energy (9), elapsed time (12)
void transmitFTMS(int rpm, double avgRpm, double kph, double avgKph, double runningDistance, double runningCalories, unsigned long elapsedTime)
{
    uint16_t transmittedKph = (uint16_t) (kph * 100);
    Serial.printf("transmittedKph: %d\n", transmittedKph);
    Serial.printf("litte end byte 0: %d\n", (uint8_t)(transmittedKph >> 8));
    Serial.printf("litte end byte 1: %d\n", (uint8_t)transmittedKph);
    bool disconnecting = !deviceConnected && oldDeviceConnected;
    bool connecting = deviceConnected && !oldDeviceConnected;
    byte bikeData[20]={
      0x00,0x01, //TODO: uint16_t for elapsed time (seconds)
      0x02, //TODO: uint8_t for energy (kCal / min)
      0x03,0x04, //TODO: uint16_t for energy (kCal / hr)
      0x05,0x06, //TODO: uint16_t for total energy
      0x07,0x08,0x09, //TODO: uint24_t for distance (meters)
      0x0A,0x0B, //TODO: uint16_t for avgRpm (0.1 resolution)
      0x0C,0x0D, //TODO: uint16_t for rpm (0.1 resolution)
      0x0E,0x0F, //TODO: uint16_t for avgKph (0.01 resolution)
      (uint8_t)transmittedKph,(uint8_t)(transmittedKph >> 8), //FIXME: works with minor bit only, major bit seems to cause issues.  May need to debug on Android.
      0x09,0x1e};
    //0x78,0x90 = instSpeed (0 counts as true here), avgSpeed(1), instCadence (2), avgCadence (3), total distance (4), expended energy (8), elapsed time (11)
    printArray(bikeData, sizeof(bikeData));
    Serial.print("\n");
    littleEndianToBigEndian(bikeData,sizeof(bikeData)-1);
    printArray(bikeData, sizeof(bikeData));
    if (deviceConnected)
    {
        indoorBikeDataCharacteristic->setValue((uint8_t *)&bikeData, 20);
        indoorBikeDataCharacteristic->notify();
    }
    
    if (disconnecting) // give the bluetooth stack the chance to get things ready & restart advertising
    {
        delay(500);                  
        pServer->startAdvertising(); 
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    if (connecting) // execute one time notification of supported features
    { 
        oldDeviceConnected = deviceConnected;
        littleEndianToBigEndian(features,sizeof(features)-1);
        fitnessMachineFeaturesCharacteristic->setValue((byte*)&features, 4);
        fitnessMachineFeaturesCharacteristic->notify();
    }
}

unsigned long elapsedTime = 0;
unsigned long elapsedSampleTime = 0;
int rev = 0;
double intervalEntries = 0;
double totalRpm = 0;
double totaKph = 0;
double runningCalories = 0.0;
double runningDistance = 0.0;
void loop()
{
    unsigned long intervalTime = millis() - elapsedTime;
    unsigned long sampleTime = millis() - elapsedSampleTime;
    bool state = digitalRead(digitalPin);
    if (sampleTime > 10 && state != magStateOld)
    {
        rev += (int)positiveEdge(state, magStateOld);
        elapsedSampleTime = millis();
    }
    if (intervalTime > 1000)
    {
        double rpm = calculateRpmFromRevolutions(rev, intervalTime);
        double kph = calculateKphFromRpm(rpm);
        intervalEntries++;
        totalRpm += rpm;
        totaKph += kph;
        double avgRpm = totalRpm / intervalEntries;
        double avgKph = totaKph / intervalEntries;
        runningDistance += calculateDistanceFromKph(intervalTime, kph);
        runningCalories += calculateCaloriesFromKph(intervalTime, kph);
        Serial.println("\n----------------------------------------------------");
        Serial.printf("elapsedTime: %d, rev: %d \n", elapsedTime, rev);
        Serial.printf("rpm: %2.2f, avgRpm: %2.2f \n", rpm, avgRpm);
        Serial.printf("kph: %2.2f, avgKph: %2.2f \n", kph, avgKph);
        Serial.printf("distance: %2.2f, calories:  %2.5f \n", runningDistance, runningCalories);

        indicateRpmWithLight(rpm);
        // bluetooth becomes congested if too many packets are sent. In a 6 hour test I was able to go as frequent as 3ms.
        transmitFTMS(rpm,avgRpm,kph,avgKph,runningDistance,runningCalories,elapsedTime);

        rev = 0;
        elapsedTime = millis();
    }
}
