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
uint32_t value = 0;
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
    bool result = (state && !oldState);
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

double calculateMphFromRpm(double rpm)
{
    double WHEEL_RADIUS = 0.00034; // in km
    double KM_TO_MI = 0.621371;

    double circumfrence = 2 * PI * WHEEL_RADIUS;
    double metricDistance = rpm * circumfrence;
    double imperialDistance = metricDistance * KM_TO_MI;
    double mph = imperialDistance * 60; // feet -> miles and minutes -> hours
                                        //    Serial.printf("rpm: %2.2f, circumfrence: %2.2f, metricDistance %2.5f , imperialDistance: %2.5f, mph: %2.2f \n",
                                        //                   rpm, circumfrence, metricDistance, imperialDistance, mph);
    return mph;
}

unsigned long distanceTime = 0;
double calculateDistanceFromMph(unsigned long distanceTimeSpan, double mph)
{
    double incrementalDistance = distanceTimeSpan * mph / 60 / 60 / 1000;
    //    Serial.printf("mph: %2.2f, distanceTimeSpan %d , incrementalDistance: %2.9f \n",
    //                   mph, distanceTimeSpan, incrementalDistance);
    return incrementalDistance;
}

double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
unsigned long caloriesTime = 0;
double calculateCaloriesFromMph(unsigned long caloriesTimeSpan, double mph)
{
    double velocity = mph * 0.44704; // translates to meters/second
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

//See https://stackoverflow.com/questions/1856514/writing-files-in-bit-form-to-a-file-in-c?rq=1
int current_bit = 0;
unsigned char bit_buffer;
FILE *f;
void writeBit (int bit)
{
  if (bit)
    bit_buffer |= (1<<current_bit);

  current_bit++;
  if (current_bit == 8)
  {
    fwrite (&bit_buffer, 1, 1, f);
    current_bit = 0;
    bit_buffer = 0;
  }
}

void flushBits (void)
{
  while (current_bit) 
    writeBit (0);
}
/* 
Fitness Machine Features Characteristic (Mandatory for FTMS)
Bit Number Definition 0=false, 1=true
0 - Average Speed Supported
1 - Cadence Supported
2 - Total Distance Supported
3 - Inclination Supported
4 - Elevation Gain Supported
5 - Pace Supported
6 - Step Count Supported
7 - Resistance Level Supported
8 - Stride Count Supported
9 - Expended Energy Supported
10 - Heart Rate Measurement Supported
11 - Metabolic Equivalent Supported
12 - Elapsed Time Supported
13 - Remaining Time Supported
14 - Power Measurement Supported
15 - Force on Belt and Power Output Supported
16 - User Data Retention Supported
17-31 - Reserved for Future Use

Indoor Bike Data characteristic
First bit refers to Characteristics bit, parentheses bit refers to corresponding Features Characteristic
More Data present (bit 0), see Sections 4.9.1.2 and 4.19, otherise Instantaneous Speed field present
Average Speed present (bit 1), see Section 4.9.1.3. - Average Speed Supported (bit 0)
Instantaneous Cadence present (bit 2), see Section 4.9.1.4. - Cadence Supported (bit 1)
Average Cadence present (bit 3), see Section 4.9.1.5. - Cadence Supported (bit 1)
Total Distance Present (bit 4), see Section 4.9.1.6. - Total Distance Supported (bit 2)
Resistance Level Present (bit 5), see Section 4.9.1.7. - Resistance Level Supported (bit 7)
Instantaneous Power Present (bit 6), see Section 4.9.1.8. - Power Measurement Supported (bit 14)
Average Power Present (bit 7), see Section 4.9.1.9. - Power Measurement Supported (bit 14)
Expended Energy Present (bit 8), see Sections 4.9.1.10, 4.9.1.11 and 4.9.1.12.- Expended Energy Supported (bit 9)
Heart Rate Present (bit 9, see Section 4.9.1.13. - Heart Rate Measurement Supported (bit 10)
Metabolic Equivalent Present (bit 10), see Section 4.9.1.14. - Metabolic Equivalent Supported (bit 11)
Elapsed Time Present (bit 11), see Section 4.9.1.15. - Elapsed Time Supported (bit 12)
Remaining Time Present (bit 12), see Section 4.9.1.16. - Remaining Time Supported (bit 13)
 */
void transmitFTMS(int rpm)
{
    // notify changed value
    if (deviceConnected)
    {
        byte test[]={0xb4,0xaf,0x98,0x1a};
        indoorBikeDataCharacteristic->setValue((uint8_t *)&value, 4);
        indoorBikeDataCharacteristic->notify();
        value++;
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
        // One time notification of supported features
        fitnessMachineFeaturesCharacteristic->setValue((uint8_t *)&value, 4);
        fitnessMachineFeaturesCharacteristic->notify();
    }
}

unsigned long elapsedTime = 0;
unsigned long elapsedSampleTime = 0;
int rev = 0;
double intervalEntries = 0;
double totalRpm = 0;
double totalMph = 0;
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
        double mph = calculateMphFromRpm(rpm);
        intervalEntries++;
        totalRpm += rpm;
        totalMph += mph;
        double avgRpm = totalRpm / intervalEntries;
        double avgMph = totalMph / intervalEntries;
        runningDistance += calculateDistanceFromMph(intervalTime, mph);
        runningCalories += calculateCaloriesFromMph(intervalTime, mph);
        Serial.println("----------------------------------------------------");
        Serial.printf("elapsedTime: %d, rev: %d \n", elapsedTime, rev);
        Serial.printf("rpm: %2.2f, avgRpm: %2.2f \n", rpm, avgRpm);
        Serial.printf("mph: %2.2f, avgMph: %2.2f \n", mph, avgMph);
        Serial.printf("distance: %2.2f, calories:  %2.5f \n", runningDistance, runningCalories);

        indicateRpmWithLight(rpm);
        // bluetooth stack will go become congested if too many packets are sent. In 6 hour test I was able to go as low as 3ms
        transmitFTMS(rpm);

        rev = 0;
        elapsedTime = millis();
    }
}
