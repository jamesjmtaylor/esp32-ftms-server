// /*
//     Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
//     Ported to Arduino ESP32 by Evandro Copercini
//     updates by chegewara
// */

// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>
// #include <math.h>

// // See the following for generating UUIDs:
// // https://www.uuidgenerator.net/

// #define FTMS_UUID "00001826-0000-1000-8000-00805f9b34fb"
// #define INDOOR_BIKE_DATA_CHARACTERISTIC_UUID "00002ad2-0000-1000-8000-00805f9b34fb"
// #define LED_BUILTIN 2

// void setup()
// {
//   pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
//   setupBluetoothServer();
//   setupHalSensor();
// }

// BLECharacteristic *pCharacteristic;
// void setupBluetoothServer()
// {
//   Serial.begin(115200);
//   Serial.println("Starting BLE work!");
//   BLEDevice::init("IC Bike");
//   BLEServer *pServer = BLEDevice::createServer();
//   BLEService *pService = pServer->createService(FTMS_UUID);
//   pCharacteristic = pService->createCharacteristic(
//       INDOOR_BIKE_DATA_CHARACTERISTIC_UUID,
//       BLECharacteristic::PROPERTY_READ |
//           BLECharacteristic::PROPERTY_WRITE);

//   pCharacteristic->setValue("Characteristic configured"); // Used for demonstration purposes.
//   pService->start();
//   // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // add this for backwards compatibility
//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(FTMS_UUID);
//   pAdvertising->setScanResponse(true);
//   pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
//   pAdvertising->setMinPreferred(0x12);
//   BLEDevice::startAdvertising();
//   Serial.println("Characteristic defined! Now you can read it in your phone!");
// }

// int analogPin = 18;
// int digitalPin = 19;

// //NOTE: ISR is triggered too frequently to be relied upon, even with RISING. RPMs are in the thousands.
// // This variable is used for syncronisation
// // We use it to ensure that the ISR and the loop
// // do not try to access the 'rev' variable
// // at the same time (Causes a Guru Meditation Error)
// //portMUX_TYPE revMux = portMUX_INITIALIZER_UNLOCKED;
// //
// //int rev = 0;
// //void IRAM_ATTR ISR() //interrupt service routine
// //{
// //    portENTER_CRITICAL_ISR(&revMux);
// //    rev++;
// //    Serial.printf("rev: %d \n", rev);
// //    portEXIT_CRITICAL_ISR(&revMux);
// //}

// void setupHalSensor()
// {
//   //    attachInterrupt(digitalPin, ISR, RISING); //attaching the interrupt
//   pinMode(analogPin, INPUT);
//   pinMode(digitalPin, INPUT);
//   Serial.begin(9600);
// }

// bool passedMagnet = true;
// int rev = 0;
// void incrementRevolutions(bool passingMagnet)
// {
//   // Serial.printf("passedMagnet: %d ,  passingMagnet %d , rev: %d \n", passedMagnet, passingMagnet, rev);
//   if (passedMagnet && passingMagnet)
//   { //Started a new pass of the magnet
//     passedMagnet = false;
//     rev++;
//   }
//   else if (!passedMagnet && !passingMagnet)
//   { //The new pass of the magnet is complete
//     passedMagnet = true;
//   }
// }

// unsigned long startTime = 0;
// unsigned long revolutionsTime = 0;
// int instantaneousRpm = 0;
// int calculateRpmFromRevolutions()
// {
//   revolutionsTime = millis() - startTime;
//   double ROAD_WHEEL_TO_TACH_WHEEL_RATIO = 6.8;
//   if (revolutionsTime > 1000) // One second
//   {
//     instantaneousRpm = rev * 60000 / revolutionsTime / ROAD_WHEEL_TO_TACH_WHEEL_RATIO;
//     startTime = millis();
//     rev = 0;
//   }
//   return instantaneousRpm;
// }

// double calculateMphFromRpm(int rpm)
// {
//   int WHEEL_RADIUS = 0.00034; // in km
//   double KM_TO_MI = 0.621371;
//   double circumfrence = 2 * PI * WHEEL_RADIUS;
//   double metricDistance = rpm * circumfrence;
//   double imperialDistance = metricDistance * KM_TO_MI;
//   double mph = imperialDistance * 60; // feet -> miles and minutes -> hours
//   return mph;
// }

// double calculateDistanceFromMph(double mph)
// {
//   return 0.0;
// }

// double tireValues[] = {0.005, 0.004, 0.012};                      //Clincher, Tubelar, MTB
// double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
// unsigned long caloriesTime = 0;
// double calculateCaloriesFromMph(double mph)
// {
//   double velocity = mph * 0.44704; // translates to meters/second
//   double riderWeight = 72.6;       //165 lbs
//   double bikeWeight = 11.1;        //Cannondale road bike
//   int theTire = 0;                 //Clinchers
//   double rollingRes = tireValues[theTire];
//   int theAero = 1; //Bartops
//   double frontalArea = aeroValues[theAero];
//   double grade = 0;
//   double headwind = 0;        // converted to m/s
//   double temperaturev = 15.6; // 60 degrees farenheit
//   double elevation = 100;     //Meters
//   double transv = 0.95;       // no one knows what this is, so why bother presenting a choice?

//   /* Common calculations */
//   double density = (1.293 - 0.00426 * temperaturev) * exp(-elevation / 7000.0);
//   double twt = 9.8 * (riderWeight + bikeWeight); // total weight in newtons
//   double A2 = 0.5 * frontalArea * density;       // full air resistance parameter
//   double tres = twt * (grade + rollingRes);      // gravity and rolling resistance

//   // we calculate power from velocity
//   double tv = velocity + headwind;      //terminal velocity
//   double A2Eff = (tv > 0.0) ? A2 : -A2; // wind in face so you must reverse effect
//   double powerv = (velocity * tres + velocity * tv * tv * A2Eff) / transv;

//   /* Common calculations */
//   caloriesTime = millis() - startTime;
//   double incrementalCalories = caloriesTime * powerv * 0.24; // simplified
//   double wl = incrementalCalories / 32318.0;                 // comes from 1 lb = 3500 Calories
//   return incrementalCalories;
// }

// void indicateRpmWithLight(int rpm)
// {
//   if (rpm > 1)
//   {
//     digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
//   }
//   else
//   {
//     digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
//   }
// }

// void transmitRpmOverBle(int rpm)
// {
//   pCharacteristic->setValue(rpm);
// }

// double runningDistance = 0.0;
// double runningCalories = 0.0;
// void loop()
// {
//   incrementRevolutions(digitalRead(digitalPin));
//   int rpm = calculateRpmFromRevolutions();
//   double mph = calculateMphFromRpm(rpm);
//   runningDistance += calculateDistanceFromMph(mph);
//   runningCalories += calculateCaloriesFromMph(mph);

//   Serial.printf("RPM: %d, Revolutions %d , MPH: %d , startTime: %d \n", rpm, rev, mph);
//   indicateRpmWithLight(rpm);
//   transmitRpmOverBle(rpm);
// }