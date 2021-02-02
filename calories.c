// Function retrieved from http://bikecalculator.com/index.html
// all calculations done in metric units, JIT conversion to/from
#include <math.h>

    // get most form values
    double tireValues[] = {0.005, 0.004, 0.012}; //Clincher, Tubelar, MTB
    double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
  void calculateCalories(double velocity, double time)// measured in meters/second and seconds
    { 
    double riderWeight = 72.6; //165 lbs
    double bikeWeight = 11.1; //Cannondale road bike
    int theTire = 0; //Clinchers
    double rollingRes = tireValues[theTire];
    int theAero = 1; //Bartops
    double frontalArea = aeroValues[theAero];
    double grade = 0;
    double headwind = 0;  // converted to m/s
    double temperaturev = 15.6; // 60 degrees farenheit 
    double elevation = 100; //Meters
    double transv = 0.95; // no one knows what this is, so why bother presenting a choice?
    
    /* Common calculations */
    double density = (1.293 - 0.00426 * temperaturev) * exp(-elevation / 7000.0);
    double twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
    double A2 = 0.5 * frontalArea * density;  // full air resistance parameter
    double tres = twt * (grade + rollingRes); // gravity and rolling resistance
    
     // we calculate power from velocity
    double tv = velocity + headwind; //terminal velocity 
    double A2Eff = (tv > 0.0) ? A2 : -A2; // wind in face so you must reverse effect
    double powerv = (velocity * tres + velocity * tv * tv * A2Eff) / transv;
      
  
    
    /* Common calculations */
    double calories = time * powerv * 0.24;  // simplified
    double wl = calories / 32318.0; // comes from 1 lb = 3500 Calories
    }
