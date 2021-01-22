// Function retrieved from http://bikecalculator.com/index.html
// all calculations done in metric units, JIT conversion to/from


		// get most form values
    double tireValues[] = {0.005, 0.004, 0.012}; //Clincher, Tubelar, MTB
    double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
	void calculateCalories() 
    { 
		riderWeight = 72.6 //165 lbs
		bikeWeight = 11.1; //Cannondale road bike
		theTire = 0; //Clinchers
		rollingRes = tireValues[theTire];
		theAero = 1; //Bartops
		frontalArea = aeroValues[theAero];
		grad = 0
		headwind = eval(headwind.value)  * (units ? 1.609 : 1.0) / 3.6;  // converted to m/s
		distancev = eval(distance.value)  * (units ? 1.609 : 1.0);
		temperaturev = 15.6 // 60 degrees farenheit 
		elevationv = 100; //Meters
		transv = 0.95; // no one knows what this is, so why bother presenting a choice?
		
		/* Common calculations */
		density = (1.293 - 0.00426 * temperaturev) * Math.exp(-elevationv / 7000.0);
		twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
		A2 = 0.5 * frontalArea * density;  // full air resistance parameter
		tres = twt * (grad + rollingRes); // gravity and rolling resistance
		
// Function retrieved from http://bikecalculator.com/index.html
// all calculations done in metric units, JIT conversion to/from


		// get most form values
    double tireValues[] = {0.005, 0.004, 0.012}; //Clincher, Tubelar, MTB
    double aeroValues[] = {0.388, 0.445, 0.420, 0.300, 0.233, 0.200}; //Hoods, Bartops, Barends, Drops, Aerobar
	void calculateCalories() 
    { 
		riderWeight = 72.6 //165 lbs
		bikeWeight = 11.1; //Cannondale road bike
		theTire = 0; //Clinchers
		rollingRes = tireValues[theTire];
		theAero = 1; //Bartops
		frontalArea = aeroValues[theAero];
		grad = 0
		headwind = eval(headwind.value)  * (units ? 1.609 : 1.0) / 3.6;  // converted to m/s
		distancev = eval(distance.value)  * (units ? 1.609 : 1.0);
		temperaturev = 15.6 // 60 degrees farenheit 
		elevationv = 100; //Meters
		transv = 0.95; // no one knows what this is, so why bother presenting a choice?
		
		
		/* Common calculations */
		density = (1.293 - 0.00426 * temperaturev) * Math.exp(-elevationv / 7000.0);
		twt = 9.8 * (riderWeight + bikeWeight);  // total weight in newtons
		A2 = 0.5 * frontalArea * density;  // full air resistance parameter
		tres = twt * (grad + rollingRes); // gravity and rolling resistance
		
		 // we calculate power from velocity
		v = eval(velocity.value) / 3.6 * (units ? 1.609 : 1.0);  // converted to m/s;
		tv = v + headwind; 
		var A2Eff = (tv > 0.0) ? A2 : -A2; // wind in face, must reverse effect
		powerv = (v * tres + v * tv * tv * A2Eff) / transv;
			
		if (v > 0.0) t = 16.6667 * distancev / v;  // v is m/s here, t is in minutes
		else t = 0.0;  // don't want any div by zero errors

		power.value = makeDecimal0(powerv);
		dragSlider.setValue(powerv/500.0);
	
		
		/* Common calculations */
		
		// c = t * 60.0 * powerv / 0.25 / 1000.0; // kilowatt-seconds, aka kilojoules. t is converted to seconds from minutes, 25% conversion efficiency
		c = t * powerv * 0.24;  // simplified
		wl = c / 32318.0; // comes from 1 lb = 3500 Calories

		/* other results */
		
		time.value = makeDecimal2(t);
		calories.value = makeDecimal0(c * (units ? 0.2388 : 1.0));
		weightloss.value = makeDecimal2(wl * (units ? 2.205 : 1.0));
	
	
		
		/* Common calculations */
		
		// c = t * 60.0 * powerv / 0.25 / 1000.0; // kilowatt-seconds, aka kilojoules. t is converted to seconds from minutes, 25% conversion efficiency
		c = t * powerv * 0.24;  // simplified
		wl = c / 32318.0; // comes from 1 lb = 3500 Calories

		/* other results */
		
		time.value = makeDecimal2(t);
		calories.value = makeDecimal0(c * (units ? 0.2388 : 1.0));
    }