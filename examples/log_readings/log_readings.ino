#include <SPI.h>
#include "driver_baro.h"

baro_ms5611 sensor(40);

void setup() 
{
    Serial.begin(9600);

    SPI.begin();
    
    sensor.Initialize();

    Serial.println("Done initializing...\n");
}

void loop() 
{
    sensor.updateTemperature();
    sensor.getPressure();
    Serial.println("Temperature (degC) : "+String(sensor.getTemperature_degC()));
    Serial.println("Pressure (mbar) : "+String(sensor.getPressure_mbar()));
    Serial.println('*****************************\n');
}