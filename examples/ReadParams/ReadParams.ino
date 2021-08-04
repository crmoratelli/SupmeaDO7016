/**
 * This is an example of how to read measurament parameters from Supmea DO7016 probe.
 *  
 * Supmea DO7016 is dissolved oxygem probe capable to obtain the following water parameters :
 *  - temperature;
 *  - dissolved oxygem in % of saturation, mg/l and ppm.
 * 
 * The probe uses RS485 to communicate, thus, requiring a conversor.
 *  Tested with a very cheap MAX485 conversor connected to an Arduino Mega 2560 board on SERIAL2 port.
 * 
 * 
 * Written by Carlos Roberto Moratelli, professor at UFSC/Brazil.
 * 
 * BSD license, all text above must be included in any redistribution.
 */ 

#include <SupmeaDO7016.h>

/**
 * RE e DE Pins of the RS485 conversor.
 *  This example considers pin 23 for both RE and DE.
 */
#define MAX485_RE_DE 23


/**
 * Functions to control RE and DE pins of RS485.
 *  You must rewrite this functions accordally with your RS485 conversor needs.
*/
void preTransmission(){
    digitalWrite(MAX485_RE_DE, 1);
}

void postTransmission(){
    digitalWrite(MAX485_RE_DE, 0);
}

SupmeaDO7016 SupProbe;

void setup(){   
    uint8_t ret;

    Serial.begin(9600);

    pinMode(MAX485_RE_DE, OUTPUT);

    /* Must call begin before first probe use. */
    ret = SupProbe.begin(Serial2, preTransmission, postTransmission);
    if(ret){
        Serial.print("ModBus error code ");        
        Serial.println(ret);
    }
}

void loop(){
    uint8_t ret;
    float temp, sat, mgl, ppm;

    /* Start the Measurement for all parameters .*/
    ret = SupProbe.startMeasurement();
    if(ret){
        goto error;
    }

    /* Waiting time before read the results.  .*/
    while(!SupProbe.isMeasurementDone());
    
    /* Read measurement results. */
    ret = SupProbe.getAllParams(temp, sat, mgl, ppm);
    if(ret){
        goto error;
    }

    Serial.print("temp:");
    Serial.print(temp);

    Serial.print(" sat:");
    Serial.print(sat);

    Serial.print(" mgl:");
    Serial.print(mgl);

    Serial.print(" ppm:");
    Serial.println(ppm);

    delay(1000);

    return;

error:
    Serial.print("ModBus error code ");        
    Serial.println(ret);

}

  