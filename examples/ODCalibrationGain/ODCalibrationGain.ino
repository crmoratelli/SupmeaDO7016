/**
 * This example calibrates the OD gain of the Supmea DO7016 probe.
 * 
 * See "OPTOD calibration gain only adjustment" on reference guide "Modbus specifications for Digital Sensors" (Aqualabo).
 *  
 * Supmea DO7016 is dissolved oxygem probe capable to obtain the following water parameters :
 *  - temperature;
 *  - dissolved oxygem in % of saturation, mg/l and ppm.
 * 
 * The probe uses RS485 to communicate, thus, requiring a conversor.
 *  Tested with a very cheap MAX485 converter connected to an Arduino Mega 2560 board on SERIAL2 port.
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

/*Operator and date for calibration validation. Data format is MMHHDDMMAAAA*/
const char * op = "John";
const char * date = "461206082021";

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

void purgeSerial(bool waitkey){
    while(Serial.available()){
        Serial.read();
    }

    while(!Serial.available() && waitkey);
}

uint8_t readSaturation(){
    uint8_t ret;
    float sat;

    while(!Serial.available()){
        /* Start the Measurement for all parameters .*/
        ret = SupProbe.startMeasurement(MEASUREMENT_SAT);
        if(ret){
            return ret;
        }

        /* Waiting time before read the results.  .*/
        while(!SupProbe.isMeasurementDone());
    
        /* Read measurement results. */
        ret = SupProbe.getSatParam(sat);
        if(ret){
            return ret;
        }

        Serial.print(sat);
        Serial.println("%");

        delay(1000);
    }

    return 0;
}

void stop(uint8_t err){
    Serial.print("ModBus error: ");
    Serial.println(err);
    cli();
    while(true);
}

void loop(){
    uint8_t ret;
    float sat;

    Serial.println("OPTOD calibration gain only adjustment.");
    Serial.println("The probe must be out of the water and exposed to the atmospheric air.");
    Serial.println("Press any key when ready...");

    purgeSerial(true);

    Serial.println("Reading atmospheric air saturation.");
    Serial.println("The air saturation is 100%. The probe must read near this value.");
    Serial.println("If the probe read a very different value, follow the steps for gain calibration.");
    Serial.println("Wait to see a stable value and press any key.");

    purgeSerial(false);

    ret = readSaturation();
    if (ret){
        stop(ret);
    }

    /* Reset Temporary Calibration. */
    SupProbe.send_frame_231();

    purgeSerial(false);

    Serial.println("Wait for the saturation to be stable again and press any key.");

    ret = readSaturation();
    if (ret){
        stop(ret);
    }

    /* Standard air saturation is 100%. */
    SupProbe.send_frame_170(MODBUS_ADDR_STD_1, 100.0);
    SupProbe.send_frame_230(COEFF4);

    Serial.println("Reading saturation with new calibration.");
    Serial.println("Press any key to confirm calibration.");

    purgeSerial(false);

    ret = readSaturation();
    if (ret){
        stop(ret);
    }

    /* Confirm Calibration. */
    SupProbe.send_frame_210(op, date);
    SupProbe.send_frame_231();

    Serial.println("Calibration gain finish.");
    Serial.println("If the result is not satisfatory, repeat the process.");

    while(true){
        purgeSerial(false);
        ret = readSaturation();
        if (ret){
            stop(ret);
        }
    }

}

  