/*!
 * @file SupmeaDO7016.cpp
 *
 * This is a library for the Supmea DO-7016 dissolved oxygen probe.
 *
 * Written by Carlos Roberto Moratelli, professor at UFSC/Brazil.
 *
 * BSD license, all text here must be included in any redistribution.
 */

#include "SupmeaDO7016.h"


/**
 * @brief Class Constructor.
 */
SupmeaDO7016::SupmeaDO7016(){
    measurement_time = 0;
}

/**
 * @brief Probe initilization. Must be called before use.
 * @param port Must be a HardwareSerial port.
 * @param preT a function pointer to a routine to configure the RS485 to pre-transmissiton.
 * @param posT a function pointer to a routine to configure the RS485 to pos-transmissiton.
 * @param probe_addr ModBus probe address. Default probe address is 10.
 * @param serial_speed Serial port speed. Default probe speed is 9600 bps.
 * @param serial_conf Serial port configuration. Default probe configuration is SERIAL_8N2.
 * @return 0 if the probe is found otherwise ModBus error.
 */
uint8_t SupmeaDO7016::begin(Stream &port, 
    void (*preT)(), 
    void (*posT)(), 
    uint8_t probe_addr = 10, 
    uint16_t serial_speed = 9600, 
    uint16_t serial_conf = SERIAL_8N2){
    uint8_t busret;

    static_cast<HardwareSerial*>(&port)->begin(serial_speed, serial_conf);

    /* Callbacks to configure the RS485 transceiver correctly */
    probe.preTransmission(preT);
    probe.postTransmission(posT);

    /* Init modbus communication */
    probe.begin(probe_addr, port);

    /** Read the DO7016 waiting time.
    *    It will be used for further communicatons.
    */
    busret = probe.readHoldingRegisters(MODBUS_ADDR_TIME_TAKEN, 1);
    if (busret == probe.ku8MBSuccess){
        measurement_time = probe.getResponseBuffer(0) + 10;
    }

    ndelay.setdelay(measurement_time);

    return busret;
}

/**
 * @brief Start probe measurement.
 * @param param Probe parameters to be measured, default to all.
 * @return 0 if success, otherwise error modbus number.
 */
uint8_t SupmeaDO7016::startMeasurement(uint8_t param = MEASUREMENT_ALL){
    uint8_t busret;
    busret = probe.writeSingleRegister(MODBUS_ADDR_START, param);
    ndelay.start();
    return busret;
}

/**
 * @brief Waiting time to get the measurement done.
 *    Read data when done.
 * @return true if it done.
 */
bool SupmeaDO7016::isMeasurementDone(){
    if(ndelay.update()){
        return true;
    }
    return false;
}

/**
 * @brief Return all parameters: temp, sat, mgl and ppm. 
 * @param temp Water temperature.
 * @param sat Percentage of saturation of dissolved oxygem.
 * @param mgl MG/L of dissolved oxygem.
 * @param ppm parts per million of dissolved oxygem.
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::getAllParams(float &temp, float &sat, float &mgl, float &ppm){
    uint8_t busret = probe.readHoldingRegisters(MODBUS_ADDR_TEMPERATURE, 8);

    if (busret == probe.ku8MBSuccess){
        temp = modbus_to_float(probe.getResponseBuffer(1), probe.getResponseBuffer(0));
        sat = modbus_to_float(probe.getResponseBuffer(3), probe.getResponseBuffer(2));
        mgl = modbus_to_float(probe.getResponseBuffer(5), probe.getResponseBuffer(4));
        ppm = modbus_to_float(probe.getResponseBuffer(7), probe.getResponseBuffer(6));
    }

    return busret;
}

/**
 * @brief Get parameter temperature. 
 * @param temp Water temperature.
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::getTempParam(float &temp){
    uint8_t busret = probe.readHoldingRegisters(MODBUS_ADDR_TEMPERATURE, 2);
    temp = modbus_to_float(probe.getResponseBuffer(1), probe.getResponseBuffer(0));
    return busret;
}

/**
 * @brief Get parameter percentage of saturation. 
 * @param sat % saturation
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::getSatParam(float &sat){
    uint8_t busret = probe.readHoldingRegisters(MODBUS_ADDR_PARAM1, 2);
    sat = modbus_to_float(probe.getResponseBuffer(1), probe.getResponseBuffer(0));
    return busret;
}

/**
 * @brief Get parameter mg/l. 
 * @param mgl mg/l
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::getMGLParam(float &mgl){
    uint8_t busret = probe.readHoldingRegisters(MODBUS_ADDR_PARAM2, 2);
    mgl = modbus_to_float(probe.getResponseBuffer(1), probe.getResponseBuffer(0));
    return busret;
}

/**
 * @brief Get parameter parts per million. 
 * @param ppm Parts per Million
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::getPPMParam(float &ppm){
    uint8_t busret = probe.readHoldingRegisters(MODBUS_ADDR_PARAM3, 2);
    ppm = modbus_to_float(probe.getResponseBuffer(1), probe.getResponseBuffer(0));
    return busret;
}

/**
 * @brief DO7016 measurement time in milliseconds.
 *  The measurement time is read from the DO7016 itself.
 *  The software reads must respect this time.
 * @return Measurement waiting time.
 */
uint16_t SupmeaDO7016::getMeasurementTime(){
    return measurement_time;
}

/**
 * Reset the device * 
 * 
 * @return 0 if success otherwise modbus error code.
 */ 
uint8_t SupmeaDO7016::resetPOD(){
    uint8_t ret;
    ret = probe.writeSingleRegister(MODBUS_ADDR_RESET_POD, 1);
    return ret;
}

/**
 * @brief Frame 170
 *  Frame for calculation of a given calibration point. 
 *  This communication is used by the master when it needs to validate a calibration point. The Sensor conducts calculation of the
 *  calibration coefficient. This calculation is performed when the master writes “calibration standard X” in the memory box for the
 *  value of the solution in which the Sensor is placed. For the calculation, the Sensor retrieves the last measurement poin
 * 
 * @param std_X Calibration standard.
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::send_frame_170(uint16_t addr, float std_X){
    uint16_t high, low;

    float_to_modbus(std_X, low, high);

    delay(CONFTIME);

    Serial.println(high, HEX);
    Serial.println(low, HEX);

    probe.setTransmitBuffer(0, high);
    probe.setTransmitBuffer(1, low);
    return probe.writeMultipleRegisters(addr, 2);
}

/**
 * @brief Frame 210
 *  Frame for validating a calibration 
 *  Calibration of one or more coefficients is validated by the Sensor when the master sends operator name and the date.
 *  IMPORTANT NOTE If the master does not send this frame (and 231 ), then the calibration will not be taken into account.
 *
 * @param operator Operator's name string.
 * @param date Calibration date string.
 * return 0 if success otherwise modbus error code. 
 */ 
uint8_t SupmeaDO7016::send_frame_210(char *op, char *date){
    int i, ret, sz;

    delay(CONFTIME);
  
    /* Calc the string size to be sent. It must sent ways 16 chars (8 x uint16) */
    sz = strlen(op) > 16 ? 8 : strlen(op)/2;

    probe.clearTransmitBuffer();

    for(i=0; i<8; i++){
        if (i<sz){
            probe.setTransmitBuffer(i, ((uint16_t*)op)[i]);
        }else{
            /* set the remain bytes with space. */
            probe.setTransmitBuffer(i, 0x2020);     
        }
    }

    /* send the operator's name. */
    ret = probe.writeMultipleRegisters(MODBUS_ADDR_OP_NAME1, 8);
    if(ret != probe.ku8MBSuccess){
        return ret;
    }
  
    delay(CONFTIME);

    /* Calc the string size to be sent. It must sent always 16 chars (8 x uint16) */
    sz = strlen(date) > 16 ? 8 : strlen(date)/2;

    probe.clearTransmitBuffer();
  
    for(i=0; i<8; i++){
        if(i<sz){
            probe.setTransmitBuffer(i, ((uint16_t*)date)[i]);
        }else{
            /* set the remain bytes space. */
            probe.setTransmitBuffer(i, 0x2020);     
        }
    }

    /* send the date. */
    ret = probe.writeMultipleRegisters(MODBUS_ADDR_DATE_CAL1, 8);
    if(ret != probe.ku8MBSuccess){
        return ret;
    }
  
    /* Sensors take less than 500ms to treat the calibration validation information. */
    delay(500);

    return 0;
}


/**
 * @brief Frame 230
 *  Frame for filling out the list of “temporary coefficients to be used for the measurement”.
 *  Activating a given coefficient in this list enables the SENSOR to return the measurement not 
 *  with the correction coefficient from the current calibration, but the one from the 
 *  temporary calibration coefficie.
 * 
 * @param coeff temporary coefficient to by used for the measurement.
 * @return return 0 if success otherwise modbus error code. 
 */
uint8_t SupmeaDO7016::send_frame_230(uint16_t coeff){
    delay(CONFTIME);

    probe.setTransmitBuffer(0, 0);
    probe.setTransmitBuffer(1, coeff);
    return probe.writeMultipleRegisters(MODBUS_ADDR_COEFF_CALC, 2);
}

/**
 * @brief Frame 231
 *  Resets all "Temporary" calibration data. And the measurements use only current coefficients.
 *  With these frames, the sensor sends the measurement with coefficients from current calibration 
 *  and also resets data written in temporary calibration.
 *  Write '0x00000000 ' at 0x014C address and also 0x0001 at 0x004C address.
 * 
 * @return 0 if success otherwise modbus error code. 
 */
uint8_t SupmeaDO7016::send_frame_231(){
    uint8_t ret;

    delay(CONFTIME);

    probe.clearTransmitBuffer();
    ret = probe.writeMultipleRegisters(MODBUS_ADDR_COEFF_CALC, 2);
    if(ret != probe.ku8MBSuccess){
        return ret;
    }

    delay(CONFTIME);

    ret = probe.writeSingleRegister(MODBUS_ADDR_STD_CAL, 1);
    if(ret != probe.ku8MBSuccess){
        return ret;
    }

    return 0;
}

/**
 * @brief Convert ModBus two 16bits float notation to IEEE754 float point.
 * 
 * @param Low low portion of the float.
 * @param High high portion of the float.
 * @return Resulting float.
 */
float SupmeaDO7016::modbus_to_float(uint16_t low, uint16_t high){
    union u16I_x_Float to_float;

    to_float.ints[1] = high;
    to_float.ints[0] = low;
    
    return to_float.Float;
}

/**
 * @brief Convert float to two 16bits ModBus notation.
 * 
 * @param f Value to be converted.
 * @param low Low portion of the float.
 * @param high High portion of the float.
 */
void SupmeaDO7016::float_to_modbus(float f, uint16_t &low, uint16_t &high){
    union u16I_x_Float to_int;

    to_int.Float = f;
    
    high = to_int.ints[1];
    low = to_int.ints[0];
}


/**
 * @brief Allows to read one or more ModBus registers.
 * @param star_address  Modbus initial addresss.
 * @param data Array to store read registers.
 * @param n_reg Number of registers to be read.
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::readRegisters(uint16_t start_address, uint16_t *data, uint8_t n_reg){
    uint8_t busret, i;
    busret = probe.readHoldingRegisters(start_address, n_reg);
    if (busret == probe.ku8MBSuccess){
        for(i=0; i<n_reg; i++){
            data[i] = probe.getResponseBuffer(i); 
        }
    }
    return busret;
}

/**
 * @brief Allows to write one or more ModBus registers.
 * @param star_address  Modbus initial addresss.
 * @param data Array with data to write.
 * @param n_reg Number of registers to be written.
 * @return 0 if success otherwise modbus error code.
 */
uint8_t SupmeaDO7016::writeRegisters(uint16_t start_address, uint16_t *data, uint8_t n_reg){
    uint8_t busret, i;

    probe.clearTransmitBuffer();
    for(i=0; i<n_reg; i++){
        probe.setTransmitBuffer(i, data[i]);
    }

    busret = probe.writeMultipleRegisters(start_address, n_reg);

    return busret;
}
