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
 *  @param port Must be a HardwareSerial port.
 *  @param pre a function pointer to a routine to configure the RS485 to pre-transmissiton.
 *  @param pos a function pointer to a routine to configure the RS485 to pos-transmissiton.
 */
SupmeaDO7016::SupmeaDO7016(Stream &port, void (*preT)(), void (*posT)()): probePort(port){
    measurement_time = 0;

    /* Callbacks to configure the RS485 transceiver correctly */
    probe.preTransmission(preT);
    probe.postTransmission(posT);
}

/**
 * @brief Probe initilization. Must be called before use.
 * @param probe_addr ModBus probe address. Default probe address is 10.
 * @param serial_speed Serial port speed. Default probe speed is 9600 bps.
 * @param serial_conf Serial port configuration. Default probe configuration is SERIAL_8N2.
 * @return 0 if the probe is found otherwise ModBus error.
 */
uint8_t SupmeaDO7016::begin(uint8_t probe_addr = 10, uint16_t serial_speed = 9600, uint16_t serial_conf = SERIAL_8N2){
    uint8_t busret;

    static_cast<HardwareSerial*>(&probePort)->begin(serial_speed, serial_conf);

    /* Initiate modbus communication */
    probe.begin(probe_addr, probePort);

    /** Read the DO7016 waiting time.
    *    It will be used for further communicatons.
    */
    busret = probe.readHoldingRegisters(MODBUS_ADDR_TIME_TAKEN, 1);
    if (busret == probe.ku8MBSuccess){
        measurement_time = probe.getResponseBuffer(0);
    }

    ndelay.setdelay(measurement_time);

    return busret;
}

/**
 * @brief Convert ModBus two 16bits float notation to IEEE784 float point.
 * 
 * @param low low portion of the float.
 * @param high high portion of the float.
 */
float SupmeaDO7016::modbus_to_float(unsigned int low, unsigned int high){
    union{
        uint16_t ints[2];
        float toFloat;
    }u16ItoFloat;

    u16ItoFloat.ints[1] = high;
    u16ItoFloat.ints[0] = low;
    
    return u16ItoFloat.toFloat;
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
