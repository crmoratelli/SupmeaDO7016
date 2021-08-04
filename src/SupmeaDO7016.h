/*!
 * @file SupmeaDO7016.h
 *
 * This is a library for the Supmea DO-7016 dissolved oxygen probe.
 *
 * Written by Carlos Roberto Moratelli, professor at UFSC/Brazil.
 *
 * BSD license, all text here must be included in any redistribution.
 */

#ifndef __SUPMEADO7016__
#define __SUPMEADO7016__

#include <ModbusMaster.h>
#include <NoDelay.h>

/**
 *  Modbus  addresses 
 * 
 *  TODO: The probe has a large number of registers, I listed below only the required by this library.
 * */
enum{
    MODBUS_ADDR_START       =   0x0001,   /* Order for starting a measurement on one or more parameters (simultaneously) */
    MODBUS_ADDR_DEF_COEFF   =   0x0002,   /* Return to factory coefficient */
    MODBUS_ADDR_STD_CAL     =   0x004C,   /* Reset Standard + Operator + Temporary Calibration Date */    
    MODBUS_ADDR_TIME_TAKEN  =   0x00A4,   /* Approximate time taken to obtain all the measurements */
    MODBUS_ADDR_TEMPERATURE =   0x0053,   /* Temperature measurement */
    MODBUS_ADDR_PARAM1      =   0x0055,   /* Measurement of Param 1 (sat) */  
    MODBUS_ADDR_PARAM2      =   0x0057,   /* Measurement of Param 2 (mgl) */  
    MODBUS_ADDR_PARAM3      =   0x0059,   /* Measurement of Param 3 (ppm) */  
    MODBUS_ADDR_COEFF_CALC  =   0x014C,   /* List of temporary Coeffs that must be used for the measurement calculation */
    MODBUS_ADDR_STD_4       =   0x0204,   /* Write standard 4 */  
    MODBUS_ADDR_TEMP_STD_1  =   0x0200,   /* Write standard 1 of the temperature */
    MODBUS_ADDR_OP_NAME1    =   0x028E,   /* Name of the operator who calibrated parameter 1 (temporary calibration) */
    MODBUS_ADDR_DATE_CAL1   =   0x0296,   /* Date of calibration of parameter 1 (temporary calibration) */
};

/* Mesurement Params */
enum {
    MEASUREMENT_TEMP   =   0x1,    /* Water Temperature */ 
    MEASUREMENT_SAT    =   0x2,    /* Param 1 - Dissolved Oxygem Saturation */ 
    MEASUREMENT_MGL    =   0x4,    /* Param 2 - Dissolved Oxygem MG/L */ 
    MEASUREMENT_PPM    =   0x8,    /* Param 3 - Dissolved Oxygem Parts Per Million */ 
    MEASUREMENT_ALL    =   0xF,    /* All  parameters */
};



class SupmeaDO7016 {
    private:
        ModbusMaster probe;
        noDelay ndelay;
        uint16_t measurement_time;
        
    public:
        /**
        * High Level API 
        * 
        * Easy to use, but with limited functionalities.
        * 
        * TODO: Implement more functionalities as calibration routines. 
        * */
        SupmeaDO7016();
        uint8_t begin(Stream &port, void (*preT)(), void (*posT)(), uint8_t probe_addr = 10, uint16_t serial_speed = 9600, uint16_t serial_conf = SERIAL_8N2);
        uint8_t startMeasurement(uint8_t param = MEASUREMENT_ALL);
        bool isMeasurementDone();
        uint8_t getAllParams(float &temp, float &sat, float &mgl, float &ppm);
        uint8_t getTempParam(float &temp);
        uint8_t getSatParam(float &sat);
        uint8_t getMGLParam(float &mgl);
        uint8_t getPPMParam(float &ppm);
        uint16_t getMeasurementTime();


        /**
        * Low Level API 
        * 
        * Allows to read/write any probe register.
        * */
        float modbus_to_float(unsigned int low, unsigned int high);
        uint8_t readRegisters(uint16_t start_address, uint16_t *data, uint8_t n_reg);
        uint8_t writeRegisters(uint16_t start_address, uint16_t *data, uint8_t n_reg);
    
};


#endif /* __SUPMEADO7016__ */
