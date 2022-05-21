/**
 * @author - Ashutosh Singh Parmar
 * @file - lib/driver_baro/driver_baro.h
 * @brief - This file contains declarations for functions that are used to read data from onboard barometer.
 * 
 * ************************************************************************************
 * ************************************************************************************
*/
#ifndef DRIVER_BARO
#define DRIVER_BARO

/**
 * @brief Including the Arduino framework and other required arduino libraries.
*/
#include <Arduino.h>
#include <SPI.h>

/**
 * @brief Default slave select pin.
*/
#define DEFAULT_SS 10

/**
 * @brief Commands for ms5611 barometer IC.
 */
#define MS5611_RESET 0X1E
#define MS5611_CNVD1_256 0X40
#define MS5611_CNVD1_512 0X42
#define MS5611_CNVD1_1024 0X44
#define MS5611_CNVD1_2048 0X46
#define MS5611_CNVD1_4096 0X48
#define MS5611_CNVD2_256 0X50
#define MS5611_CNVD2_512 0X52
#define MS5611_CNVD2_1024 0X54
#define MS5611_CNVD2_2048 0X56
#define MS5611_CNVD2_4096 0X58
#define MS5611_READ_ADC 0X00
#define MS5611_READ_PROM(add) ( 0XA0 | ( (add & 0x07) << 1 ) ) 

/**
 * @brief Delays required for various MS5611 sequences.
*/
#define MSB5611_RESET_DELAY_MS 3
#define MS5611_ADC_DELAY_MS 9


class baro_ms5611
{
    protected:
        /**
         * @brief This variable contains the slave select pin number for the deivce.
        */
        uint8_t ss_pin;

        /**
         * @brief This array holds the factory calibration coefficients read from the sensor during the initilization process
        */
        uint16_t coeff[6];

        /**
         * @brief These are the temperature related variables.
        */
        double dT;
        double TEMP;
        double OFF;
        double SENS;
        double P;

        void csb(uint8_t);

        void select();
        void deselect();

        void reset(void);

        uint16_t readCoefficients(uint8_t);

        /**
         * @brief This is a utility function. The coefficients in the datasheet are numbered from 1 to 6 but the array is 0 indexed.
         * This may lead to confusion during writing library functions. This is the function that maps C(1)-C(6) to their respective array members.
        */
        inline uint16_t C(uint8_t loc){
            return this->coeff[loc-1];
        }

    public:

        baro_ms5611(uint8_t );

        void Initialize();

        uint32_t readADC();

        /**
         * ******************************************************************************************************
         * 
         * @brief These are blocking APIs, they block the execution for performing the invoked operation on the device.
         * 
         * @note These APIs safguard against illegal access or sequences. It is recommended to use these APIs for most of the time.
        */
        void startPressureConv();
        void startTemperatureConv();

        uint32_t readPressureRaw();
        uint32_t readTemperatureRaw();
        
        void updateTemperature();
        double getPressure();
        /**
         * *****************************************************************************************************
        */
        

        /**
         * ******************************************************************************************************
         * 
         * @brief These are non blocking APIs, they are used to improve CPU utilization of the system as they do not block the execution for performing the invoked operation on the device.
         * 
         * @note These APIs DONOT safguard against illegal access or sequences. It is upto the user code to ensure
         * that APIs are called in correct sequence.
        */
        void startPressureConv_NB();
        void startTemperatureConv_NB();

        void updateTemperature_NB();
        double getPressure_NB();
        /**
         * *****************************************************************************************************
        */


        /**
         * @brief These APIs simply read and return the object member variables.
        */
        double getTemperature();
        double getTemperature_degC();
        double getPressure_mbar();
        double getPressure_inHg();
};

#endif