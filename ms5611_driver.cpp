/**
 * @author - Ashutosh Singh Parmar
 * @file - lib/driver_baro/driver_baro.cpp
 * @brief - This file contains declarations for functions that are used to read data from onboard barometer.
 * 
 * ************************************************************************************
 * ************************************************************************************
*/
#include "ms5611_driver.h"

/**
 * @brief SPI settings object for barometer chip.
*/
SPISettings ms5611_spi_settings(10000000, MSBFIRST, SPI_MODE3);

/**
 * This function contorls the slave select pin of ms5611 barometer
 * 
 * @param uint8_t state - the logic level for slave select pin
 * 
 * @return NOTHING
 * 
 * @note CSB' is the chip select input of MS5611 sensor and thus the function is named so.
*/
void baro_ms5611 :: csb(uint8_t state)
{
    digitalWrite( this->ss_pin, state);
}

/**
 * This function selects the ms5611 chip and initialzes the spi port with required settings.
 * 
 * @param NONE
 * 
 * @return NOTHING
 */
void baro_ms5611 :: select()
{
    SPI.beginTransaction( ms5611_spi_settings );
    digitalWrite( this->ss_pin, LOW );
}

/**
 * This function deselects the ms5611 chip.
 * 
 * @param NONE
 * 
 * @return NOTHING
 */
void baro_ms5611 :: deselect()
{
    digitalWrite( this->ss_pin, HIGH );
    SPI.endTransaction();
}

/**
 * This functions sends the reset command to ms5611 chip.
 * 
 * @note It blocks the operation until the reset operation is complete.
 * @note The amount of time in ms for which it blocks the operation is defined in the MACRO MS5611_RESET_DELAY_MS.
 * @note It is the first operation that must be performed on the ms5611 chip after a power up.
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: reset(void)
{
    this->select();
    
    SPI.transfer(MS5611_RESET);
    delay(MSB5611_RESET_DELAY_MS);
    
    this->deselect();
}

/**
 * This function is used to read calibration values from ms5611 PROM.
 * 
 * @param uint8_t coeff - The coefficent number to read. The coefficients are numbered from 1 to 6 in the datasheet.
 * This libary adheres to the terminology of datasheet; thus, the numbers are 1 to 6.
 * 
 * @return uint16_t - The value read from the device PROM for the passed coefficient number.
 * 
 * @note If the coefficient number is outside the valid range, the function returns 0.
 * 
*/
uint16_t baro_ms5611 :: readCoefficients( uint8_t coeff )
{
    if(coeff<1 || coeff>6)
        return 0;

    uint16_t val=0;
    uint16_t temp=0;

    this->select();

    SPI.transfer( MS5611_READ_PROM(coeff) );

    temp = SPI.transfer(0XFF);
    val = val | (temp<<8);

    temp = SPI.transfer(0XFF);
    val = val | temp;
    
    this->deselect();

    return val;
}

/**
 * baro_ms5611 class constructor; initliazes the sensor and reads the calibration coefficients.
 * 
 * @brief Class constructor
 * 
 * @param uint8_t SS_PIN the SPI slave select pin for ms5611 chip
*/
baro_ms5611 :: baro_ms5611(uint8_t SS_PIN = DEFAULT_SS)
{
    this->ss_pin = SS_PIN;

    for( uint8_t i=0; i<6; i++ )
    {
        this->coeff[i]=0;
    }
}

/**
 * This function initializes the sensor by sending the reset command and reading and storing the coefficients from the PROM.
 * 
 * @note To improve boot times, the coefficients can be read once and stored in NVM of the host.
 * This will improve the boot time only when the memory read operation is faster than SPI access operations.
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: Initialize()
{
    this->reset();

    for( uint8_t i=0; i<=5; i++ )
    {
        this->coeff[i] = this->readCoefficients(i+1);
    }
}

/**
 * This function is used to read ADC value.
 * 
 * @note This function MUST be called only after a conversion operation is complete. Otherwise the results will be incorrect.
 * @note The ADC value read using this function corresponds to pressure or temperature depending on the last conversion performed.
 * 
 * @param NONE
 * 
 * @return uint32_t Returns a 24 bit value read from the ADC of the device.
 * 
*/
uint32_t baro_ms5611 :: readADC()
{
    uint32_t val,temp;
    val=temp=0;

    this->select();

    SPI.transfer(MS5611_READ_ADC);

    temp = SPI.transfer(0XFF);
    val = val | (temp<<16);

    temp = SPI.transfer(0XFF);
    val = val | (temp<<8);

    temp = SPI.transfer(0XFF);
    val = val | temp;

    this->deselect();

    return val;
}


/**
 ***************************************************************************************************************************
 * 
 * @brief These are blocking APIs, they block the execution for performing the invoked operation on the device.
 * 
 * @note These APIs safguard against illegal access or sequences. It is recommended to use these APIs for most of the time.
 */

/**
 * 
 * This function is used to start the Pressure conversion.
 * 
 * @note This function blocks the execution until the ADC conversion is complete.
 * @note The amount of time for which the execution is blocked (or the time required for ADC conversion) is defined in a MACRO.
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: startPressureConv()
{
    this->select();
    SPI.transfer(MS5611_CNVD1_1024);
    this->deselect();

    delay(MS5611_ADC_DELAY_MS);
    
}

/**
 * This function is used to start the Temperature conversion.
 * 
 * @note This function blocks the execution until conversion is complete.
 * @note The amount of time for which the execution is blocked (or the time required for ADC conversion) is defined in a MACRO.
 * 
 * @param NONE 
 * 
 * @return NOTHING
*/
void baro_ms5611 :: startTemperatureConv()
{
    this->select();
    SPI.transfer(MS5611_CNVD2_1024);
    this->deselect();

    delay(MS5611_ADC_DELAY_MS);
}

/**
 * This function is used to read raw 24bit digital pressure value from the chip.
 * 
 * @note This function blocks the execution until the pressure value is obtained.
 * 
 * @param NONE
 * 
 * @return uint32_t - The RAW ADC value read from the device after a pressure convertion is complete.
*/
uint32_t baro_ms5611 :: readPressureRaw()
{
    this->startPressureConv();

    return this->readADC();
}

/**
 * This function is used to read raw 24bit digital temperature value from the chip.
 * 
 * @note This function blocks the execution until the temperature value is obtained.
 * 
 * @param NONE
 * 
 * @return uint32_t - The RAW ADC value read from the device after a pressure convertion is complete
*/
uint32_t baro_ms5611 :: readTemperatureRaw()
{
    this->startTemperatureConv();

    return this->readADC();
}

/**
 * This function reads the temperature raw temperature form the device and performs the calculation mentioned in the datasheet.
 * The results are stored in obejct memeber variables and are used for temperature compensation during pressure read operation.
 * 
 * @note This function blocks the execution until object variables are updated.
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: updateTemperature()
{
    this->dT = (double)this->readTemperatureRaw() - ( this->C(5) * 256.0 );

    this->TEMP = 2000.0 + ( this->dT * ( this->C(6)  / 8388608.0 ) );

    this->OFF = ( this->C(2) * 65536.0 ) + ( ( this->C(4) * this->dT ) / 128.0 );

    this->SENS = ( this->C(1) * 32768.0 ) + ( ( this->C(3) * this->dT ) / 256.0 );
}

/**
 * This function is used to read currrent air pressure.
 * 
 * @note This function returns the pressure in pressure(mbar) * 10^2.
 * @note This function blocks the execution until pressure value is obtained.
 * @note This function used the stored values for temperature compensation. This is done becase the temperature may not change as rapidly as pressure.
 * Using stored values for a fixed number of succedding pressure measurements will probably not affect the accuracy too much.
 * 
 * @note It is solely up to the user code to periodically update the temperature measurements.
 * 
 * @param NONE
 * 
 * @return double - the pressure read from the device.
*/
double baro_ms5611 :: getPressure()
{

    this->P = ( (this->readPressureRaw() * this->SENS/2097152.0) - this->OFF )/32768.0;
    return this->P;
}

/**
 ***************************************************************************************************************************
*/


/**
 **************************************************************************************************************************
 * 
 * @brief These are non blocking APIs, they are used to improve CPU utilization of the system as they do not block the execution for performing the invoked operation on the device.
 * 
 * @note These APIs DONOT safguard against illegal access or sequences. It is upto the user code to ensure
 * that APIs are called in correct sequence.
*/

/**
 * This function start a pressure conversion in the device and exits. 
 * 
 * @note It does not wait for the operation to complete
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: startPressureConv_NB()
{
    this->select();
    SPI.transfer(MS5611_CNVD1_1024);
    this->deselect();
}

/**
 * This function starts a temperature conversion in the device and exits.
 * 
 * @note It does not wait for the operation to complete
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: startTemperatureConv_NB()
{
    this->select();
    SPI.transfer(MS5611_CNVD2_1024);
    this->deselect();
}

/**
 * This function simply reads the ADC and updates the temperature related variable.
 * 
 * @note It does not start a temperature conversion before reading ADC.
 * @note User code MUST make sure that a temperature converison was previuously started and that it has ended before calling this function.
 * 
 * @param NONE
 * 
 * @return NOTHING
*/
void baro_ms5611 :: updateTemperature_NB()
{
    this->dT = (double)this->readADC() - ( this->C(5) * 256.0 );

    this->TEMP = 2000.0 + ( this->dT * ( this->C(6)  / 8388608.0 ) );

    this->OFF = ( this->C(2) * 65536.0 ) + ( ( this->C(4) * this->dT ) / 128.0 );

    this->SENS = ( this->C(1) * 32768.0 ) + ( ( this->C(3) * this->dT ) / 256.0 );
}

/**
 * This function simply reads the ADC and calculates pressure compensated pressure value in mbar x 100.
 * 
 * @note It does not start a pressure conversion before reading ADC.
 * @note User code MUST make sure that a pressure conversion was previously start and that it has ended before callling this function.
 * 
 * @param NONE
 * 
 * @return NOTHING 
*/
double baro_ms5611 :: getPressure_NB()
{
    this->P = ( (this->readADC() * this->SENS/2097152.0) - this->OFF )/32768.0;
    return this->P;
}

/**
 **************************************************************************************************************************
*/

/**
 * This function reads and returns the temperature value stored in object variable.
 * 
 * @note This function returns the temperature in temperature(degC) * 10^2.
 * 
 * @param NONE
 * 
 * @return double - the stored temperature value.
*/
double baro_ms5611 :: getTemperature()
{
    return this->TEMP;
}

/**
 * This function reads and returns the temperature value stored in object variable.
 * 
 * @param NONE
 * 
 * @return double - the stored temperature value in deg C.
*/
double baro_ms5611 :: getTemperature_degC()
{
    return this->TEMP/100;
}

/**
 * This function returns the stored pressure in mbar
 * 
 * @param NONE
 * 
 * @return double - The stored pressure value in mbar
*/
double baro_ms5611 :: getPressure_mbar()
{
    return this->P/100;
}

/**
 * This function returns the stored pressure in inch Hg
 * 
 * @param NONE
 * 
 * @return double - The stored pressure value in inch Hg
*/
double baro_ms5611 :: getPressure_inHg()
{
    return this->P * 0.0002953;
}