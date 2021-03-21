#ifndef MS5837_h
#define MS5837_h

#include "Arduino.h"
#include <Wire.h> //There are multiple ports in this: Wire, Wire

class MS5837 {
public:
	float pressure_mbar; //Pressure returned in mbar.
	float pressure_psi;
	float pressure_pa; //pascal
	float pressure_kpa; //kiloPascal
	float temp_c; // Temperature returned in deg C

	MS5837(); //constructor

	bool initPressureSensor();
	void read();	//read from I2C takes up to 40 ms, so use sparingly if possible.
private:
	TwoWire * _i2cPort;//This stores the requested i2c port
	uint16_t C[8];
	uint32_t D1_pres, D2_temp;
	int32_t TEMP;
	int32_t P;

	void calculate(); //Performs calculations per the sensor data sheet for conversion and second order compensation.
	uint8_t crc4(uint16_t n_prom[]);
};
#endif
