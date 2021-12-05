/*
 * bmp280.c
 *
 *  Created on: Dec 5, 2021
 *      Author: grkm
 */

#include "bmp280.h"


signed long temperature_raw;
signed long pressure_raw;

unsigned short dig_T1;
unsigned short dig_P1;
signed short dig_T2;
signed short dig_T3;
signed short dig_P2;
signed short dig_P3;
signed short dig_P4;
signed short dig_P5;
signed short dig_P6;
signed short dig_P7;
signed short dig_P8;
signed short dig_P9;

float temperature;
float pressure;
float altitude;

uint8_t I2CRegisterRead(uint8_t dataAddr){
	uint8_t rxBuff;
	HAL_I2C_Mem_Read(&hi2c1, BMP180ADDR, dataAddr, 1, &rxBuff, 1, 1000);
	return rxBuff;
}

void I2CRegisterWrite(uint8_t dataAddr, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c1, BMP180ADDR, dataAddr, 1, &data, 1, 1000);
}

void calibrationOfBMP280(void){
	uint8_t theCalibData[24] = {0};
	HAL_I2C_Mem_Read(
			&hi2c1,
			BMP180ADDR,
			CALIB_DATA_ADDR,
			1,
			theCalibData,
			CALIB_DATA_SIZE,
			1000
			);

	dig_T1 = ((theCalibData[1]  << 8) | (theCalibData[0]));
	dig_T2 = ((theCalibData[3]  << 8) | (theCalibData[2]));
	dig_T3 = ((theCalibData[5]  << 8) | (theCalibData[4]));
	dig_P1 = ((theCalibData[7]  << 8) | (theCalibData[6]));
	dig_P2 = ((theCalibData[9]  << 8) | (theCalibData[8]));
	dig_P3 = ((theCalibData[11] << 8) | (theCalibData[10]));
	dig_P4 = ((theCalibData[13] << 8) | (theCalibData[12]));
	dig_P5 = ((theCalibData[15] << 8) | (theCalibData[14]));
	dig_P6 = ((theCalibData[17] << 8) | (theCalibData[16]));
	dig_P7 = ((theCalibData[19] << 8) | (theCalibData[18]));
	dig_P8 = ((theCalibData[21] << 8) | (theCalibData[20]));
	dig_P9 = ((theCalibData[23] << 8) | (theCalibData[22]));
}



void BMP280Init(uint8_t ctrl_meas, uint8_t config){
	I2CRegisterWrite(CTRL_MEAS, ctrl_meas);
	I2CRegisterWrite(CONFIG, config);
	calibrationOfBMP280();
}

void BMP280Calculation(void){
	uint8_t status;
	uint8_t rawData[6];

	do{
		status=I2CRegisterRead(STATUS);
	} while(
			((status & 0x08) == STATUS_CONV)||
			((status & 0x01) == STATUS_COPY)
			);

	HAL_I2C_Mem_Read(
			&hi2c1,
			BMP180ADDR,
			START_RAW_DATA_POINT,
			1,
			rawData,
			RAW_DATA_LENGTH,
			1000
			);

	temperature_raw =  ((rawData[3] << 12) | (rawData[4] << 4) | (rawData[5] >> 4));
	pressure_raw    =  ((rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4));

	double var1, var2;
	var1=(((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	var2=((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	double t_fine = (int32_t)(var1+var2);
	volatile float T = (var1+var2)/5120.0;

	var1=((double)t_fine/2.0)-64000.0;
	var2=var1*var1*((double)dig_P6)/32768.0;
	var2=var2+var1*((double)dig_P5)*2.0;
	var2=(var2/4.0)+(((double)dig_P4)*65536.0);
	var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1=(1.0+var1/32768.0)*((double)dig_P1);
	volatile double p = 1048576.0-(double)pressure_raw;
	p=(p-(var2/4096.0))*6250.0/var1;
	var1=((double)dig_P9)*p*p/2147483648.0;
	var2=p*((double)dig_P8)/32768.0;
	p=p+(var1+var2+((double)dig_P7))/16.0;

	temperature = T;
	pressure    = p;
	altitude=44330.0f*(1-powf(pressure/101325.0f,1.0f/5.255f));
}

