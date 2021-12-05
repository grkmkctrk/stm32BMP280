/*
 * bmp280.h
 *
 *  Created on: Dec 5, 2021
 *      Author: grkm
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_


#include "stm32f4xx_hal.h"
#include "math.h"


#define BMP180ADDR 0xEE

/////////////////////////
// ID //
#define CHIP_IDENT 0xD0
// ID //
/////////////////////////

/////////////////////////
// RESET //
#define RESET 0xE0
// RESET //
/////////////////////////

//////////////////////
// STATUS //
#define STATUS 0xF3
// bit 3 set to 1 whenever conversion is running
// set to 0 when the result have been transferred to the data register
#define STATUS_CONV 0x08
#define STATUS_COPY 0x01
// STATUS //
//////////////////////

//////////////////////////////////////////////////////////////////////
// CTRL_MEAS //
#define CTRL_MEAS 0xF4
//		SAMPLING
#define OVERSAMPLING_16BIT 0x01 // x1
#define OVERSAMPLING_17BIT 0x02 // x2
#define OVERSAMPLING_18BIT 0x03 // x4
#define OVERSAMPLING_19BIT 0x04 // x8
#define OVERSAMPLING_20BIT 0x05 // x16 or 6,7,8

//		MODES
#define SLEEP  0x00
#define FORCED 0x01 // or 0x02
#define NORMAL 0x03
//			Temperature Offset
#define CTRL_MEAS_T_OFFSET 0x05
//			Pressure Offset
#define CTRL_MEAS_P_OFFEST 0x02
//			Power Mode Offset
#define CTRL_MEAS_M_OFFEST 0x00
//				Temperature oversampling
#define OVERSAMPLING_T_16BIT (OVERSAMPLING_16BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_17BIT (OVERSAMPLING_17BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_18BIT (OVERSAMPLING_18BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_19BIT (OVERSAMPLING_19BIT << CTRL_MEAS_T_OFFSET)
#define OVERSAMPLING_T_20BIT (OVERSAMPLING_20BIT << CTRL_MEAS_T_OFFSET)
//				Pressure oversampling
#define OVERSAMPLING_P_16BIT (OVERSAMPLING_16BIT << CTRL_MEAS_P_OFFSET)
#define OVERSAMPLING_P_17BIT (OVERSAMPLING_17BIT << CTRL_MEAS_P_OFFSET)
#define OVERSAMPLING_P_18BIT (OVERSAMPLING_18BIT << CTRL_MEAS_P_OFFSET)
#define OVERSAMPLING_P_19BIT (OVERSAMPLING_19BIT << CTRL_MEAS_P_OFFSET)
#define OVERSAMPLING_P_20BIT (OVERSAMPLING_20BIT << CTRL_MEAS_P_OFFEST)
//				Power Mode Selection
#define SLEEP_MODE  (SLEEP  << CTRL_MEAS_M_OFFEST);
#define FORCED_MODE (FORCED << CTRL_MEAS_M_OFFEST);
#define NORMAL_MODE (NORMAL << CTRL_MEAS_M_OFFEST);
// CTRL_MEAS //
//////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////
// CONFIG
#define CONFIG 0xF5
//		T_SB_OFFSET
#define T_SB_OFFSET 0x05
//		FILTER_OFFSET
#define FILTER_OFFSET 0x02
//		SPI_EN_OFFSET
#define SPI_EN_OFFSET 0x00
//			T_SB SETTING (ms)
#define T_SB_0_5  (0x00 << T_SB_OFFSET)
#define T_SB_62_5 (0x01 << T_SB_OFFSET)
#define T_SB_125  (0x02 << T_SB_OFFSET)
#define T_SB_250  (0x03 << T_SB_OFFSET)
#define T_SB_500  (0x04 << T_SB_OFFSET)
#define T_SB_1000 (0x05 << T_SB_OFFSET)
#define T_SB_2000 (0x06 << T_SB_OFFSET)
#define T_SB_4000 (0x07 << T_SB_OFFSET)
//			FILTER
#define FILTER_2  (0x02 << FILTER_OFFSET)
#define FILTER_5  (0x04 << FILTER_OFFSET)
#define FILTER_11 (0x08 << FILTER_OFFSET)
#define FILTER_22 (0x10 << FILTER_OFFSET)
//			SPI_EN
#define SPI3W_EN 	  (0x01 << SPI_EN_OFFSET)
#define SPI4W_EN 	  (0x00 << SPI_EN_OFFSET)
// CONFIG
///////////////////////////////////////////////////

// CALIBRATION DATA
#define CALIB_DATA_ADDR 0x88
#define CALIB_DATA_SIZE 24


// RAW DATA READING
#define START_RAW_DATA_POINT 0xF7
#define RAW_DATA_LENGTH 6

extern I2C_HandleTypeDef hi2c1;

uint8_t I2CRegisterRead(
		uint8_t dataAddr
		);

void I2CRegisterWrite(
		uint8_t dataAddr,
		uint8_t data
		);

void calibrationOfBMP280(void);

void BMP280Init(
		uint8_t ctrl_meas,
		uint8_t config
		);

void BMP280Calculation(void);

#endif /* INC_BMP280_H_ */













