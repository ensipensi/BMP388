/**
 **********************************************************************************************
 * @file: bmp388.c
 *
 * @brief: Library for Bosch BMP388
 *         This file contains all methods and functions needed.
 *
 * @author: Juan Francisco Padilla Fuentes (juanfrancisco.padilla@sensify.es)
 **********************************************************************************************
 * @version: 1.0
 * @date: 2023-05-04
 **********************************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Internet of Things SL.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * Credits: This library is based on the library shared in github by
 * Michał Słomiany (mslomiany@outlook.com) on the repository MSlomiany/BMP388-STM32 under MIT
 * License.
 **********************************************************************************************
 */

/* Includes */
#include "bmp388.h"
#include "private/bmp_388_priv_regs.h"
#include "bmp_i2c.h"

/****************************************************************** PRIVATE FUNCTIONS *******************************************************************/

/**
 *  \ingroup bmp388SelfTest
 * \page bmp388_api_Calculate_Crc Calculate_Crc
 * \code
 * int8_t Calculate_Crc(uint8_t seed, uint8_t data)
 * \endcode
 * @brief Function calculate cyclic redundancy code for coefficient values
 *
 * @param[in] seed CRC seed
 * @param[in] data Data to validate
 *
 * @return int8_t - calculated CRC
 */
int8_t BMP_Calculate_Crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x1D;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		if ((seed & 0x80) ^ (data & 0x80))
		{
			var2 = 1;
		}
		else
		{
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_Check_Chip_Id Check_Chip_Id
 * \code
 * STATUS Check_Chip_Id(BmpInstance *pMyBmp)
 * \endcode
 * @brief Function check sensor ID and indicate whether valid device is communicating.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates whether sensor id is valid
 * @retval 0 valid id (STATUS_OK)
 * @retval >0 invalid id, check STATUS enum
 */
STATUS BMP_Check_Chip_Id(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	uint8_t tmp;

	result = BMP_Read8(pMyBmp->I2ch, REG_CHIP_ID, &tmp);
	if (result == STATUS_OK && tmp != BMP_CHIP_ID)
	{
		result = ERR_WRONG_CHIP_ID;
	}
	return result;
}

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_Save_Calibration_Coeffs Save_Calibration_Coeffs
 * \code
 * STATUS Save_Calibration_Coeffs(BmpInstance *pMyBmp, BmpRegCalibData *bmp_reg_calib_data)
 * \endcode
 * @brief Function read, parse and save calibration coefficients to bmp_reg_calib_data structure.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates if coefficients are saved correctly or not
 */
STATUS BMP_Save_Calibration_Coeffs(BmpInstance *pMyBmp, BmpRegCalibData *data)
{
	STATUS result = STATUS_OK;

	if (BMP_Read16(pMyBmp->I2ch, REG_T1, &data->nvm_par_t1))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_T2, &data->nvm_par_t2))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_T3, (uint8_t *)&data->nvm_par_t3))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_P1, (uint16_t *)&data->nvm_par_p1))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_P2, (uint16_t *)&data->nvm_par_p2))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P3, (uint8_t *)&data->nvm_par_p3))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P4, (uint8_t *)&data->nvm_par_p4))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_P5, &data->nvm_par_p5))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_P6, &data->nvm_par_p6))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P7, (uint8_t *)&data->nvm_par_p7))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P8, (uint8_t *)&data->nvm_par_p8))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read16(pMyBmp->I2ch, REG_P9, (uint16_t *)&data->nvm_par_p9))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P10, (uint8_t *)&data->nvm_par_p10))
	{
		result = ERR_I2C;
	}
	else if (BMP_Read8(pMyBmp->I2ch, REG_P11, (uint8_t *)&data->nvm_par_p11))
	{
		result = ERR_I2C;
	}

	return result;
}

/**
 * @brief Function that obtains calibration coefficients
 *
 * @param pMyBmp BMPx instance pointer
 */
void BMP_Get_Calibration_Coeffs(BmpInstance *pMyBmp)
{
	BmpRegCalibData data;

	if (BMP_Save_Calibration_Coeffs(pMyBmp, &data) == STATUS_OK)
	{
		pMyBmp->CalData.par_t1 = data.nvm_par_t1 / 0.00390625;
		pMyBmp->CalData.par_t2 = (data.nvm_par_t2 / 1.07e9);
		pMyBmp->CalData.par_t3 = data.nvm_par_t3 / 2.81e14;
		pMyBmp->CalData.par_p1 = (data.nvm_par_p1 - 16384) / 1.048576e6;
		pMyBmp->CalData.par_p2 = (data.nvm_par_p2 - 16384) / 536870912;
		pMyBmp->CalData.par_p3 = data.nvm_par_p3 / 4294967296;
		pMyBmp->CalData.par_p4 = data.nvm_par_p4 / 137438953472;
		pMyBmp->CalData.par_p5 = data.nvm_par_p5 / 0.125;
		pMyBmp->CalData.par_p6 = (double)data.nvm_par_p6 / 64;
		pMyBmp->CalData.par_p7 = (double)data.nvm_par_p7 / 256;
		pMyBmp->CalData.par_p8 = (double)data.nvm_par_p8 / 3.2768e4;
		pMyBmp->CalData.par_p9 = (double)data.nvm_par_p9 / 2.8147e14;
		pMyBmp->CalData.par_p10 = (double)data.nvm_par_p10 / 2.8147e14;
		pMyBmp->CalData.par_p11 = (double)data.nvm_par_p11 / 3.6893e19;
	}
}

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_Check_Calibration_Coeffs Check_Calibration_Coeffs
 * \code
 * STATUS Check_Calibration_Coeffs(BmpInstance *pMyBmp);
 * \endcode
 * @brief Function check if calibration coefficients stored in device's memory are valid.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates whether calibration coefficients are valid
 * @retval 0 valid coefficients (STATUS_OK)
 * @retval >0 invalid coefficients, check STATUS enum
 */
STATUS BMP_Check_Calibration_Coeffs(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;
	uint8_t crc = 0xFF;
	uint8_t stored_crc;
	uint8_t coeffsArray[21];
	uint8_t i;
	BmpRegCalibData data;

	if (HAL_I2C_Mem_Read(pMyBmp->I2ch, BMP388_I2C_ADDR, REG_T1, 1, coeffsArray, 21, 10))
	{
		result = ERR_I2C;
	}
	for (i = 0; i < 21; i++)
	{
		crc = (uint8_t)BMP_Calculate_Crc(crc, coeffsArray[i]);
	}
	crc = (crc ^ 0xFF);
	result = BMP_Read8(pMyBmp->I2ch, REG_CRC, &stored_crc);
	if (stored_crc != crc)
	{
		pMyBmp->CalData = (BmpCalibrationData){0};
		result = ERR_INVALID_COEFFICIENTS;
	}
	else
	{
		result = BMP_Save_Calibration_Coeffs(pMyBmp, &data);
		BMP_Get_Calibration_Coeffs(pMyBmp);
	}

	return result;
}

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_Compensate_Temperature Compensate_Temperature
 * \code
 * int64_t Compensate_Temperature(BmpInstance *pMyBmp, uint32_t temperature);
 * \endcode
 * @brief Function calculate compensated temperature
 *
 * @param[in] pMyBmp BMPx instance pointer that has to be initialised
 * @param[in] temperature Uncompensated raw temperature
 *
 * @return int64_t - compensated temperature
 */
int64_t Compensate_Temperature(BmpInstance *pMyBmp, uint32_t temperature)
{
	float partial_data1;
	float partial_data2;

	partial_data1 = (float)(temperature - pMyBmp->CalData.par_t1);
	partial_data2 = (float)(partial_data1 * pMyBmp->CalData.par_t2);

	pMyBmp->CalData.t_comp = partial_data2 + (partial_data1 * partial_data1) * pMyBmp->CalData.par_t3;

	return pMyBmp->CalData.t_comp;
}

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_Read_Temperature Read_Temperature
 * @brief Function read and parse measured temperature in Celsius degrees
 * \code
 * float Parse_Temperature(BmpInstance *pMyBmp, uint32_t raw_temperature);
 * \endcode
 *
 * @param[in] pMyBmp BMPx instance pointer that has to be initialised.
 * @param[in] raw_temperature Raw temperature from device registers
 *
 * @return float - measured temperature
 */
float Parse_Temperature(BmpInstance *pMyBmp, uint32_t raw_temperature)
{
	float temperature = Compensate_Temperature(pMyBmp, raw_temperature);
	return temperature;
}

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_Compensate_Pressure Compensate_Pressure
 * \code
 * uint64_t Compensate_Pressure(BmpInstance *pMyBmp, uint32_t pressure);
 * \endcode
 * @brief Function calculate compensated pressure
 *
 * @param[in] pressure Uncompensated raw pressure
 * @return uint64_t - compensated pressure
 */
uint64_t Compensate_Pressure(BmpInstance *pMyBmp, uint32_t pressure)
{

	float comp_press;
	double pd_1;
	double pd_2;
	double pd_3;
	double pd_4;
	double po_1;
	double po_2;

	pd_1 = pMyBmp->CalData.par_p6 * pMyBmp->CalData.t_comp;
	pd_2 = pMyBmp->CalData.par_p7 * (pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp);
	pd_3 = pMyBmp->CalData.par_p8 * (pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp);
	po_1 = pMyBmp->CalData.par_p5 + pd_1 + pd_2 + pd_3;

	pd_1 = pMyBmp->CalData.par_p2 * pMyBmp->CalData.t_comp;
	pd_2 = pMyBmp->CalData.par_p3 * (pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp);
	pd_3 = pMyBmp->CalData.par_p4 * (pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp * pMyBmp->CalData.t_comp);
	po_2 = (double)pressure * (pMyBmp->CalData.par_p1 + pd_1 + pd_2 + pd_3);

	pd_1 = (double)pressure * (double)pressure;
	pd_2 = pMyBmp->CalData.par_p9 + pMyBmp->CalData.par_p10 * pMyBmp->CalData.t_comp;
	pd_3 = pd_1 * pd_2;
	pd_4 = pd_3 + ((double)pressure * (double)pressure * (double)pressure) * pMyBmp->CalData.par_p11;
	comp_press = po_1 + po_2 + pd_4;
	return comp_press;
}
/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_Parse_Pressure Parse_Pressure
 * \code
 * float Parse_Pressure(BmpInstance *pMyBmp, uint32_t raw_pressure);
 * \endcode
 * @brief Function parse measured pressure. Since temperature is neccessary to
 * compensate raw readed pressure, function has to be used after Parse_temperature() one.
 *
 * @param[in] pMyBmp BMPx instance pointer that has to be initialised
 * @param[in] raw_pressure Pointer to variable storing pressure measurement
 *
 * @return float - measured pressure
 */
float Parse_Pressure(BmpInstance *pMyBmp, uint32_t raw_pressure)
{
	float pressure = Compensate_Pressure(pMyBmp, raw_pressure);
	return pressure;
}

/****************************************************************** PUBLIC FUNCTIONS *******************************************************************/
/**
 * @brief API entry point. Initialize sensor with selected values and I2C bus. Perform self-test and
 * parse calibration coefficients.
 *
 */
STATUS BMP_Init(BmpInstance *pMyBmp, I2C_HandleTypeDef *I2ch, uint16_t en_pin, GPIO_TypeDef *en_port,
				bool pres_r, bool temp_r, MODE mode, OVERSAMPLING osr_p, OVERSAMPLING osr_t, IIR_FILTER iir_filter, DATA_RATE data_rate)
{
	uint8_t result = 0xFF;
	uint32_t time_counter_ms = 0;
	uint32_t timeout_ms = 100;
	// BMPx instance variables settings
	pMyBmp->I2ch = I2ch;
	pMyBmp->en_pin = en_pin;
	pMyBmp->en_port = en_port;
	BMP_Turn_On(pMyBmp);

	uint32_t initial_time_ms = (HAL_GetTick() * 100 / HAL_GetTickFreq());
	while (result != STATUS_OK && time_counter_ms < timeout_ms)
	{
		result = BMP_Check_Chip_Id(pMyBmp);
		if (result == STATUS_OK)
		{
			result = BMP_Check_Calibration_Coeffs(pMyBmp);
		}
		if (result == STATUS_OK)
		{
			result = BMP_Set_Oversampling(pMyBmp, osr_p, osr_t);
		}
		if (result == STATUS_OK)
		{
			result = BMP_Set_Filter_Coefficient(pMyBmp, iir_filter);
		}
		if (result == STATUS_OK)
		{
			result = BMP_Set_Output_Data_Rate(pMyBmp, data_rate);
		}
		if (result == STATUS_OK)
		{
			result = BMP_Set_Power_Mode(pMyBmp, mode, temp_r, pres_r);
		}
		time_counter_ms += ((HAL_GetTick() * 1000 / HAL_GetTickFreq()) - initial_time_ms);
	}

	return result;
}

/**
 * @brief Function turn on BMPx sensor
 *
 */
STATUS BMP_Turn_On(BmpInstance *pMyBmp)
{
	uint8_t result = STATUS_OK;
	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else if (HAL_GPIO_ReadPin(pMyBmp->en_port, pMyBmp->en_pin) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(pMyBmp->en_port, pMyBmp->en_pin, GPIO_PIN_SET);
		//        HAL_I2C_Init(pMyBmp->I2ch);
		DELAY(20); // Delay of 20 ms necessary after power on.

		if (pMyBmp->PowerControl.select_mode == NORMAL_MODE ||
			pMyBmp->PowerControl.select_mode == FORCED_MODE ||
			pMyBmp->PowerControl.select_mode == SLEEP_MODE)
		{
			result = BMP_Set_Power_Mode(pMyBmp,
										pMyBmp->PowerControl.select_mode,
										pMyBmp->PowerControl.temp_en,
										pMyBmp->PowerControl.pres_en);
			DELAY(80); // Delay of 80 ms necessary after selecting mode.
		}
		else
		{
			result = ERR_INIT;
		}
	}
	else
	{
		result = ERR_SWITCH_STATE;
	}

	return result;
}

/**
 * @brief Function turn off BMPx sensor
 *
 */
STATUS BMP_Turn_Off(BmpInstance *pMyBmp)
{
	uint8_t result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else if (HAL_GPIO_ReadPin(pMyBmp->en_port, pMyBmp->en_pin) == GPIO_PIN_SET)
	{
		HAL_GPIO_WritePin(pMyBmp->en_port, pMyBmp->en_pin, GPIO_PIN_RESET);
		//        HAL_I2C_DeInit(pMyBmp->I2ch);
	}
	else
	{
		result = ERR_SWITCH_STATE;
	}

	return result;
}

/**
 * @brief Function parse measured temperature and pressure as float values.
 *
 */
STATUS BMP_Get_Data(BmpInstance *pMyBmp, float *pressure, float *temperature)
{
	uint8_t result = STATUS_OK;
	uint32_t tmp_temp;
	uint32_t tmp_pres;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		result = BMP_Read24(pMyBmp->I2ch, REG_DATA, &tmp_pres, &tmp_temp);
		*temperature = Parse_Temperature(pMyBmp, tmp_temp);
		*pressure = Parse_Pressure(pMyBmp, tmp_pres);
	}

	return result;
}

/**
 * @brief Get pressure IIR filter coefficient
 *
 */
STATUS BMP_Get_Filter_Coefficient(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = 0;
		result = BMP_Read8(pMyBmp->I2ch, REG_IIR_CONFIG, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->filter_coefficient = ((tmp & 0b00001110) >> 1);
		}
		else
		{
			pMyBmp->filter_coefficient = 0xFF;
		}
	}
	return result;
}

/**
 * @brief Set pressure IIR filter coefficient
 *
 */
STATUS BMP_Set_Filter_Coefficient(BmpInstance *pMyBmp, IIR_FILTER coeff)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;

		tmp = (coeff << 1);
		result = BMP_Write8(pMyBmp->I2ch, REG_IIR_CONFIG, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->filter_coefficient = coeff;
		}
		else
		{
			pMyBmp->filter_coefficient = 0xFF;
		}
	}

	return result;
}

/**
 * @brief Get sensor output data rate
 *
 */
STATUS BMP_Get_Output_Data_Rate(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = 0;
		result = BMP_Read8(pMyBmp->I2ch, REG_ODR, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->data_rate = tmp & 0b00011111;
		}
		else
		{
			pMyBmp->data_rate = 0xFF;
		}
	}

	return result;
}

/**
 * @brief Set sensor output data rate
 *
 */
STATUS BMP_Set_Output_Data_Rate(BmpInstance *pMyBmp, DATA_RATE data_rate)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		result = BMP_Write8(pMyBmp->I2ch, REG_ODR, data_rate);
		if (result == STATUS_OK)
		{
			pMyBmp->data_rate = data_rate;
		}
		else
		{
			pMyBmp->data_rate = 0xFF;
		}
	}

	return result;
}

/**
 * @brief Get pressure and temperature oversampling settled before
 *
 */
STATUS BMP_Get_Oversampling(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = 0;

		result = BMP_Read8(pMyBmp->I2ch, REG_OSR, &tmp);

		if (result == STATUS_OK)
		{
			pMyBmp->pres_oversampling = (0b00000111 & tmp);
			pMyBmp->temp_oversampling = (0b00111000 & tmp) >> 3;
		}
		else
		{
			pMyBmp->pres_oversampling = 0xFF;
			pMyBmp->temp_oversampling = 0xFF;
		}
	}

	return result;
}

/**
 * @brief Set selected pressure and temperature oversampling
 *
 */
STATUS BMP_Set_Oversampling(BmpInstance *pMyBmp, OVERSAMPLING press_osr, OVERSAMPLING temp_osr)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = (temp_osr << 3 | press_osr);

		result = BMP_Write8(pMyBmp->I2ch, REG_OSR, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->pres_oversampling = press_osr;
			pMyBmp->temp_oversampling = temp_osr;
		}
		else
		{
			pMyBmp->pres_oversampling = 0xFF;
			pMyBmp->temp_oversampling = 0xFF;
		}
	}

	return result;
}

/**
 * @brief Get sensor power mode
 *
 */
STATUS BMP_Get_Power_Mode(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = 0;

		result = BMP_Read8(pMyBmp->I2ch, REG_PWR_CNTRL, &tmp);

		if (result == STATUS_OK)
		{
			pMyBmp->PowerControl.select_mode = (0b00110000 & tmp) >> 4;
			pMyBmp->PowerControl.temp_en = (0b00000010 & tmp) >> 1;
			pMyBmp->PowerControl.pres_en = (0b00000001 & tmp);
		}
		else
		{
			pMyBmp->PowerControl.select_mode = 0xFF;
			pMyBmp->PowerControl.pres_en = 0x00;
			pMyBmp->PowerControl.temp_en = 0x00;
		}
	}

	return result;
}

/**
 * @brief Set sensor power mode
 *
 */
STATUS BMP_Set_Power_Mode(BmpInstance *pMyBmp, MODE select_mode, bool temp_en, bool pres_en)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;
		tmp = (select_mode << 4 | temp_en << 1 | pres_en);

		result = BMP_Write8(pMyBmp->I2ch, REG_PWR_CNTRL, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->PowerControl.select_mode = select_mode;
			pMyBmp->PowerControl.pres_en = pres_en;
			pMyBmp->PowerControl.temp_en = temp_en;
		}
		else
		{
			pMyBmp->PowerControl.select_mode = 0xFF;
			pMyBmp->PowerControl.pres_en = 0x00;
			pMyBmp->PowerControl.temp_en = 0x00;
		}
	}

	return result;
}

/**
 * @brief Get FIFO configuration.
 *
 */
STATUS BMP_Get_FIFO_Config(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;

		result = BMP_Read8(pMyBmp->I2ch, REG_FIFO_CONFIG_1, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->FIFOConf.Config1.enable = (0b00000001 & tmp);
			pMyBmp->FIFOConf.Config1.stop = (0b00000010 & tmp) >> 1;
			pMyBmp->FIFOConf.Config1.time = (0b00000100 & tmp) >> 2;
			pMyBmp->FIFOConf.Config1.press = (0b00001000 & tmp) >> 3;
			pMyBmp->FIFOConf.Config1.temp = (0b00010000 & tmp) >> 4;

			result = BMP_Read8(pMyBmp->I2ch, REG_FIFO_CONFIG_2, &tmp);
			if (result == STATUS_OK)
			{
				pMyBmp->FIFOConf.Config2.subsampling = (0b00000111 & tmp);
				pMyBmp->FIFOConf.Config2.data_select = (0b00011000 & tmp) >> 3;
			}
			else
			{
				pMyBmp->FIFOConf.Config2.data_select = 0;
				pMyBmp->FIFOConf.Config2.subsampling = 0xFF;
			}
		}
		else
		{
			pMyBmp->FIFOConf.Config1.enable = 0;
			pMyBmp->FIFOConf.Config1.stop = 0;
			pMyBmp->FIFOConf.Config1.time = 0;
			pMyBmp->FIFOConf.Config1.press = 0;
			pMyBmp->FIFOConf.Config1.temp = 0;

			pMyBmp->FIFOConf.Config2.subsampling = 0xFF;
			pMyBmp->FIFOConf.Config2.data_select = 0;
		}
	}

	return result;
}

/**
 * @brief Set FIFO configuration.
 *
 */
STATUS BMP_Set_FIFO_Config(BmpInstance *pMyBmp, FIFOConfig1 *Config1, FIFOConfig2 *Config2)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp = 0;

		tmp = (0x00 | Config1->enable | (Config1->stop << 1) | (Config1->time << 2) | (Config1->press << 3) |
			   (Config1->temp << 4));
		result = BMP_Write8(pMyBmp->I2ch, REG_FIFO_CONFIG_1, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->FIFOConf.Config1.enable = Config1->enable;
			pMyBmp->FIFOConf.Config1.stop = Config1->stop;
			pMyBmp->FIFOConf.Config1.time = Config1->time;
			pMyBmp->FIFOConf.Config1.press = Config1->press;
			pMyBmp->FIFOConf.Config1.temp = Config1->temp;

			tmp = (0x00 | Config2->subsampling | Config2->data_select << 3);
			result = BMP_Write8(pMyBmp->I2ch, REG_FIFO_CONFIG_2, tmp);
			if (result == STATUS_OK)
			{
				pMyBmp->FIFOConf.Config2.subsampling = Config2->subsampling;
				pMyBmp->FIFOConf.Config2.data_select = Config2->data_select;
			}
			else
			{
				pMyBmp->FIFOConf.Config2.data_select = 0;
				pMyBmp->FIFOConf.Config2.subsampling = 0xFF;
			}
		}
		else
		{
			pMyBmp->FIFOConf.Config1.enable = 0;
			pMyBmp->FIFOConf.Config1.stop = 0;
			pMyBmp->FIFOConf.Config1.time = 0;
			pMyBmp->FIFOConf.Config1.press = 0;
			pMyBmp->FIFOConf.Config1.temp = 0;

			pMyBmp->FIFOConf.Config2.subsampling = 0xFF;
			pMyBmp->FIFOConf.Config2.data_select = 0;
		}
	}

	return result;
}

/**
 * @brief Get FIFO watermark. To use it, fwtm_en (fifo watermark enable) must be set to true in Int_Ctrl register.
 *        Also, if FIFO watermark is 0, it will never activate fifo watermark interrupt. Value must be set from 0 to
 *        511 (512 bytes is maximum fifo_length).
 *
 */
STATUS BMP_Get_FIFO_Watermark(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint16_t tmp;

		result = BMP_Read16(pMyBmp->I2ch, REG_FIFO_WTM, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->FIFOConf.fifo_wtm = tmp;
		}
		else
		{
			pMyBmp->FIFOConf.fifo_wtm = 0;
		}
	}

	return result;
}

/**
 * @brief Set FIFO watermark. To use it, fwtm_en (fifo watermark enable) must be set to true in Int_Ctrl register.
 *        Also, if FIFO watermark is 0, it will never activate fifo watermark interrupt. Value must be set from 0 to
 *        511 (512 bytes is maximum fifo_length).
 *
 */
STATUS BMP_Set_FIFO_Watermark(BmpInstance *pMyBmp, uint16_t fwtm)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else if (fwtm > 511)
	{
		result = ERR_DATA_OVERSIZE;
	}
	else
	{
		result = BMP_Write16(pMyBmp->I2ch, REG_FIFO_WTM, fwtm);
		if (result == STATUS_OK)
		{
			pMyBmp->FIFOConf.fifo_wtm = fwtm;
		}
		else
		{
			pMyBmp->FIFOConf.fifo_wtm = 0;
		}
	}

	return result;
}

/**
 * @brief Get interface configuration.
 *
 */
STATUS BMP_Get_Iface_Conf(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;

		result = BMP_Read8(pMyBmp->I2ch, REG_IF_CONF, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->IfaceConf.spi_mode = (0b00000001 & tmp);
			pMyBmp->IfaceConf.i2c_wdt_en = (0b00000010 & tmp) >> 1;
			pMyBmp->IfaceConf.i2c_wdt_sel = (0b00000100 & tmp) >> 2;
		}
		else
		{
			pMyBmp->IfaceConf.spi_mode = 0;
			pMyBmp->IfaceConf.i2c_wdt_sel = 0;
			pMyBmp->IfaceConf.i2c_wdt_en = 0;
		}
	}

	return result;
}

/**
 * @brief Set interface configuration.
 *
 */
STATUS BMP_Set_Iface_Conf(BmpInstance *pMyBmp, InterfaceConfig *iface_config)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;
		tmp = 0x00 | (iface_config->i2c_wdt_sel << 2) | (iface_config->i2c_wdt_en << 1) | iface_config->spi_mode;

		result = BMP_Write8(pMyBmp->I2ch, REG_IF_CONF, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->IfaceConf.spi_mode = iface_config->spi_mode;
			pMyBmp->IfaceConf.i2c_wdt_en = iface_config->i2c_wdt_en;
			pMyBmp->IfaceConf.i2c_wdt_sel = iface_config->i2c_wdt_sel;
		}
		else
		{
			pMyBmp->IfaceConf.spi_mode = 0;
			pMyBmp->IfaceConf.i2c_wdt_sel = 0;
			pMyBmp->IfaceConf.i2c_wdt_en = 0;
		}
	}

	return result;
}

/**
 * @brief Get interrupt configuration.
 *
 */
STATUS BMP_Get_Int_Ctrl(BmpInstance *pMyBmp)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;

		result = BMP_Read8(pMyBmp->I2ch, REG_INT_CTRL, &tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->IntConf.drdy_en = (0b01000000 & tmp) >> 6;
			pMyBmp->IntConf.ffull_en = (0b00010000 & tmp) >> 4;
			pMyBmp->IntConf.fwtm_en = (0b00001000 & tmp) >> 3;
			pMyBmp->IntConf.int_latch = (0b00000100 & tmp) >> 2;
			pMyBmp->IntConf.int_level = (0b00000010 & tmp) >> 1;
			pMyBmp->IntConf.output_type = (0b00000001 & tmp);
		}
		else
		{
			pMyBmp->IntConf.drdy_en = 0;
			pMyBmp->IntConf.ffull_en = 0;
			pMyBmp->IntConf.fwtm_en = 0;
			pMyBmp->IntConf.int_latch = 0;
			pMyBmp->IntConf.int_level = 0;
			pMyBmp->IntConf.output_type = 0;
		}
	}

	return result;
}

/**
 * @brief Set interrupt configuration.
 *
 */
STATUS BMP_Set_Int_Ctrl(BmpInstance *pMyBmp, InterruptControl *IntCtrl)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;
		tmp = (0x00 | IntCtrl->output_type | (IntCtrl->int_level << 1) | (IntCtrl->int_latch << 2) |
			   (IntCtrl->fwtm_en << 3) | (IntCtrl->ffull_en << 4) | (IntCtrl->drdy_en << 6));

		result = BMP_Write8(pMyBmp->I2ch, REG_INT_CTRL, tmp);
		if (result == STATUS_OK)
		{
			pMyBmp->IntConf.drdy_en = IntCtrl->drdy_en;
			pMyBmp->IntConf.ffull_en = IntCtrl->ffull_en;
			pMyBmp->IntConf.fwtm_en = IntCtrl->fwtm_en;
			pMyBmp->IntConf.int_latch = IntCtrl->int_latch;
			pMyBmp->IntConf.int_level = IntCtrl->int_level;
			pMyBmp->IntConf.output_type = IntCtrl->output_type;
		}
		else
		{
			pMyBmp->IntConf.drdy_en = 0;
			pMyBmp->IntConf.ffull_en = 0;
			pMyBmp->IntConf.fwtm_en = 0;
			pMyBmp->IntConf.int_latch = 0;
			pMyBmp->IntConf.int_level = 0;
			pMyBmp->IntConf.output_type = 0;
		}
	}

	return result;
}

/**
 * @brief Sensor command function.
 *
 */
STATUS BMP_Write_Cmd(BmpInstance *pMyBmp, CMD command)
{
	STATUS result = STATUS_OK;

	if (pMyBmp == NULL)
	{
		result = ERR_INIT;
	}
	else
	{
		uint8_t tmp;

		tmp = command;

		result = BMP_Write8(pMyBmp->I2ch, REG_INT_CTRL, tmp);
	}

	return result;
}
