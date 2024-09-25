/**
  **********************************************************************************************
  * @file: bmp388.h
  *
  * @brief: Library for Bosch BMP388 header file 
  *         This file contains all definitions of methods and functions needed.
  *         Check bmp388_regs.h to see registers and defines.
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

/**
 * @defgroup bmp388 BMP388
 * @brief BMP388 API functions and methods.
 */

#ifndef BMP388_H
#define BMP388_H

/*! Includes */
#include "main.h"
#include "bmp388_regs.h"



/**
 *  @brief Device delay helper. It uses HAL_Delay() or osDelay() if FreeRTOS is used
 */
#ifdef CMSIS_OS2_H_
    #define DELAY(x)            osDelay(x/portTICK_PERIOD_MS)
#elif CMSIS_OS_H_
	#define DELAY(x)            osDelay(x/portTICK_PERIOD_MS)
#else
    #define DELAY(x)            HAL_Delay(x)
#endif

/**
 * \ingroup bmp388
 * \defgroup bmp388Init Initialization
 * @brief Sensor initialization
 */


/**
 * \ingroup bmp388Init
 * \page bmp388_api_BMP_init BMP_init
 * \code
 * BmpInstance BMP_Init(I2C_HandleTypeDef *I2ch,bool pres_r, bool temp_r, MODE mode);
 * \endcode
 * @brief API entry point. Initalize sensor with selected values and I2C bus. Perform self-test and
 * parse calibration coefficients.
 * 
 * @param[in] pMyBmp BMPx instance that has to be initialised
 * @param[in] I2ch Pointer to selected I2C handler
 * @param[in] en_pin Enable pin number
 * @param[in] en_port Enable pin port
 * @param[in] pres_r Enable or disable pressure readings
 * @param[in] temp_r Enable or disable temperature readings
 * @param[in] mode Select mode of operation
 * @param[in] osr_p Pressure oversampling
 * @param[in] osr_t Temperature oversampling
 * @param[in] iir_filter IIR filter
 * @param[in] data_rate Data rate
 *
 * @return STATUS - Indicates if initialization was correctly done or not
 */
STATUS BMP_Init(BmpInstance* pMyBmp, I2C_HandleTypeDef *I2ch, uint16_t en_pin, GPIO_TypeDef *en_port,
                bool pres_r, bool temp_r, MODE mode, OVERSAMPLING osr_p, OVERSAMPLING osr_t, IIR_FILTER iir_filter, DATA_RATE data_rate);

/**
 * \ingroup bmp388Init
 * \page bmp388_api_BMP_turn_on BMP_turn_on
 * \code
 * STATUS BMP_Turn_On(BmpInstance *pMyBmp);
 * \endcode
 * @brief Turn on sensor if it is off and initialised.
 * 
 * @param[in] pMyBmp BMPx instance that has to be initialised
 * 
 * @return STATUS - Indicates if turn on was correctly done or not
 */
STATUS BMP_Turn_On(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388Init
 * \page bmp388_api_BMP_turn_off BMP_turn_off
 * \code
 * STATUS BMP_Turn_Off(BmpInstance *pMyBmp);
 * \endcode
 * @brief Turn off sensor if it is initialised.
 * 
 * @param[in] pMyBmp BMPx instance that has to be initialised
 * 
 * @return STATUS - Indicates if turn off was correctly done or not
 */
STATUS BMP_Turn_Off(BmpInstance *pMyBmp);


/**
 * \ingroup bmp388
 * \defgroup bmp388dataMeas Measurement
 * @brief Pressure and temperature measurement functions
 */

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_Get_Data Get_Data 
 * \code
 * STATUS BMP_Get_Data(BmpInstance *pMyBmp, float *pressure, float *temperature);
 * \endcode 
 * @brief Function read and parse measured temperature and pressure values.
 *        Default operation mode is Indoor Navigation, more info at BMP380 datasheet
 * 
 * @param[in] pMyBmp BMPx instance that has to be initialised
 * @param[in out] pressure pressure value in Pa as float
 * @param[in out] temperature temperature value in C degrees as float
 * 
 * @return STATUS - Indicates if measures were taken correctly or not
 */
STATUS BMP_Get_Data(BmpInstance *pMyBmp, float *pressure, float *temperature);

/**
 * \ingroup bmp388
 * \defgroup bmp388set Settings
 * @brief Sensor settings functions
 */

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Filter_Coefficient Get_Filter_Coefficient
 * \code
 * STATUS BMP_Get_Filter_Coefficient(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get pressure IIR filter coefficient.
 * 		    IIR_Filter value is stored on BmpInstance.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates if filter was obtained correctly or not
 */
STATUS BMP_Get_Filter_Coefficient(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Filter_Coefficient Set_Filter_Coefficient
 * \code
 * STATUS Set_Filter_Coefficient(BmpInstance *pMyBmp, IIR_FILTER coeff);
 * \endcode
 * @brief Set pressure IIR filter coefficient
 * 
 * @param[in] pMyBmp BMPx instance pointer
 * @param[in] coeff Selected filter coefficient
 * 
 * @return STATUS - indicates if filter was set correctly or not
 */
STATUS BMP_Set_Filter_Coefficient(BmpInstance *pMyBmp, IIR_FILTER coeff);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Output_Data_Rate Get_Output_Data_Rate
 * \code
 * STATUS Get_Output_Data_Rate(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get sensor output data rate. Data rate is stored on BmpInstance.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates if output data rate was obtained correctly or not
 */
STATUS BMP_Get_Output_Data_Rate(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Output_Data_Rate Set_Output_Data_Rate
 * \code
 * STATUS Set_Output_Data_Rate(BmpInstance *pMyBmp, DATA_RATE data_rate);
 * \endcode
 * @brief Set sensor output data rate
 * 
 * @param[in] pMyBmp BMPx instance pointer
 * @param[in] data_rate Selected output data rate
 * 
 * @return STATUS - indicates if output data rate was set correctly or not
 */
STATUS BMP_Set_Output_Data_Rate(BmpInstance *pMyBmp, DATA_RATE data_rate);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Oversampling Get_Oversampling
 * \code
 * STATUS BMP_Get_Oversampling(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get pressure and temperature oversampling. 
 *        Values are stored on BmpInstance.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 *
 * @return STATUS - indicates if oversampling was obtained correctly or not.
 */
STATUS BMP_Get_Oversampling(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Oversampling Set_Oversampling
 * \code
 * STATUS Set_Oversampling(BmpInstance *pMyBmp, OVERSAMPLING press_osr, OVERSAMPLING temp_osr);
 * \endcode
 * @brief Set selected pressure and temperature oversampling
 * 
 * @param[in] pMyBmp BMPx instance pointer
 * @param[in] press_osr Selected pressure oversampling value
 * @param[in] temp_osr Selected temperature oversampling value
 * 
 * @return STATUS - indicates if oversampling is set correctly or not
 */
STATUS BMP_Set_Oversampling(BmpInstance *pMyBmp, OVERSAMPLING press_osr, OVERSAMPLING temp_osr);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Power_Mode Get_Power_Mode
 * \code
 * STATUS Get_Power_Mode(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get sensor power mode. Power mode is stored in BMPx instance.
 *
 * @param[in] pMyBmp BMPx instance pointer
 *
 * @return STATUS - indicates if power mode was obtained correctly or not
 */
STATUS BMP_Get_Power_Mode(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Power_Mode Set_Power_Mode
 * \code
 * STATUS Set_Power_Mode(BmpInstance *pMyBmp, MODE select_mode, bool temp_en, bool pres_en);
 * \endcode
 * @brief Set sensor power mode
 * 
 * @param[in] pMyBmp BMPx instance pointer
 * @param[in] select_mode Selected power mode
 * @param[in] temp_en Temperature measurement setting
 * @param[in] pres_en Pressure measurement setting
 * 
 * @return STATUS - indicates if power mode was set correctly or not
 */
STATUS BMP_Set_Power_Mode(BmpInstance *pMyBmp, MODE select_mode, bool temp_en, bool pres_en);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_FIFO_Config Get_FIFO_Config
 * \code
 * STATUS Get_FIFO_Config(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get FIFO configuration from sensor. Data is stored in BMPx instance.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 *
 * @return STATUS - indicates if FIFO configuration was obtained correctly or not
 */
STATUS BMP_Get_FIFO_Config(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_FIFO_Config Set_FIFO_Config
 * \code
 * STATUS Set_FIFO_Config(BmpInstance *pMyBmp, FIFOConfig1 *Config1, FIFOConfig2 *Config2);
 * \endcode
 * @brief Set sensor FIFO configuration.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * @param[in] Config1 Configuration 1 for FIFO_CONFIG_1 register.
 * @param[in] Config2 Configuration 2 for FIFO_CONFIG_2 register.
 *
 * @return STATUS - indicates if FIFO configuration was established correctly or not
 */
STATUS BMP_Set_FIFO_Config(BmpInstance *pMyBmp, FIFOConfig1 *Config1, FIFOConfig2 *Config2);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_FIFO_Watermark Get_FIFO_Watermark
 * \code
 * STATUS BMP_Get_FIFO_Watermark(BmpInstance *pMyBmp);
 * \endcode
 * 
 * @brief Get FIFO watermark. To use it, fwtm_en (fifo watermark enable) must be set to true in Int_Ctrl register.
 *        Also, if FIFO watermark is 0, it will never activate fifo watermark interrupt. Value must be set from 0 to
 *        511 (512 bytes is maximum fifo_length).
 *        FIFO watermark is stored in BMPx instance.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * 
 * @return STATUS - indicates if FIFO watermark was obtained correctly or not
 */
STATUS BMP_Get_FIFO_Watermark(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_FIFO_Watermark Set_FIFO_Watermark
 * \code
 * STATUS BMP_Set_FIFO_Watermark(BmpInstance *pMyBmp, uint16_t fwtm);
 * \endcode
 * 
 * @brief Set FIFO watermark. To use it, fwtm_en (fifo watermark enable) must be set to true in Int_Ctrl register.
 *        Also, if FIFO watermark is 0, it will never activate fifo watermark interrupt. Value must be set from 0 to
 *        511 (512 bytes is maximum fifo_length).
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * @param[in] fwtm FIFO watermark value.
 * 
 * @return STATUS - indicates if FIFO watermark was stablished correctly or not
 */
STATUS BMP_Set_FIFO_Watermark(BmpInstance *pMyBmp, uint16_t fwtm);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Iface_Conf Get_Iface_Conf
 * \code
 * STATUS BMP_Get_Iface_Conf(BmpInstance *pMyBmp);
 * \endcode
 * 
 * @brief Get interface configuration.
 *        Configuration is stored in BMPx instance.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * 
 * @return STATUS - indicates if FIFO watermark was obtained correctly or not
 */
STATUS BMP_Get_Iface_Conf(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Iface_Conf Set_Iface_Conf
 * \code
 * STATUS BMP_Set_Iface_Conf(BmpInstance *pMyBmp, InterfaceConfig *iface_config);
 * \endcode
 * 
 * @brief Set interface configuration.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * @param[in] iface_config Interface configuration pointer.
 * 
 * @return STATUS - indicates if FIFO watermark was obtained correctly or not
 */
STATUS BMP_Set_Iface_Conf(BmpInstance *pMyBmp, InterfaceConfig *iface_config);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Get_Int_Ctrl Get_Int_Ctrl
 * \code
 * STATUS BMP_Get_Int_Ctrl(BmpInstance *pMyBmp);
 * \endcode
 * @brief Get sensor interrupt configuration. 
 *        Configuration is stored in BMPx instance.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 *
 * @return STATUS - indicates if interrupt configuration was obtained correctly or not
 */
STATUS BMP_Get_Int_Ctrl(BmpInstance *pMyBmp);

/**
 * \ingroup bmp388set
 * \page bmp388_api_Set_Int_Ctrl Set_Int_Ctrl
 * \code
 * STATUS BMP_Set_Int_Ctrl(BmpInstance *pMyBmp, InterruptControl *IntCtrl);
 * \endcode
 * @brief Get sensor interrupt configuration.
 *
 * @param[in] pMyBmp BMPx instance pointer.
 * @param[in] IntCtrl Interrupt configuration struct pointer.
 * 
 * @return STATUS - indicates if interrupt configuration was obtained correctly or not
 */
STATUS BMP_Set_Int_Ctrl(BmpInstance *pMyBmp, InterruptControl *IntCtrl);

/**
 * \ingroup bmp388
 * \defgroup bmp388cmd Command
 * 
 * @brief BMP388 Commands 
 * 
 */
/**
 * \ingroup bmp388cmd
 * \page bmp388cmd_api_Cmd Write_Cmd
 * \code
 * STATUS BMP_Write_Cmd(BmpInstance *pMyBmp, CMD command);
 * \endcode
 * 
 * @brief Sensor command function
 * 
 * @param[in] pMyBmp BMPx instance pointer.
 * @param[in] command command to send.
 * 
 * @return STATUS - indicates if command was stablished correctly or not
 */
STATUS BMP_Write_Cmd(BmpInstance *pMyBmp, CMD command);

#endif
