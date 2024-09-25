/**
  **********************************************************************************************
  * @file: bmp388_regs.h
  *
  * @brief: Registers definitions for BMP388
  *         This file contains registers defines and common structures.
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
#pragma once
/*! Includes */
#ifndef BMP388_REGS_H
#define BMP388_REGS_H
#include <stdbool.h>

/**
 *  @brief Device id.
 */


#define BMP_CHIP_ID 			0x50

/**
 *  @brief Registers addresses.
 */

#define REG_CHIP_ID         0x00        // Chip ID
#define REG_ERR_REG         0x02        // Errors register
#define REG_STATUS          0x03        // Conversion status flags
#define REG_EVENT           0x10        // After powerup or reset
#define REG_INT_STATUS      0x11        // Interrupt settings
#define REG_FIFO_LENGTH0    0x12        // FIFO LENGTH 0, LSB
#define REG_FIFO_LENGTH1    0x13        // FIFO LENGTH 1, MSB
#define REG_FIFO_DATA       0x14        // FIFO data
#define REG_FIFO_WTM        0x15        // FIFO watermark, 2 bytes register with 9 bits
#define REG_FIFO_CONFIG_1   0x17        // FIFO configuration 1
#define REG_FIFO_CONFIG_2   0x18        // FIFO configuration 2
#define REG_INT_CTRL        0x19        // Interrupt pin configuration
#define REG_IF_CONF         0x1A        // Interface configuration
#define REG_PWR_CNTRL       0x1B        // Power control settings
#define REG_OSR             0x1C        // Oversampling settings
#define REG_ODR             0x1D        // Output data rate
#define REG_IIR_CONFIG      0x1F        // Filter coefficient setting
#define REG_CMD             0x7E        // Command register (soft reset)

#define REG_DATA            0x04        // Data register
#define REG_PRESSURE        0x04        // Pressure data register, 3 bytes register (0x04 to 0x06 included)
#define REG_TEMPERATURE     0x07        // Temperature data register, 3 bytes register(0x07 to 0x09 included)
#define REG_TIMER           0x0C        // Sensor timer data register, 3 bytes register(0x0C to 0x0E included)

// Trimming parameters
#define REG_CRC     0x30                // CRC sum
#define REG_T1      0x31                //Temperature coefficient register 1, 16 bit
#define REG_T2      0x33                //Temperature coefficient register 2, 16 bit
#define REG_T3      0x35                //Temperature coefficient register 3, 8 bit
#define REG_P1      0x36                //Pressure coefficient register 1, 16 bit
#define REG_P2      0x38                //Pressure coefficient register 2, 16 bit                         
#define REG_P3      0x3A                //Pressure coefficient register 3, 8 bit        
#define REG_P4      0x3B                //Pressure coefficient register 4, 8 bit
#define REG_P5      0x3C                //Pressure coefficient register 5, 16 bit
#define REG_P6      0x3E                //Pressure coefficient register 6, 16 bit
#define REG_P7      0x40                //Pressure coefficient register 7, 8 bit
#define REG_P8      0x41                //Pressure coefficient register 8, 8 bit
#define REG_P9      0x42                //Pressure coefficient register 9, 16 bit
#define REG_P10     0x44                //Pressure coefficient register 10, 8 bit
#define REG_P11     0x45                //Pressure coefficient register 11, 8 bit

/**
 *  @brief Command macros.
 */

//ERR_REG - Error type, if error appeared
typedef enum{
    FATAL_ERR   = 0x01,
    CMD_ERR     = 0x02,
    CONF_ERR    = 0x04
}ERR;

//STATUS - Device status
typedef enum{
    CMD_RDY     = 0x10,
    DRDY_PRESS  = 0x20,
    DRDY_TEMP   = 0x40
}DEV_STATUS;

//EVENT - Device power up or softreset
typedef enum{
    POWER_ON     = 0x01,
    POWER_OFF    = 0x00
}EVENT_STATUS;

//INT_STATUS - Interrupt status
typedef enum{
    FWM_INT       = 0x01,           //FIFO watermark interrupt
    FFULL_INT     = 0x02,           //FIFO full interrupt
    DRDY_INT      = 0x08            //Data ready interrupt
}INT_STATUS;

//FIFO_CONFIG_1 - FIFO configuration 1
/**
 * @brief Structure with FIFO configuration 1 register values.
 * 
 */
typedef struct FIFOConfig1{
    bool enable;                       //Enable or disable FIFO mode
    bool stop;                         //Stop when FIFO is full or not
    bool time;						   //Enable or disable sensortime frame
    bool press;                        //Enable or disable FIFO to measure pressure
    bool temp;                         //Enable or disable FIFO to measure temperature
}FIFOConfig1;

//FIFO_CONFIG_2 - FIFO configuration 2
/**
 * @brief Structure with FIFO configuration 2 register values.
 * 
 */
typedef struct FIFOConfig2{
    uint8_t subsampling;                //FIFO subsampling, 3 bits register
    bool data_select;                   //FIFO select data source, filtered or unfiltered one
}FIFOConfig2;

//FIFO_CONFIG - FIFO configuration struct
/**
 * @brief Structure with FIFO configuration.
 * 
 */
typedef struct FIFOConfig{
    FIFOConfig1 Config1;                //FIFO Config 1 struct
    FIFOConfig2 Config2;                //FIFO Config 2 struct
    uint16_t fifo_wtm;                  //FIFO watermark value from 0 to 511
}FIFOConfig;

// INT_CTRL - Interrupt Configuration
/**
 * @brief Structure with interrupt configuration register values.
 * 
 */
typedef struct InterruptControl{
    bool output_type;                   //Output type. 0 if push-pull, 1 if open-drain
    bool int_level;                     //interrupt level. 0: active low, 1: active high
    bool int_latch;                     //latching of interrupts enable(1) or disable(0)
    bool fwtm_en;                       //enable or disable FIFO watermark
    bool ffull_en;                      //enable or disable interrupt if FIFO is full
    bool drdy_en;                       //enable or disable nterrupt if there is data on FIFO
}InterruptControl;

//IF_CONF - Interface configuration
/**
 * @brief Structure with Interface configuration register values.
 * 
 */
typedef struct InterfaceConfig{
    bool spi_mode;                      //Select SPI mode. 0: SPI 4-wire mode, 1: SPI 3-wire mode
    bool i2c_wdt_en;                    //Enable I2C watchdog. 
    bool i2c_wdt_sel;                   //Selet timer period of I2C watchdog.0: timeout after 1.25ms, 1: timeout after 40 ms
}InterfaceConfig;

// PWR_CNTRL - power control
/**
 * @enum MODE
 * @brief Power modes
 * 
 * @var MODE SLEEP_MODE
 * Sleep mode, 0x00.
 * @var MODE FORCED_MODE
 * Forced mode, 0x01.
 * @var MODE NORMAL_MODE
 * Normal mode, 0x03.
 */
typedef enum{
    SLEEP_MODE      = 0x00,
    FORCED_MODE     = 0x01,
    NORMAL_MODE     = 0x03
}MODE;

/**
 * @brief Structure with power control register values. 
 *        Pressure and Temperature can be enable or disable independently.
 */
typedef struct ModeCtrl{
    bool pres_en;
    bool temp_en;
    MODE select_mode;
}ModeCtrl;

//OSR - Oversampling register
/**
 * @enum OVERSAMPLING
 * @brief Oversampling settings enum
 * 
 * @var OVERSAMPLING OVERSAMPLING_1
 * Pressure: 2.64 Pa. Temp: 0.0050 C
 * @var OVERSAMPLING OVERSAMPLING_2
 * Pressure: 1.32 Pa. Temp: 0.0025 C
 * @var OVERSAMPLING OVERSAMPLING_4
 * Pressure: 0.66 Pa. Temp: 0.0012 C
 * @var OVERSAMPLING OVERSAMPLING_8
 * Pressure: 0.33 Pa. Temp: /0.0006 C
 * @var OVERSAMPLING OVERSAMPLING_16
 * Pressure: 0.17 Pa. Temp: 0.0003 C
 * @var OVERSAMPLING OVERSAMPLING_32
 * Pressure: 0.085 Pa. Temp: 0.00015 C
 */
typedef enum{
    OVERSAMPLING_1 = 0x00,                  //Pressure: 16b/2.64 Pa. Temp: 16b/0.0050 C
    OVERSAMPLING_2,                         //Pressure: 17b/1.32 Pa. Temp: 17b/0.0025 C
    OVERSAMPLING_4,                         //Pressure: 18b/0.66 Pa. Temp: 18b/0.0012 C
    OVERSAMPLING_8,                         //Pressure: 19b/0.33 Pa. Temp: 19b/0.0006 C
    OVERSAMPLING_16,                        //Pressure: 20b/0.17 Pa. Temp: 20b/0.0003 C
    OVERSAMPLING_32                         //Pressure: 21b/0.085 Pa. Temp: 21b/0.00015 C
}OVERSAMPLING;

//ODR - Output data rate register
/**
 * @enum DATA_RATE
 * @brief Output data rate values
 * 
 * @var DATA_RATE DATA_RATE_200
 * 200 Hz, 5 ms.
 * @var DATA_RATE DATA_RATE_100
 * 100 Hz, 10 ms.
 * @var DATA_RATE DATA_RATE_50
 * 50 Hz, 20 ms.
 * @var DATA_RATE DATA_RATE_25
 * 25 Hz, 40 ms. 
 * @var DATA_RATE DATA_RATE_12p5
 * 12.5 Hz, 80 ms.
 * @var DATA_RATE DATA_RATE_6p25
 * 6.25 Hz, 160 ms.
 * @var DATA_RATE DATA_RATE_3p1
 * 3.10 Hz, 320 ms.
 * @var DATA_RATE DATA_RATE_1p5
 * 1.50 Hz, 640 ms.
 * @var DATA_RATE DATA_RATE_0p78
 * 0.78 Hz, 1.280 s.
 * @var DATA_RATE DATA_RATE_0p39
 * 0.39 Hz, 2.560 s.
 * @var DATA_RATE DATA_RATE_0p2
 * 0.20 Hz, 5.120 s.
 * @var DATA_RATE DATA_RATE_0p1
 * 0.10 Hz, 10.24 s.
 * @var DATA_RATE DATA_RATE_0p05
 * 0.05 Hz, 20.48 s.
 * @var DATA_RATE DATA_RATE_0p02
 * 0.02 Hz, 40.96 s.
 * @var DATA_RATE DATA_RATE_0p01
 * 0.01 Hz, 81.92 s.
 * @var DATA_RATE DATA_RATE_0p006
 * 0.006 Hz, 163.84 s.
 * @var DATA_RATE DATA_RATE_0p003
 * 0.003 Hz, 327.68 s.
 * @var DATA_RATE DATA_RATE_0p0015
 * 0.0015 Hz, 655.36 s.
 */
typedef enum{
    DATA_RATE_200 = 0x00,                   // 200 Hz, 5 ms
    DATA_RATE_100,                          // 100 Hz, 10 ms
    DATA_RATE_50,                           // 50 Hz, 20 ms
    DATA_RATE_25,                           // 25 Hz, 40 ms
    DATA_RATE_12p5,                         // 12.5 Hz, 80 ms
    DATA_RATE_6p25,                         // 6.25 Hz, 160 ms
    DATA_RATE_3p1,                          // 3.1 Hz, 320 ms
    DATA_RATE_1p5,                          // 1.5 Hz, 640 ms
    DATA_RATE_0p78,                         // 0.78 Hz, 1.280 s
    DATA_RATE_0p39,                         // 0.39 Hz, 2.560 s
    DATA_RATE_0p2,                          // 0.20 Hz, 5.120 s
    DATA_RATE_0p1,                          // 0.10 Hz, 10.24 s
    DATA_RATE_0p05,                         // 0.05 Hz, 20.48 s
    DATA_RATE_0p02,                         // 0.02 Hz, 40.96 s
    DATA_RATE_0p01,                         // 0.01 Hz, 81.92 s
    DATA_RATE_0p006,                        // 0.006 Hz, 163.84 s
    DATA_RATE_0p003,                        // 0.003 Hz, 327.68 s
    DATA_RATE_0p0015                        // 0.0015 Hz, 655.36 s
}DATA_RATE;

//IIR_FILTER - IIR filter
/**
 * @enum IIR_FILTER
 * @brief IIR filter values
 * 
 * @var IIR_FILTER FILTER_0
 * 0
 * @var IIR_FILTER FILTER_1
 * 1
 * @var IIR_FILTER FILTER_3
 * 3
 * @var IIR_FILTER FILTER_7
 * 7
 * @var IIR_FILTER FILTER_15
 * 15
 * @var IIR_FILTER FILTER_31
 * 31
 * @var IIR_FILTER FILTER_63
 * 63
 * @var IIR_FILTER FILTER_127
 * 127
 */
typedef enum{
    FILTER_0,
    FILTER_1,
    FILTER_3,
    FILTER_7,
    FILTER_15,
    FILTER_31,
    FILTER_63,
    FILTER_127
}IIR_FILTER;

//CMD - Command
/**
 * @enum CMD
 * @brief Available commands
 * @var CMD FIFO_FLUSH
 * Clean FIFO buffer from sensor.
 * @var CMD SOFTRESET
 * Reset sensor.
 */
typedef enum{
    FIFO_FLUSH          = 0xB0,                             //FIFO buffer clean, does not change FIFO config registers
    SOFTRESET           = 0xB6                              //Device reset, all settings overwritten with default state
}CMD;

/**
 * @brief Structure with parsed calibration data read from registers
 * 
 */
typedef struct BmpCalibrationData
{
    double  par_t1;
    double par_t2;
    double  par_t3;
    double  par_p1;
    double  par_p2;
    double  par_p3;
    double  par_p4;
    double  par_p5;
    double  par_p6;
    double  par_p7;
    double  par_p8;
    double  par_p9;
    double  par_p10;
    double  par_p11;
    /*! Calibrated temperature for pressure calculations */
    float t_comp;
} BmpCalibrationData;


/**
 * @brief Structure with sensor actual settings.
 * 
 */
typedef struct BmpInstance
{
    /*! Pointer to selected I2C handler */
    I2C_HandleTypeDef *I2ch;
    /*! Device calibration data */
    BmpCalibrationData CalData;
    /*! Power Control*/
    struct ModeCtrl PowerControl;
    /*! FIFO configuration*/
    struct FIFOConfig FIFOConf;
    /*! Interface configuration*/
    struct InterfaceConfig IfaceConf;
    /*! Interrupt configuration*/
    struct InterruptControl IntConf;
    /*! Temperature oversampling */
    OVERSAMPLING temp_oversampling;
    /*! Pressure oversampling */
    OVERSAMPLING pres_oversampling;
    /*! Selected filter coefficient */
    IIR_FILTER filter_coefficient;
    /*! Selected data rate */
    DATA_RATE data_rate;
    /*! Enable pin */
    uint16_t en_pin;
    GPIO_TypeDef *en_port;
} BmpInstance;

/**
 * @enum STATUS 
 * @brief Error status
 * 
 * @var STATUS STATUS_OK
 * No errors.
 * @var STATUS ERR_INIT
 * Initialization couldn´t be done.
 * @var STATUS ERR_WRONG_CHIP_ID
 * Wrong Chip ID.
 * @var STATUS ERR_INVALID_COEFFICIENTS
 * Invalid sensor coefficients.
 * @var STATUS ERR_SWITCH_STATE
 * Switch sensor power couldn´t be done.
 * @var STATUS ERR_I2C
 * I2C communication error.
 * @var STATUS ERR_DATA_OVERSIZE
 * Data sent oversize error.
 */
typedef enum{
    STATUS_OK                   = 0x00,                     // Function done correctly
    ERR_INIT                    = 0x01,                     // Init was not done
    ERR_WRONG_CHIP_ID           = 0x02,                     // Wrong Chip ID
    ERR_INVALID_COEFFICIENTS    = 0x03,                     // Invalid sensor coefficients
    ERR_SWITCH_STATE            = 0x04,                     // Switch sensor power error, couldn´t turn on or off
    ERR_I2C                     = 0x05,                     // I2C Communication Error
    ERR_DATA_OVERSIZE           = 0x06                      // Data sent oversize
}STATUS;

#endif
