
#include "bmp_i2c.h"

/**
 * \ingroup bmp388
 * \defgroup bmp388InterfFunc Interface functions.
 * @brief I2C interface functions.
 */

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_Read_8 BMP_Read8
 * \code
 * STATUS BMP_Read8(I2C_HandleTypeDef *I2ch, uint8_t address, uint8_t *pdata);
 * \endcode
 * @brief This function read 8-bit data from single device register.
 *
 * @param[in]  I2ch I2C interface
 * @param[in] address Register address
 * @param[in, out] pdata Pointer to data read from register
 *
 * @return STATUS - Error status
 */
STATUS BMP_Read8(I2C_HandleTypeDef *I2ch, uint8_t address, uint8_t *pdata)
{
	STATUS ret_val = STATUS_OK;
	*pdata = 0;
	uint8_t tmp;

    if(HAL_I2C_Mem_Read(I2ch, BMP388_I2C_ADDR, address, 1, &tmp, 1, 10))
    {
    	ret_val = ERR_I2C;
    }else
    {
    	*pdata = tmp;
    }

    return ret_val;
}

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_Read_16 BMP_Read16
 * \code
 * STATUS BMP_Read16(I2C_HandleTypeDef *I2ch, uint8_t address, uint16_t *data);
 * \endcode
 * @brief This function read and parse 16-bit data from single device register. Since because data is
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value.
 *
 * @param[in]  I2ch I2C interface
 * @param[in] address Register address
 * @param[in, out] pdata Pointer to data read from register
 * @return STATUS - Error status
 */
STATUS BMP_Read16(I2C_HandleTypeDef *I2ch, uint8_t address, uint16_t *pdata)
{
	STATUS ret_val = STATUS_OK;
	*pdata = 0;
    uint8_t tmp[2];

    if(HAL_I2C_Mem_Read(I2ch, BMP388_I2C_ADDR, address, 1, tmp, 2, 10))
    {
    	ret_val = ERR_I2C;
    }
    else
    {
    	*pdata = (tmp[1] << 8 | tmp[0]);
    }

    return ret_val;
}

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_Read_24 BMP_Read24
 * \code
 * STATUS BMP_Read24(I2C_HandleTypeDef *I2ch, uint8_t address, uint32_t *pressure, uint32_t *temperature);
 * \endcode
 * @brief This function read and parse 24-bit data from single device register. Since data is
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value which is stored as 32-bit unsigneg integer. Beacuse
 * burst read is recommended, function read pressure and temperature registers at the same time.
 *
 * @param[in]  I2ch I2C interface
 * @param[in] address Register address
 * @param[in] pressure Pointer to variable storing readed raw pressure
 * @param[in] temperature  Pointer to variable storing readed raw temperature
 *
 * @return STATUS - Error status
 */
STATUS BMP_Read24(I2C_HandleTypeDef *I2ch, uint8_t address, uint32_t *pressure, uint32_t *temperature)
{
	STATUS ret_val = STATUS_OK;
    uint8_t tmp[6];

    if(HAL_I2C_Mem_Read(I2ch, BMP388_I2C_ADDR, address, 1, tmp, 6, 10))
    {
    	ret_val = ERR_I2C;
    }
    else
    {
    	*pressure = (tmp[2] << 16 | tmp[1] << 8 | tmp[0]);
    	*temperature = (tmp[5] << 16 | tmp[4] << 8 | tmp[3]);
    }

    return ret_val;
}

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_Write8 BMP_Write8
 * \code
 * STATUS BMP_Write8(I2C_HandleTypeDef *I2ch, uint8_t address, uint8_t data);
 * \endcode
 * @brief This function write 8-bit data to desired register. It's used for device control pusposes.
 *
 * @param[in]  I2ch I2C interface
 * @param[in] address Register address
 * @param[in] data Data to be write to register
 *
 * @return STATUS - Error status
 */
STATUS BMP_Write8(I2C_HandleTypeDef *I2ch, uint8_t address, uint8_t data)
{
	STATUS ret_val = STATUS_OK;
    uint8_t tmp = data;

    if(HAL_I2C_Mem_Write(I2ch, BMP388_I2C_ADDR, address, 1, &tmp, 1, 10))
    {
    	ret_val = ERR_I2C;
    }

    return ret_val;
}

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_Write16 BMP_Write16
 * \code
 * STATUS BMP_Write16(I2C_HandleTypeDef *I2ch, uint8_t address, uint16_t data);
 * \endcode
 * @brief This function write 16-bit data to desired register. It's used for device control pusposes.
 *
 * @param[in]  I2ch I2C interface
 * @param[in] address Register address
 * @param[in] data Data to be write to register
 *
 * @return STATUS - Error status
 */
STATUS BMP_Write16(I2C_HandleTypeDef *I2ch, uint8_t address, uint16_t data)
{
	STATUS ret_val = STATUS_OK;
    uint16_t tmp = data;

    if(HAL_I2C_Mem_Write(I2ch, BMP388_I2C_ADDR, address, 2, (uint8_t *)&tmp, 2, 10))
    {
    	ret_val = ERR_I2C;
    }

    return ret_val;
}

