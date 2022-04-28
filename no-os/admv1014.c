/***************************************************************************//**
 *   @file   admv1014.c
 *   @brief  Implementation of admv1014 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <malloc.h>
#include "admv1014.h"
#include "no_os_error.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * @brief Writes data to ADMV1014 over SPI.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param data - Data value to write.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int admv1014_spi_write(struct admv1014_dev *dev, uint8_t reg_addr,
		       uint16_t data)
{
	uint8_t buff[ADMV1014_BUFF_SIZE_BYTES];

	/*
	    |                Byte 0           |     Byte 1     |       Byte 2        |
	    | 0 | Addr bits 5-0 | Data bit 15 | Data bits 14-7 | Data bits 6 - 0 | 0 |
	*/
	buff[0] = ADMV1014_SPI_WRITE_CMD | (reg_addr << 1) | (data >> 15);
	buff[1] = data >> 7;
	buff[2] = data << 1;

	return no_os_spi_write_and_read(dev->spi_desc, buff,
					ADMV1014_BUFF_SIZE_BYTES);
}

/**
 * @brief Reads data from ADMV1014 over SPI.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param data - Data read from the device.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int admv1014_spi_read(struct admv1014_dev *dev, uint8_t reg_addr,
		      uint16_t *data)
{
	uint8_t buff[ADMV1014_BUFF_SIZE_BYTES];
	int ret;

	buff[0] = ADMV1014_SPI_READ_CMD | (reg_addr << 1);
	buff[1] = 0;
	buff[2] = 0;

	ret = no_os_spi_write_and_read(dev->spi_desc, buff, ADMV1014_BUFF_SIZE_BYTES);
	if(ret)
		return ret;

	/*
	    |                Byte 0           |     Byte 1     |       Byte 2        |
	    | 1 | Addr bits 5-0 | Data bit 15 | Data bits 14-7 | Data bits 6 - 0 | 0 |
	*/
	*data = ((buff[0] & NO_OS_BIT(0)) << 15 | buff[1] << 7 | buff[2] >> 1);

	return 0;
}

/**
 * @brief Update ADMV1014 register.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - Mask for specific register bits to be updated.
 * @param data - Data written to the device (requires prior bit shifting).
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int admv1014_spi_update_bits(struct admv1014_dev *dev, uint8_t reg_addr,
			     uint16_t mask, uint16_t data)
{
	uint16_t read_val;
	int ret;

	ret = admv1014_spi_read(dev, reg_addr, &read_val);
	if (ret)
		return ret;

	read_val &= ~mask;
	read_val |= data;

	return admv1014_spi_write(dev, reg_addr, read_val);
}