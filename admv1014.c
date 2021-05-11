// SPDX-License-Identifier: GPL-2.0+
/*
 * ADMV1014 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

/* ADMV1014 Register Map */
#define ADMV1014_REG_SPI_CONTROL		0x00
#define ADMV1014_REG_ALARM 			0x01
#define ADMV1014_REG_ALARM_MASKS		0x02
#define ADMV1014_REG_ENABLE			0x03
#define ADMV1014_REG_QUAD			0x04
#define ADMV1014_REG_LO_AMP_PHASE_ADJUST1	0x05
#define ADMV1014_REG_MIXER   			0x07
#define ADMV1014_REG_IF_AMP  			0x08
#define ADMV1014_REG_IF_AMP_BB_AMP		0x09
#define ADMV1014_REG_BB_AMP_AGC			0x0A
#define ADMV1014_REG_VVA_TEMP_COMP		0x0B

/* ADMV1014_REG_SPI_CONTROL Map */
#define ADMV1014_PARITY_EN_MSK       		BIT(15)
#define ADMV1014_PARITY_EN(x)         		FIELD_PREP(ADMV1014_PARITY_EN_MSK, x)
#define ADMV1014_SPI_SOFT_RESET_MSK		BIT(14)
#define ADMV1014_SPI_SOFT_RESET(x)         	FIELD_PREP(ADMV1014_SPI_SOFT_RESET_MSK, x)
#define ADMV1014_CHIP_ID_MSK			GENMASK(11, 4)
#define ADMV1014_CHIP_ID(x)         		FIELD_PREP(ADMV1014_CHIP_ID_MSK, x)
#define ADMV1014_REVISION_ID_MSK		GENMASK(3, 0)
#define ADMV1014_REVISION_ID(x)        		FIELD_PREP(ADMV1014_REVISION_ID_MSK, x)

/* ADMV1014_REG_ALARM Map */
#define ADMV1014_PARITY_ERROR_MSK       	BIT(15)
#define ADMV1014_PARITY_ERROR(x)         	FIELD_PREP(ADMV1014_PARITY_ERROR_MSK, x)
#define ADMV1014_TOO_FEW_ERRORS_MSK		BIT(14)
#define ADMV1014_TOO_FEW_ERRORS(x)         	FIELD_PREP(ADMV1014_TOO_FEW_ERRORS_MSK, x)
#define ADMV1014_TOO_MANY_ERRORS_MSK		BIT(13)
#define ADMV1014_TOO_MANY_ERRORS(x)         	FIELD_PREP(ADMV1014_TOO_MANY_ERRORS_MSK, x)
#define ADMV1014_ADDRESS_RANGE_ERROR_MSK	BIT(12)
#define ADMV1014_ADDRESS_RANGE_ERROR(x)         FIELD_PREP(ADMV1014_ADDRESS_RANGE_ERROR_MSK, x)

/* ADMV1014_REG_ENABLE Map */
#define ADMV1014_IBIAS_PD_MSK       		BIT(14)
#define ADMV1014_IBIAS_PD(x)         		FIELD_PREP(ADMV1014_IBIAS_PD_MSK, x)
#define ADMV1014_P1DB_COMPENSATION_MSK		GENMASK(13, 12)
#define ADMV1014_P1DB_COMPENSATION(x)         	FIELD_PREP(ADMV1014_P1DB_COMPENSATION_MSK, x)
#define ADMV1014_IF_AMP_PD_MSK			BIT(11)
#define ADMV1014_IF_AMP_PD(x)         		FIELD_PREP(ADMV1014_IF_AMP_PD_MSK, x)
#define ADMV1014_QUAD_BG_PD_MSK			BIT(9)
#define ADMV1014_QUAD_BG_PD(x)         		FIELD_PREP(ADMV1014_QUAD_BG_PD_MSK, x)
#define ADMV1014_BB_AMP_PD_MSK			BIT(8)
#define ADMV1014_BB_AMP_PD x)         		FIELD_PREP(ADMV1014_BB_AMP_PD_MSK, x)
#define ADMV1014_QUAD_IBIAS_PD_MSK		BIT(7)
#define ADMV1014_QUAD_IBIAS_PD(x)         	FIELD_PREP(ADMV1014_QUAD_IBIAS_PD_MSK, x)
#define ADMV1014_DET_EN_MSK			BIT(6)
#define ADMV1014_DET_EN(x)         		FIELD_PREP(ADMV1014_DET_EN_MSK, x)
#define ADMV1014_BG_PD_MSK			BIT(5)
#define ADMV1014_BG_PD(x)         		FIELD_PREP(ADMV1014_BG_PD_MSK, x)

/* ADMV1014_REG_QUAD Map */
#define ADMV1014_QUAD_SE_MODE_MSK		GENMASK(9, 6)
#define ADMV1014_QUAD_SE_MODE(x)         	FIELD_PREP(ADMV1014_QUAD_SE_MODE_MSK, x)
#define ADMV1014_QUAD_FILTERS_MSK		GENMASK(3, 0)
#define ADMV1014_QUAD_FILTERS(x)         	FIELD_PREP(ADMV1014_QUAD_FILTERS_MSK, x)


/* ADMV1014_LO_AMP_PHASE_ADJUST1 Map */
#define ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK	GENMASK(15, 9)
#define ADMV1014_LOAMP_PH_ADJ_I_FINE(x)        	FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK, x)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK	GENMASK(8, 2)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE(x)        	FIELD_PREP(ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK, x)

/* ADMV1014_REG_MIXER Map */
#define ADMV1014_MIXER_VGATE_MSK		GENMASK(15, 9)
#define ADMV1014_MIXER_VGATE(x)        		FIELD_PREP(ADMV1014_MIXER_VGATE_MSK, x)
#define ADMV1014_DET_PROG_MSK			GENMASK(6, 0)
#define ADMV1014_DET_PROG(x)			FIELD_PREP(ADMV1014_DET_PROG_MSK, x)

/* ADMV1014_REG_IF_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_I_MSK	GENMASK(11, 8)
#define ADMV1014_IF_AMP_COARSE_GAIN_I(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_I_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_Q_MSK		GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_Q(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_Q_MSK, x)
#define ADMV1014_IF_AMP_FINE_GAIN_I_MSK		GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_I(x)		FIELD_PREP(ADMV1014_IF_AMP_FINE_GAIN_I_MSK, x)

/* ADMV1014_REG_IF_AMP_BB_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK	GENMASK(15, 12)
#define ADMV1014_IF_AMP_COARSE_GAIN_Q(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_Q_MSK		GENMASK(9, 5)
#define ADMV1014_BB_AMP_OFFSET_Q(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_Q_MSK, x)
#define ADMV1014_BB_AMP_OFFSET_I_MSK		GENMASK(4, 0)
#define ADMV1014_BB_AMP_OFFSET_I(x)		FIELD_PREP(ADMV1014_BB_AMP_OFFSET_I_MSK, x)

/* ADMV1014_REG_BB_AMP_AGC Map */
#define ADMV1014_BB_AMP_REF_GEN_MSK		GENMASK(6, 3)
#define ADMV1014_IF_AMP_COARSE_GAIN_Q(x)  	FIELD_PREP(ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK, x)
#define ADMV1014_BB_AMP_GAIN_CTRL_MSK		GENMASK(2, 1)
#define ADMV1014_BB_AMP_GAIN_CTRL(x)		FIELD_PREP(ADMV1014_BB_AMP_GAIN_CTRL_MSK, x)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK	BIT(0)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM(x)	FIELD_PREP(ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK, x)

/* ADMV1014_REG_VVA_TEMP_COMP Map */
#define ADMV1014_VVA_TEMP_COMP_MSK		GENMASK(15, 0)
#define ADMV1014_VVA_TEMP_COMP(x)  		FIELD_PREP(ADMV1014_VVA_TEMP_COMP_MSK, x)




