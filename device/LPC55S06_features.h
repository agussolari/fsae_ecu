/*
** ###################################################################
**     Version:             rev. 1.0, 2020-04-09
**     Build:               b231016
**
**     Abstract:
**         Chip specific module features.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2023 NXP
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2020-04-09)
**         Initial version based on Niobe4mini
**
** ###################################################################
*/

#ifndef _LPC55S06_FEATURES_H_
#define _LPC55S06_FEATURES_H_

/* SOC module features */

#if defined(CPU_LPC55S06JBD64)
    /* @brief LPC_CAN availability on the SoC. */
    #define FSL_FEATURE_SOC_LPC_CAN_COUNT (1)
    /* @brief CASPER availability on the SoC. */
    #define FSL_FEATURE_SOC_CASPER_COUNT (1)
    /* @brief CDOG availability on the SoC. */
    #define FSL_FEATURE_SOC_CDOG_COUNT (1)
    /* @brief CRC availability on the SoC. */
    #define FSL_FEATURE_SOC_CRC_COUNT (1)
    /* @brief CTIMER availability on the SoC. */
    #define FSL_FEATURE_SOC_CTIMER_COUNT (5)
    /* @brief DMA availability on the SoC. */
    #define FSL_FEATURE_SOC_DMA_COUNT (2)
    /* @brief FLASH availability on the SoC. */
    #define FSL_FEATURE_SOC_FLASH_COUNT (1)
    /* @brief FLEXCOMM availability on the SoC. */
    #define FSL_FEATURE_SOC_FLEXCOMM_COUNT (9)
    /* @brief GINT availability on the SoC. */
    #define FSL_FEATURE_SOC_GINT_COUNT (2)
    /* @brief GPIO availability on the SoC. */
    #define FSL_FEATURE_SOC_GPIO_COUNT (1)
    /* @brief SECGPIO availability on the SoC. */
    #define FSL_FEATURE_SOC_SECGPIO_COUNT (1)
    /* @brief HASHCRYPT availability on the SoC. */
    #define FSL_FEATURE_SOC_HASHCRYPT_COUNT (1)
    /* @brief I2C availability on the SoC. */
    #define FSL_FEATURE_SOC_I2C_COUNT (8)
    /* @brief I2S availability on the SoC. */
    #define FSL_FEATURE_SOC_I2S_COUNT (8)
    /* @brief INPUTMUX availability on the SoC. */
    #define FSL_FEATURE_SOC_INPUTMUX_COUNT (1)
    /* @brief IOCON availability on the SoC. */
    #define FSL_FEATURE_SOC_IOCON_COUNT (1)
    /* @brief LPADC availability on the SoC. */
    #define FSL_FEATURE_SOC_LPADC_COUNT (1)
    /* @brief MPU availability on the SoC. */
    #define FSL_FEATURE_SOC_MPU_COUNT (1)
    /* @brief MRT availability on the SoC. */
    #define FSL_FEATURE_SOC_MRT_COUNT (1)
    /* @brief OSTIMER availability on the SoC. */
    #define FSL_FEATURE_SOC_OSTIMER_COUNT (1)
    /* @brief PINT availability on the SoC. */
    #define FSL_FEATURE_SOC_PINT_COUNT (1)
    /* @brief SECPINT availability on the SoC. */
    #define FSL_FEATURE_SOC_SECPINT_COUNT (1)
    /* @brief PMC availability on the SoC. */
    #define FSL_FEATURE_SOC_PMC_COUNT (1)
    /* @brief PUF availability on the SoC. */
    #define FSL_FEATURE_SOC_PUF_COUNT (1)
    /* @brief PUF_SRAM_CTRL availability on the SoC. */
    #define FSL_FEATURE_SOC_PUF_SRAM_CTRL_COUNT (1)
    /* @brief LPC_RNG1 availability on the SoC. */
    #define FSL_FEATURE_SOC_LPC_RNG1_COUNT (1)
    /* @brief RTC availability on the SoC. */
    #define FSL_FEATURE_SOC_RTC_COUNT (1)
    /* @brief SCT availability on the SoC. */
    #define FSL_FEATURE_SOC_SCT_COUNT (1)
    /* @brief SPI availability on the SoC. */
    #define FSL_FEATURE_SOC_SPI_COUNT (7)
    /* @brief SYSCON availability on the SoC. */
    #define FSL_FEATURE_SOC_SYSCON_COUNT (1)
    /* @brief SYSCTL1 availability on the SoC. */
    #define FSL_FEATURE_SOC_SYSCTL1_COUNT (1)
    /* @brief USART availability on the SoC. */
    #define FSL_FEATURE_SOC_USART_COUNT (8)
    /* @brief UTICK availability on the SoC. */
    #define FSL_FEATURE_SOC_UTICK_COUNT (1)
    /* @brief WWDT availability on the SoC. */
    #define FSL_FEATURE_SOC_WWDT_COUNT (1)
#elif defined(CPU_LPC55S06JHI48)
    /* @brief LPC_CAN availability on the SoC. */
    #define FSL_FEATURE_SOC_LPC_CAN_COUNT (1)
    /* @brief CASPER availability on the SoC. */
    #define FSL_FEATURE_SOC_CASPER_COUNT (1)
    /* @brief CDOG availability on the SoC. */
    #define FSL_FEATURE_SOC_CDOG_COUNT (1)
    /* @brief CRC availability on the SoC. */
    #define FSL_FEATURE_SOC_CRC_COUNT (1)
    /* @brief CTIMER availability on the SoC. */
    #define FSL_FEATURE_SOC_CTIMER_COUNT (5)
    /* @brief DMA availability on the SoC. */
    #define FSL_FEATURE_SOC_DMA_COUNT (2)
    /* @brief FLASH availability on the SoC. */
    #define FSL_FEATURE_SOC_FLASH_COUNT (1)
    /* @brief FLEXCOMM availability on the SoC. */
    #define FSL_FEATURE_SOC_FLEXCOMM_COUNT (8)
    /* @brief GINT availability on the SoC. */
    #define FSL_FEATURE_SOC_GINT_COUNT (2)
    /* @brief GPIO availability on the SoC. */
    #define FSL_FEATURE_SOC_GPIO_COUNT (1)
    /* @brief SECGPIO availability on the SoC. */
    #define FSL_FEATURE_SOC_SECGPIO_COUNT (1)
    /* @brief HASHCRYPT availability on the SoC. */
    #define FSL_FEATURE_SOC_HASHCRYPT_COUNT (1)
    /* @brief I2C availability on the SoC. */
    #define FSL_FEATURE_SOC_I2C_COUNT (7)
    /* @brief I2S availability on the SoC. */
    #define FSL_FEATURE_SOC_I2S_COUNT (4)
    /* @brief INPUTMUX availability on the SoC. */
    #define FSL_FEATURE_SOC_INPUTMUX_COUNT (1)
    /* @brief IOCON availability on the SoC. */
    #define FSL_FEATURE_SOC_IOCON_COUNT (1)
    /* @brief LPADC availability on the SoC. */
    #define FSL_FEATURE_SOC_LPADC_COUNT (1)
    /* @brief MPU availability on the SoC. */
    #define FSL_FEATURE_SOC_MPU_COUNT (1)
    /* @brief MRT availability on the SoC. */
    #define FSL_FEATURE_SOC_MRT_COUNT (1)
    /* @brief OSTIMER availability on the SoC. */
    #define FSL_FEATURE_SOC_OSTIMER_COUNT (1)
    /* @brief PINT availability on the SoC. */
    #define FSL_FEATURE_SOC_PINT_COUNT (1)
    /* @brief SECPINT availability on the SoC. */
    #define FSL_FEATURE_SOC_SECPINT_COUNT (1)
    /* @brief PMC availability on the SoC. */
    #define FSL_FEATURE_SOC_PMC_COUNT (1)
    /* @brief PUF availability on the SoC. */
    #define FSL_FEATURE_SOC_PUF_COUNT (1)
    /* @brief PUF_SRAM_CTRL availability on the SoC. */
    #define FSL_FEATURE_SOC_PUF_SRAM_CTRL_COUNT (1)
    /* @brief LPC_RNG1 availability on the SoC. */
    #define FSL_FEATURE_SOC_LPC_RNG1_COUNT (1)
    /* @brief RTC availability on the SoC. */
    #define FSL_FEATURE_SOC_RTC_COUNT (1)
    /* @brief SCT availability on the SoC. */
    #define FSL_FEATURE_SOC_SCT_COUNT (1)
    /* @brief SPI availability on the SoC. */
    #define FSL_FEATURE_SOC_SPI_COUNT (4)
    /* @brief SYSCON availability on the SoC. */
    #define FSL_FEATURE_SOC_SYSCON_COUNT (1)
    /* @brief SYSCTL1 availability on the SoC. */
    #define FSL_FEATURE_SOC_SYSCTL1_COUNT (1)
    /* @brief USART availability on the SoC. */
    #define FSL_FEATURE_SOC_USART_COUNT (7)
    /* @brief UTICK availability on the SoC. */
    #define FSL_FEATURE_SOC_UTICK_COUNT (1)
    /* @brief WWDT availability on the SoC. */
    #define FSL_FEATURE_SOC_WWDT_COUNT (1)
#endif

/* LPADC module features */

/* @brief FIFO availability on the SoC. */
#define FSL_FEATURE_LPADC_FIFO_COUNT (2)
/* @brief Has subsequent trigger priority (bitfield CFG[TPRICTRL]). */
#define FSL_FEATURE_LPADC_HAS_CFG_SUBSEQUENT_PRIORITY (1)
/* @brief Has differential mode (bitfield CMDLn[DIFF]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_DIFF (0)
/* @brief Has channel scale (bitfield CMDLn[CSCALE]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_CSCALE (0)
/* @brief Has conversion type select (bitfield CMDLn[CTYPE]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_CTYPE (1)
/* @brief Has conversion resolution select  (bitfield CMDLn[MODE]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_MODE (1)
/* @brief Has compare function enable (bitfield CMDHn[CMPEN]). */
#define FSL_FEATURE_LPADC_HAS_CMDH_CMPEN (1)
/* @brief Has Wait for trigger assertion before execution (bitfield CMDHn[WAIT_TRIG]). */
#define FSL_FEATURE_LPADC_HAS_CMDH_WAIT_TRIG (1)
/* @brief Has offset calibration (bitfield CTRL[CALOFS]). */
#define FSL_FEATURE_LPADC_HAS_CTRL_CALOFS (1)
/* @brief Has gain calibration (bitfield CTRL[CAL_REQ]). */
#define FSL_FEATURE_LPADC_HAS_CTRL_CAL_REQ (1)
/* @brief Has calibration average (bitfield CTRL[CAL_AVGS]). */
#define FSL_FEATURE_LPADC_HAS_CTRL_CAL_AVGS (1)
/* @brief Has internal clock (bitfield CFG[ADCKEN]). */
#define FSL_FEATURE_LPADC_HAS_CFG_ADCKEN (0)
/* @brief Enable support for low voltage reference on option 1 reference (bitfield CFG[VREF1RNG]). */
#define FSL_FEATURE_LPADC_HAS_CFG_VREF1RNG (0)
/* @brief Has calibration (bitfield CFG[CALOFS]). */
#define FSL_FEATURE_LPADC_HAS_CFG_CALOFS (0)
/* @brief Has offset trim (register OFSTRIM). */
#define FSL_FEATURE_LPADC_HAS_OFSTRIM (1)
/* @brief OFSTRIM availability on the SoC. */
#define FSL_FEATURE_LPADC_OFSTRIM_COUNT (2)
/* @brief Has Trigger status register. */
#define FSL_FEATURE_LPADC_HAS_TSTAT (1)
/* @brief Has power select (bitfield CFG[PWRSEL]). */
#define FSL_FEATURE_LPADC_HAS_CFG_PWRSEL (1)
/* @brief Has alternate channel B scale (bitfield CMDLn[ALTB_CSCALE]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_ALTB_CSCALE (0)
/* @brief Has alternate channel B select enable (bitfield CMDLn[ALTBEN]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_ALTBEN (0)
/* @brief Has alternate channel input (bitfield CMDLn[ALTB_ADCH]). */
#define FSL_FEATURE_LPADC_HAS_CMDL_ALTB_ADCH (0)
/* @brief Has offset calibration mode (bitfield CTRL[CALOFSMODE]). */
#define FSL_FEATURE_LPADC_HAS_CTRL_CALOFSMODE (0)
/* @brief Conversion averaged bitfiled width. */
#define FSL_FEATURE_LPADC_CONVERSIONS_AVERAGED_BITFIELD_WIDTH (3)
/* @brief Has B side channels. */
#define FSL_FEATURE_LPADC_HAS_B_SIDE_CHANNELS (1)
/* @brief Indicate whether the LPADC STAT register has trigger exception interrupt function (bitfield STAT[TEXC_INT]). */
#define FSL_FEATURE_LPADC_HAS_STAT_TEXC_INT (1)
/* @brief Indicate whether the LPADC STAT register has trigger completion interrupt function (bitfield STAT[TCOMP_INT]). */
#define FSL_FEATURE_LPADC_HAS_STAT_TCOMP_INT (1)
/* @brief Indicate whether the LPADC STAT register has calibration ready function (bitfield STAT[CAL_RDY]). */
#define FSL_FEATURE_LPADC_HAS_STAT_CAL_RDY (1)
/* @brief Indicate whether the LPADC STAT register has ADC active function (bitfield STAT[ADC_ACTIVE]). */
#define FSL_FEATURE_LPADC_HAS_STAT_ADC_ACTIVE (1)
/* @brief Indicate whether the LPADC IE register has trigger exception interrupt enable function (bitfield IE[TEXC_IE]). */
#define FSL_FEATURE_LPADC_HAS_IE_TEXC_IE (1)
/* @brief Indicate whether the LPADC IE register has trigger completion interrupt enable function (bitfield IE[TCOMP_IE]). */
#define FSL_FEATURE_LPADC_HAS_IE_TCOMP_IE (1)
/* @brief Indicate whether the LPADC CFG register has trigger resume/restart enable function (bitfield CFG[TRES]). */
#define FSL_FEATURE_LPADC_HAS_CFG_TRES (1)
/* @brief Indicate whether the LPADC CFG register has trigger command resume/restart enable function (bitfield CFG[TCMDRES]). */
#define FSL_FEATURE_LPADC_HAS_CFG_TCMDRES (1)
/* @brief Indicate whether the LPADC CFG register has high priority trigger exception disable function (bitfield CFG[HPT_EXDI]). */
#define FSL_FEATURE_LPADC_HAS_CFG_HPT_EXDI (1)
/* @brief Indicate LPADC CFG register TPRICTRL bitfield width. */
#define FSL_FEATURE_LPADC_CFG_TPRICTRL_BITFIELD_WIDTH (2)
/* @brief Has internal temperature sensor. */
#define FSL_FEATURE_LPADC_HAS_INTERNAL_TEMP_SENSOR (1)
/* @brief Temperature sensor parameter A (slope). */
#define FSL_FEATURE_LPADC_TEMP_PARAMETER_A (799.0f)
/* @brief Temperature sensor parameter B (offset). */
#define FSL_FEATURE_LPADC_TEMP_PARAMETER_B (280.0f)
/* @brief Temperature sensor parameter Alpha. */
#define FSL_FEATURE_LPADC_TEMP_PARAMETER_ALPHA (8.5f)
/* @brief Temperature sensor need calibration. */
#define FSL_FEATURE_LPADC_TEMP_NEED_CALIBRATION (1)
/* @brief the address of temperature sensor parameter A (slope) in Flash. */
#define FSL_FEATURE_FLASH_NMPA_TEMP_SLOPE_ADDRS (0x3FD28U)
/* @brief the address of temperature sensor parameter B (offset) in Flash. */
#define FSL_FEATURE_FLASH_NMPA_TEMP_OFFSET_ADDRS (0x3FD2CU)
/* @brief the buffer size of temperature sensor. */
#define FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE (2U)

/* ANALOGCTRL module features */

/* @brief Has PLL_USB_OUT_BIT_FIELD bitfile in XO32M_CTRL reigster. */
#define FSL_FEATURE_ANACTRL_HAS_NO_ENABLE_PLL_USB_OUT_BIT_FIELD (1)
/* @brief Has XO32M_ADC_CLK_MODE bitfile in DUMMY_CTRL reigster. */
#define FSL_FEATURE_ANACTRL_HAS_XO32M_ADC_CLK_MODE_BIF_FIELD (1)
/* @brief Has auxiliary bias(register AUX_BIAS). */
#define FSL_FEATURE_ANACTRL_HAS_AUX_BIAS_REG (1)

/* CAN module features */

/* @brief Support CANFD or not */
#define FSL_FEATURE_CAN_SUPPORT_CANFD (1)

/* CASPER module features */

/* @brief Base address of the CASPER dedicated RAM */
#define FSL_FEATURE_CASPER_RAM_BASE_ADDRESS (0x04000000)
/* @brief HW interleaving of the CASPER dedicated RAM */
#define FSL_FEATURE_CASPER_RAM_HW_INTERLEAVE (1)

/* CDOG module features */

/* No feature definitions */

/* CTIMER module features */

/* @brief CTIMER has no capture channel. */
#define FSL_FEATURE_CTIMER_HAS_NO_INPUT_CAPTURE (0)
/* @brief CTIMER has no capture 2 interrupt. */
#define FSL_FEATURE_CTIMER_HAS_NO_IR_CR2INT (0)
/* @brief CTIMER capture 3 interrupt. */
#define FSL_FEATURE_CTIMER_HAS_IR_CR3INT (1)
/* @brief Has CTIMER CCR_CAP2 (register bits CCR[CAP2RE][CAP2FE][CAP2I]. */
#define FSL_FEATURE_CTIMER_HAS_NO_CCR_CAP2 (0)
/* @brief Has CTIMER CCR_CAP3 (register bits CCR[CAP3RE][CAP3FE][CAP3I]). */
#define FSL_FEATURE_CTIMER_HAS_CCR_CAP3 (1)
/* @brief CTIMER Has register MSR */
#define FSL_FEATURE_CTIMER_HAS_MSR (1)

/* DMA module features */

/* @brief Number of channels */
#define FSL_FEATURE_DMA_NUMBER_OF_CHANNELS (23)
/* @brief Align size of DMA descriptor */
#define FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE (512)
/* @brief DMA head link descriptor table align size */
#define FSL_FEATURE_DMA_LINK_DESCRIPTOR_ALIGN_SIZE (16U)

/* FLEXCOMM module features */

#if defined(CPU_LPC55S06JBD64)
    /* @brief FLEXCOMM0 USART INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_USART_INDEX  (0)
    /* @brief FLEXCOMM0 SPI INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_SPI_INDEX  (0)
    /* @brief FLEXCOMM0 I2C INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_I2C_INDEX  (0)
    /* @brief FLEXCOMM0 I2S INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_I2S_INDEX  (0)
    /* @brief FLEXCOMM1 USART INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_USART_INDEX  (1)
    /* @brief FLEXCOMM1 SPI INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_SPI_INDEX  (1)
    /* @brief FLEXCOMM1 I2C INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_I2C_INDEX  (1)
    /* @brief FLEXCOMM1 I2S INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_I2S_INDEX  (1)
    /* @brief FLEXCOMM2 USART INDEX 2 */
    #define FSL_FEATURE_FLEXCOMM2_USART_INDEX  (2)
    /* @brief FLEXCOMM2 I2C INDEX 2 */
    #define FSL_FEATURE_FLEXCOMM2_I2C_INDEX  (2)
    /* @brief FLEXCOMM2 I2S INDEX 2 */
    #define FSL_FEATURE_FLEXCOMM2_I2S_INDEX  (2)
    /* @brief FLEXCOMM3 USART INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_USART_INDEX  (3)
    /* @brief FLEXCOMM3 SPI INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_SPI_INDEX  (3)
    /* @brief FLEXCOMM3 I2C INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_I2C_INDEX  (3)
    /* @brief FLEXCOMM3 I2S INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_I2S_INDEX  (3)
    /* @brief FLEXCOMM4 USART INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_USART_INDEX  (4)
    /* @brief FLEXCOMM4 SPI INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_SPI_INDEX  (4)
    /* @brief FLEXCOMM4 I2C INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_I2C_INDEX  (4)
    /* @brief FLEXCOMM4 I2S INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_I2S_INDEX  (4)
    /* @brief FLEXCOMM5 USART INDEX 5 */
    #define FSL_FEATURE_FLEXCOMM5_USART_INDEX  (5)
    /* @brief FLEXCOMM5 I2C INDEX 5 */
    #define FSL_FEATURE_FLEXCOMM5_I2C_INDEX  (5)
    /* @brief FLEXCOMM5 I2S INDEX 5 */
    #define FSL_FEATURE_FLEXCOMM5_I2S_INDEX  (5)
    /* @brief FLEXCOMM6 USART INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_USART_INDEX  (6)
    /* @brief FLEXCOMM6 SPI INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_SPI_INDEX  (6)
    /* @brief FLEXCOMM6 I2C INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_I2C_INDEX  (6)
    /* @brief FLEXCOMM6 I2S INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_I2S_INDEX  (6)
    /* @brief FLEXCOMM7 USART INDEX 7 */
    #define FSL_FEATURE_FLEXCOMM7_USART_INDEX  (7)
    /* @brief FLEXCOMM7 SPI INDEX 7 */
    #define FSL_FEATURE_FLEXCOMM7_SPI_INDEX  (7)
    /* @brief FLEXCOMM7 I2C INDEX 7 */
    #define FSL_FEATURE_FLEXCOMM7_I2C_INDEX  (7)
    /* @brief FLEXCOMM7 I2S INDEX 7 */
    #define FSL_FEATURE_FLEXCOMM7_I2S_INDEX  (7)
    /* @brief FLEXCOMM8 SPI(HS_SPI) INDEX 8 */
    #define FSL_FEATURE_FLEXCOMM8_SPI_INDEX  (8)
    /* @brief I2S has DMIC interconnection */
    #define FSL_FEATURE_FLEXCOMM_INSTANCE_I2S_HAS_DMIC_INTERCONNECTIONn(x) (0)
    /* @brief I2S support dual channel transfer */
    #define FSL_FEATURE_FLEXCOMM_INSTANCE_I2S_SUPPORT_SECONDARY_CHANNELn(x) \
        (((x) == FLEXCOMM0) ? (0) : \
        (((x) == FLEXCOMM1) ? (0) : \
        (((x) == FLEXCOMM2) ? (0) : \
        (((x) == FLEXCOMM3) ? (0) : \
        (((x) == FLEXCOMM4) ? (0) : \
        (((x) == FLEXCOMM5) ? (0) : \
        (((x) == FLEXCOMM6) ? (1) : \
        (((x) == FLEXCOMM7) ? (1) : \
        (((x) == FLEXCOMM8) ? (0) : (-1))))))))))
#elif defined(CPU_LPC55S06JHI48)
    /* @brief FLEXCOMM0 USART INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_USART_INDEX  (0)
    /* @brief FLEXCOMM0 SPI INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_SPI_INDEX  (0)
    /* @brief FLEXCOMM0 I2C INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_I2C_INDEX  (0)
    /* @brief FLEXCOMM0 I2S INDEX 0 */
    #define FSL_FEATURE_FLEXCOMM0_I2S_INDEX  (0)
    /* @brief FLEXCOMM1 USART INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_USART_INDEX  (1)
    /* @brief FLEXCOMM1 I2C INDEX 1 */
    #define FSL_FEATURE_FLEXCOMM1_I2C_INDEX  (1)
    /* @brief FLEXCOMM2 USART INDEX 2 */
    #define FSL_FEATURE_FLEXCOMM2_USART_INDEX  (2)
    /* @brief FLEXCOMM2 I2C INDEX 2 */
    #define FSL_FEATURE_FLEXCOMM2_I2C_INDEX  (2)
    /* @brief FLEXCOMM3 USART INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_USART_INDEX  (3)
    /* @brief FLEXCOMM3 SPI INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_SPI_INDEX  (3)
    /* @brief FLEXCOMM3 I2C INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_I2C_INDEX  (3)
    /* @brief FLEXCOMM3 I2S INDEX 3 */
    #define FSL_FEATURE_FLEXCOMM3_I2S_INDEX  (3)
    /* @brief FLEXCOMM4 USART INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_USART_INDEX  (4)
    /* @brief FLEXCOMM4 I2C INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_I2C_INDEX  (4)
    /* @brief FLEXCOMM4 I2S INDEX 4 */
    #define FSL_FEATURE_FLEXCOMM4_I2S_INDEX  (4)
    /* @brief FLEXCOMM5 USART INDEX 5 */
    #define FSL_FEATURE_FLEXCOMM5_USART_INDEX  (5)
    /* @brief FLEXCOMM5 I2C INDEX 5 */
    #define FSL_FEATURE_FLEXCOMM5_I2C_INDEX  (5)
    /* @brief FLEXCOMM6 USART INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_USART_INDEX  (6)
    /* @brief FLEXCOMM6 SPI INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_SPI_INDEX  (6)
    /* @brief FLEXCOMM6 I2C INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_I2C_INDEX  (6)
    /* @brief FLEXCOMM6 I2S INDEX 6 */
    #define FSL_FEATURE_FLEXCOMM6_I2S_INDEX  (6)
    /* @brief FLEXCOMM8 SPI(HS_SPI) INDEX 8 */
    #define FSL_FEATURE_FLEXCOMM8_SPI_INDEX  (8)
    /* @brief I2S has DMIC interconnection */
    #define FSL_FEATURE_FLEXCOMM_INSTANCE_I2S_HAS_DMIC_INTERCONNECTIONn(x) (0)
    /* @brief I2S support dual channel transfer */
    #define FSL_FEATURE_FLEXCOMM_INSTANCE_I2S_SUPPORT_SECONDARY_CHANNELn(x) \
        (((x) == FLEXCOMM0) ? (0) : \
        (((x) == FLEXCOMM1) ? (0) : \
        (((x) == FLEXCOMM2) ? (0) : \
        (((x) == FLEXCOMM3) ? (0) : \
        (((x) == FLEXCOMM4) ? (0) : \
        (((x) == FLEXCOMM5) ? (0) : \
        (((x) == FLEXCOMM6) ? (1) : \
        (((x) == FLEXCOMM8) ? (0) : (-1)))))))))
#endif /* defined(CPU_LPC55S06JBD64) */

/* GINT module features */

/* @brief The count of th port which are supported in GINT. */
#define FSL_FEATURE_GINT_PORT_COUNT (2)

/* HASHCRYPT module features */

/* @brief the address of alias offset */
#define FSL_FEATURE_HASHCRYPT_ALIAS_OFFSET (0x00000000)
/* @brief hashcrypt has reload feature */
#define FSL_FEATURE_HASHCRYPT_HAS_RELOAD_FEATURE (1)

/* I2S module features */

/* @brief I2S6 and I2S7 support dual channel transfer. */
#define FSL_FEATURE_I2S_SUPPORT_SECONDARY_CHANNEL (1)
/* @brief I2S has DMIC interconnection */
#define FSL_FEATURE_FLEXCOMM_I2S_HAS_DMIC_INTERCONNECTION (0)

/* INPUTMUX module features */

/* @brief Inputmux has DMA Request Enable */
#define FSL_FEATURE_INPUTMUX_HAS_SIGNAL_ENA (0)
/* @brief Inputmux has channel mux control */
#define FSL_FEATURE_INPUTMUX_HAS_CHANNEL_MUX (0)

/* IOCON module features */

/* @brief Func bit field width */
#define FSL_FEATURE_IOCON_FUNC_FIELD_WIDTH (4)

/* MRT module features */

/* @brief number of channels. */
#define FSL_FEATURE_MRT_NUMBER_OF_CHANNELS  (4)

/* PINT module features */

/* @brief Number of connected outputs */
#define FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS (8)

/* PLU module features */

/* @brief Has WAKEINT_CTRL register. */
#define FSL_FEATURE_PLU_HAS_WAKEINT_CTRL_REG (1)

/* PMC module features */

/* @brief UTICK does not support PD configure. */
#define FSL_FEATURE_UTICK_HAS_NO_PDCFG (1)
/* @brief WDT OSC does not support PD configure. */
#define FSL_FEATURE_WWDT_HAS_NO_PDCFG (1)

/* POWERLIB module features */

/* @brief Powerlib API is different with other LPC series devices. */
#define FSL_FEATURE_POWERLIB_EXTEND (1)

/* PUF module features */

/* @brief Number of PUF key slots available on device. */
#define FSL_FEATURE_PUF_HAS_KEYSLOTS (4)
/* @brief the shift status value */
#define FSL_FEATURE_PUF_HAS_SHIFT_STATUS (1)
/* @brief PUF has dedicated SRAM control */
#define FSL_FEATURE_PUF_HAS_SRAM_CTRL (1)
/* @brief Puf Activation Code Address. */
#define FSL_FEATURE_PUF_ACTIVATION_CODE_ADDRESS (255488)
/* @brief Puf Activation Code Size. */
#define FSL_FEATURE_PUF_ACTIVATION_CODE_SIZE (1192)

/* RTC module features */

/* @brief Has SUBSEC Register (register SUBSEC) */
#define FSL_FEATURE_RTC_HAS_SUBSEC (1)

/* SCT module features */

/* @brief Number of events */
#define FSL_FEATURE_SCT_NUMBER_OF_EVENTS (16)
/* @brief Number of states */
#define FSL_FEATURE_SCT_NUMBER_OF_STATES (32)
/* @brief Number of match capture */
#define FSL_FEATURE_SCT_NUMBER_OF_MATCH_CAPTURE (16)
/* @brief Number of outputs */
#define FSL_FEATURE_SCT_NUMBER_OF_OUTPUTS (10)

/* SECPINT module features */

/* @brief Number of connected outputs */
#define FSL_FEATURE_SECPINT_NUMBER_OF_CONNECTED_OUTPUTS (2)

/* SPI module features */

/* @brief SSEL pin count. */
#define FSL_FEATURE_SPI_SSEL_COUNT (4)

/* SYSCON module features */

/* @brief Flash page size in bytes */
#define FSL_FEATURE_SYSCON_FLASH_PAGE_SIZE_BYTES (512)
/* @brief Flash sector size in bytes */
#define FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES (32768)
/* @brief Flash size in bytes */
#define FSL_FEATURE_SYSCON_FLASH_SIZE_BYTES (249856)
/* @brief Has Power Down mode */
#define FSL_FEATURE_SYSCON_HAS_POWERDOWN_MODE (1)
/* @brief CCM_ANALOG availability on the SoC.  */
#define FSL_FEATURE_SOC_CCM_ANALOG_COUNT (1)
/* @brief Starter register discontinuous. */
#define FSL_FEATURE_SYSCON_STARTER_DISCONTINUOUS (1)

/* SYSCTL1 module features */

/* @brief SYSCTRL has Code Gray feature. */
#define FSL_FEATURE_SYSCTRL_HAS_CODE_GRAY (1)

/* WWDT module features */

/* @brief Has no RESET register. */
#define FSL_FEATURE_WWDT_HAS_NO_RESET (1)
/* @brief WWDT does not support oscillator lock. */
#define FSL_FEATURE_WWDT_HAS_NO_OSCILLATOR_LOCK (1)

#endif /* _LPC55S06_FEATURES_H_ */

