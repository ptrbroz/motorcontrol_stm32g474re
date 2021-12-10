/*
 *  Adapted from Ben's drv 8323.h july 2021 (defines changed to reflect drv8353)
 *
 *  Created on: Aug 1, 2020
 *  Author: ben
 *
 *  See the datasheet for the DRV8323 here:  https://www.ti.com/product/DRV8323
 */

#ifndef INC_DRV8353_H_
#define INC_DRV8353_H_

#include "spi.h"
#include <stdint.h>

/// Registers ///
#define FSR1            0x0     /// Fault Status Register 1
#define FSR2            0x1     /// Fault Status Register 2
#define DCR             0x2     /// Drive Control Register
#define HSR             0x3     /// Gate Drive HS Register
#define LSR             0x4     /// Gate Drive LS Register
#define OCPCR           0x5     /// OCP Control Register
#define CSACR           0x6     /// CSA Control Register

/// Drive Control Fields ///
#define DIS_CPUV_EN         0x0     /// Charge pump UVLO fault
#define DIS_CPUV_DIS        0x1
#define DIS_GDF_EN          0x0     /// Gate drive fauilt
#define DIS_GDF_DIS         0x1
#define OTW_REP_EN          0x1     /// Over temp warning reported on nFAULT/FAULT bit
#define OTW_REP_DIS         0x0
#define PWM_MODE_6X         0x0     /// PWM Input Modes
#define PWM_MODE_3X         0x1
#define PWM_MODE_1X         0x2
#define PWM_MODE_IND        0x3
#define PWM_1X_COM_SYNC     0x0     /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC    0x1
#define PWM_1X_DIR_0        0x0     /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1        0x1

/// Gate Drive HS Fields ///
#define LOCK_ON             0x6
#define LOCK_OFF            0x3
#define IDRIVEP_HS_50MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_HS_50MAb    0x1     //marked b since first two values are both listed as same val in drv8353 datasheet. Same later in file
#define IDRIVEP_HS_100MA    0x2
#define IDRIVEP_HS_150MA    0x3
#define IDRIVEP_HS_300MA    0x4
#define IDRIVEP_HS_350MA    0x5
#define IDRIVEP_HS_400MA    0x6
#define IDRIVEP_HS_450MA    0x7
#define IDRIVEP_HS_550MA    0x8
#define IDRIVEP_HS_600MA    0x9
#define IDRIVEP_HS_650MA    0xA
#define IDRIVEP_HS_700MA    0xB
#define IDRIVEP_HS_850MA    0xC
#define IDRIVEP_HS_900MA    0xD
#define IDRIVEP_HS_950MA    0xE
#define IDRIVEP_HS_1000MA   0xF
#define IDRIVEN_HS_100MA    0x0     /// High side turn off current
#define IDRIVEN_HS_100MAb   0x1
#define IDRIVEN_HS_200MA    0x2
#define IDRIVEN_HS_300MA    0x3
#define IDRIVEN_HS_600MA    0x4
#define IDRIVEN_HS_700MA    0x5
#define IDRIVEN_HS_800MA    0x6
#define IDRIVEN_HS_900MA    0x7
#define IDRIVEN_HS_1100MA   0x8
#define IDRIVEN_HS_1200MA   0x9
#define IDRIVEN_HS_1300MA   0xA
#define IDRIVEN_HS_1400MA   0xB
#define IDRIVEN_HS_1700MA   0xC
#define IDRIVEN_HS_1800MA   0xD
#define IDRIVEN_HS_1900MA   0xE
#define IDRIVEN_HS_2000MA   0xF

/// Gate Drive LS Fields ///
#define TDRIVE_500NS        0x0     /// Peak gate-current drive time
#define TDRIVE_1000NS       0x1
#define TDRIVE_2000NS       0x2
#define TDRIVE_4000NS       0x3
#define IDRIVEP_LS_50MA     0x0     /// Gate drive high side turn on current
#define IDRIVEP_LS_50MAb    0x1
#define IDRIVEP_LS_100MA    0x2
#define IDRIVEP_LS_150MA    0x3
#define IDRIVEP_LS_300MA    0x4
#define IDRIVEP_LS_350MA    0x5
#define IDRIVEP_LS_400MA    0x6
#define IDRIVEP_LS_450MA    0x7
#define IDRIVEP_LS_550MA    0x8
#define IDRIVEP_LS_600MA    0x9
#define IDRIVEP_LS_650MA    0xA
#define IDRIVEP_LS_700MA    0xB
#define IDRIVEP_LS_850MA    0xC
#define IDRIVEP_LS_900MA    0xD
#define IDRIVEP_LS_950MA    0xE
#define IDRIVEP_LS_1000MA   0xF
#define IDRIVEN_LS_100MA    0x0     /// High side turn off current
#define IDRIVEN_LS_100MAb   0x1
#define IDRIVEN_LS_200MA    0x2
#define IDRIVEN_LS_300MA    0x3
#define IDRIVEN_LS_600MA    0x4
#define IDRIVEN_LS_700MA    0x5
#define IDRIVEN_LS_800MA    0x6
#define IDRIVEN_LS_900MA    0x7
#define IDRIVEN_LS_1100MA   0x8
#define IDRIVEN_LS_1200MA   0x9
#define IDRIVEN_LS_1300MA   0xA
#define IDRIVEN_LS_1400MA   0xB
#define IDRIVEN_LS_1700MA   0xC
#define IDRIVEN_LS_1800MA   0xD
#define IDRIVEN_LS_1900MA   0xE
#define IDRIVEN_LS_2000MA   0xF

/// OCP Control Fields ///
#define TRETRY_8MS          0x0     /// VDS OCP and SEN OCP retry time
#define TRETRY_50US         0x1
#define DEADTIME_50NS       0x0     /// Deadtime
#define DEADTIME_100NS      0x1
#define DEADTIME_200NS      0x2
#define DEADTIME_400NS      0x3
#define OCP_LATCH           0x0     /// OCP Mode
#define OCP_RETRY           0x1
#define OCP_REPORT          0x2
#define OCP_NONE            0x3
#define OCP_DEG_1US         0x0     /// OCP Deglitch Time
#define OCP_DEG_2US         0x1
#define OCP_DEG_4US         0x2
#define OCP_DEG_8US         0x3
#define VDS_LVL_0_06        0x0
#define VDS_LVL_0_07        0x1
#define VDS_LVL_0_08        0x2
#define VDS_LVL_0_09        0x3
#define VDS_LVL_0_10        0x4
#define VDS_LVL_0_20        0x5
#define VDS_LVL_0_30        0x6
#define VDS_LVL_0_40        0x7
#define VDS_LVL_0_50        0x8
#define VDS_LVL_0_60        0x9
#define VDS_LVL_0_70        0xA
#define VDS_LVL_0_80        0xB
#define VDS_LVL_0_90        0xC
#define VDS_LVL_1_00        0xD
#define VDS_LVL_1_50        0xE
#define VDS_LVL_2_00        0xF

/// CSA Control Fields ///
#define CSA_FET_SP          0x0     /// Current sense amplifier positive input
#define CSA_FET_SH          0x1
#define VREF_DIV_1          0x0     /// Amplifier reference voltage is VREV/1
#define VREF_DIV_2          0x1     /// Amplifier reference voltage is VREV/2
#define CSA_GAIN_5          0x0     /// Current sensor gain
#define CSA_GAIN_10         0x1
#define CSA_GAIN_20         0x2
#define CSA_GAIN_40         0x3
#define DIS_SEN_EN          0x0     /// Overcurrent Fault
#define DIS_SEN_DIS         0x1
#define SEN_LVL_0_25        0x0     /// Sense OCP voltage level
#define SEN_LVL_0_5         0x1
#define SEN_LVL_0_75        0x2
#define SEN_LVL_1_0         0x3


typedef struct{
	union{
		uint8_t spi_tx_buff[2];
		uint16_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[2];
		uint16_t spi_rx_word;
	};
	uint16_t fault;
} DRVStruct;


uint16_t drv_spi_write(DRVStruct * drv, uint16_t val);
uint16_t drv_read_FSR1(DRVStruct drv);
uint16_t drv_read_FSR2(DRVStruct drv);
uint16_t drv_read_register(DRVStruct drv, int reg);
void drv_write_register(DRVStruct drv, int reg, int val);
void drv_write_DCR(DRVStruct drv, int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT);
void drv_write_HSR(DRVStruct drv, int LOCK, int IDRIVEP_HS, int IDRIVEN_HS);
void drv_write_LSR(DRVStruct drv, int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS);
void drv_write_OCPCR(DRVStruct drv, int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL);
void drv_write_CSACR(DRVStruct drv, int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL);
void drv_enable_gd(DRVStruct drv);
void drv_disable_gd(DRVStruct drv);
void drv_calibrate(DRVStruct drv);
void drv_print_faults(DRVStruct drv);



#endif /* INC_DRV8353_H_ */
