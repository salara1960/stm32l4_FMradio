/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdarg.h>
#include <ctype.h>

#include "hdr.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

enum {
	devOK = 0,
	devTIK = 1,
	devUART = 2,
	devMEM = 4,
	devRTC = 8,
	devEVT = 0x10,
	devSYS = 0x20,
	devSPI = 0x40,
	devLCD = 0x80,
	devRDA = 0x100,
	devFS = 0x200
#if defined(SET_BLE) || defined(SET_AUDIO)
	,
	devBLE = 0x400,
	devQUE = 0x800
#endif
};

enum {
	cmdNone = -1,
	cmdHelp = 0,
	cmdRestart,
	cmdEpoch,
	cmdErr,
	cmdsRead,
	cmdsErase,
	cmdsNext,
	cmdsWrite,
	cmdSec,
	cmdVer,
	cmdClr,
	cmdScan,
	cmdFreq,
	cmdVol,
	cmdMute,
	cmdBass,
	cmdList,
	cmdBand,
	cmdCfg,
#ifdef SET_BLE
	cmdWakeUp,
#endif
	cmdExitSleep,
	cmdSleep,
	cmdSleepCont,
#ifdef SET_RDS
	cmdRds,
#endif
	cmdEvt,
	cmdAck,
	cmdCmd
#ifdef SET_IRED
	,cmdiRed
#endif
};

enum {
	evt_None = -1,
	evt_Help = 0,
	evt_Restart,
	evt_Epoch,
	evt_Err,
	evt_sRead,
	evt_sErase,
	evt_sNext,
	evt_sWrite,
	evt_Sec,
	evt_Ver,
	evt_Clr,
	evt_Scan,
	evt_Freq,
	evt_Vol,
	evt_Mute,
	evt_Bass,
	evt_List,
	evt_Band,
	evt_Cfg,
#ifdef SET_BLE
	evt_WakeUp,
#endif
	evt_ExitSleep,
	evt_Sleep,
	evt_SleepCont,
#ifdef SET_RDS
	evt_Rds,
#endif
	evt_Evt,
	evt_Ack,
	evt_Cmd
#ifdef SET_IRED
	,evt_iRed
#endif
};



#ifdef SET_W25FLASH
	#include "w25.h"
#endif
#ifdef SET_DISPLAY
	#include "ST7565.h"
#endif
#include "rda5807.h"
#ifdef SET_IRED
	#include "IRremote.h"
#endif

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define _10ms 1
#define _20ms (_10ms * 2)
#define _30ms (_10ms * 3)
#define _40ms (_10ms * 4)
#define _50ms (_10ms * 5)
#define _60ms (_10ms * 6)
#define _70ms (_10ms * 7)
#define _80ms (_10ms * 8)
#define _90ms (_10ms * 9)
#define _100ms (_10ms * 10)
#define _110ms (_10ms * 11)
#define _120ms (_10ms * 12)
#define _130ms (_10ms * 13)
#define _140ms (_10ms * 14)
#define _150ms (_10ms * 15)
#define _160ms (_10ms * 16)
#define _170ms (_10ms * 17)
#define _180ms (_10ms * 18)
#define _190ms (_10ms * 19)
#define _200ms (_10ms * 20)
#define _250ms (_10ms * 25)
#define _300ms (_10ms * 30)
#define _350ms (_10ms * 35)
#define _400ms (_10ms * 40)
#define _450ms (_10ms * 45)
#define _500ms (_10ms * 50)
#define _600ms (_10ms * 60)
#define _700ms (_10ms * 70)
#define _800ms (_10ms * 80)
#define _900ms (_10ms * 90)
#define _1s (_10ms * 100)
#define _1s25 ((_1s * 1) + _250ms)
#define _1s3 ((_1s * 1) + _300ms)
#define _1s5 (_1s + _500ms)
#define _2s (_1s * 2)
#define _2s3 ((_1s * 2) + _300ms)
#define _2s5 ((_1s * 2) + _500ms)
#define _3s (_1s * 3)
#define _4s (_1s * 4)
#define _4s3 ((_1s * 4) + _300ms)
#define _5s (_1s * 5)
#define _6s (_1s * 6)
#define _7s (_1s * 7)
#define _8s (_1s * 8)
#define _9s (_1s * 9)
#define _10s (_1s * 10)
#define _15s (_1s * 15)
#define _20s (_1s * 20)
#define _30s (_1s * 30)




#define MAX_UART_BUF    1024
#if defined(SET_BLE) || defined(SET_AUDIO)
	#define MAX_BLE_BUF  256
	#define MAX_ERR_CODE 12
#else
#define MAX_ERR_CODE 10
#endif

#ifdef SET_BLE
	#ifdef SET_RDS
		#ifdef SET_IRED
			#define MAX_CMDS  28
		#else
			#define MAX_CMDS  27
		#endif
	#else
		#ifdef SET_IRED
			#define MAX_CMDS  27
		#else
			#define MAX_CMDS  26
		#endif
	#endif
#else
	#ifdef SET_RDS
		#ifdef SET_IRED
			#define MAX_CMDS  27
		#else
			#define MAX_CMDS  26
		#endif
	#else
		#ifdef SET_IRED
			#define MAX_CMDS  26
		#else
			#define MAX_CMDS  25
		#endif
	#endif
#endif
#define MAX_LIST          26//25
#define MAX_BAND           4
#define MAX_STEP           4


#ifdef SET_FIFO_MODE
	#define MAX_FIFO_SIZE 64
#endif

#define MAX_SIZE_NAME     31
#pragma pack(push,1)
typedef struct {
	uint8_t band;
	float freq;
	char name[MAX_SIZE_NAME];
} rec_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct {
	uint16_t blockA;
	uint16_t blockB;
	uint16_t blockC;
	uint16_t blockD;
} blocks_t;
#pragma pack(pop)




/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void Report(const uint8_t addTime, const char *fmt, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISPLAY_ON_Pin GPIO_PIN_0
#define DISPLAY_ON_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_1
#define KEY0_GPIO_Port GPIOC
#define KEY0_EXTI_IRQn EXTI1_IRQn
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOC
#define KEY1_EXTI_IRQn EXTI2_IRQn
#define ERR_LED_Pin GPIO_PIN_3
#define ERR_LED_GPIO_Port GPIOC
#define TIK_LED_Pin GPIO_PIN_1
#define TIK_LED_GPIO_Port GPIOA
#define LOG_TX_Pin GPIO_PIN_2
#define LOG_TX_GPIO_Port GPIOA
#define LOG_RX_Pin GPIO_PIN_3
#define LOG_RX_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define IRED_Pin GPIO_PIN_11
#define IRED_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOC
#define SPI1_RST_Pin GPIO_PIN_2
#define SPI1_RST_GPIO_Port GPIOD
#define SPI1_DC_Pin GPIO_PIN_5
#define SPI1_DC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern I2C_HandleTypeDef hi2c1;

extern uint16_t devError;
extern SPI_HandleTypeDef *portFLASH;
extern SPI_HandleTypeDef *portLED;
extern I2C_HandleTypeDef *portI2C;
extern UART_HandleTypeDef *cmdPort;
extern volatile uint8_t uartRdy;
extern uint8_t rxByte;
extern uint16_t rxInd;
extern char rxBuf[MAX_UART_BUF];

extern uint32_t spi_cnt;
extern volatile uint8_t spiRdy;

#ifdef SET_RDS
	// регистры RDS data
	#define RDA5807M_REG_BLER_CD 0x10
	#define RDA5807M_REG_BLOCK_A 0x0C
	#define RDA5807M_REG_BLOCK_B 0x0D
	#define RDA5807M_REG_BLOCK_C 0x0E
	#define RDA5807M_REG_BLOCK_D 0x0F
	// флаги
	#define RDA5807M_FLG_DHIZ   0x8000
	#define RDA5807M_FLG_DMUTE  0x4000
	#define RDA5807M_FLG_SEEK   0x0100
	#define RDA5807M_FLG_SEEKUP 0x0200
	#define RDA5807M_FLG_ENABLE 0x0001
	#define RDA5807M_FLG_TUNE   0x0010
	#define RDA5807M_FLG_RDS    0x0008
	#define RDA5807M_FLAG_RDSR  0x8000
	#define RDA5807M_FLAG_STC   0x4000
	//маски
	#define RDA5807M_CHAN_MASK    0xFFC0
	#define RDA5807M_CHAN_SHIFT   6
	#define RDA5807M_VOLUME_MASK  0x000F
	#define RDA5807M_VOLUME_SHIFT 0
	#define RDA5807M_BLERA_MASK   0x000C
	#define RDA5807M_BLERA_SHIFT  2
	#define RDA5807M_BLERB_MASK   0x0003
	#define RDA5807M_BLERC_MASK   0xC000
	#define RDA5807M_BLERC_SHIFT  14
	#define RDA5807M_BLERD_MASK   0x3000
	#define RDA5807M_BLERD_SHIFT  12
	//группы RDS
	#define RDS_ALL_GROUPTYPE_MASK    0xF000
	#define RDS_ALL_GROUPTYPE_SHIFT   12
	#define RDS_ALL_GROUPVER          0x800
	#define RDS_ALL_TP                0x400
	#define RDS_ALL_PTY_MASK          0x3E0
	#define RDS_ALL_PTY_SHIFT         5
	#define RDS_GROUP0_TA             4
	#define RDS_GROUP0_MS             3
	#define RDS_GROUP0_DI             2
	#define RDS_GROUP0_C1C0_MASK      3
	#define RDS_GROUP4A_MJD15_16_MASK 3
	//#define RDS_GROUP4A_MJD0_14_MASK 0xFFFE
	#define RDS_GROUP4A_MJD0_14_SHIFT  1
	#define RDS_GROUP4A_HOURS4_MASK    1
	#define RDS_GROUP4A_HOURS0_3_MASK  0xF000
	#define RDS_GROUP4A_HOURS0_3_SHIFT 12
	#define RDS_GROUP4A_MINUTES_MASK   0x0FC0
	#define RDS_GROUP4A_MINUTES_SHIFT  6
	#define RDS_GROUP4A_LTO_SIGN_MASK  0x0020
	#define RDS_GROUP4A_LTO_MASK       0x001F
	#define RDS_GROUP2_ABFLAG_MASK     0x0010
	#define RDS_GROUP2_ADDRESS_MASK    0x000F
	//
	#define MAX_SPTY                   32
	#define REPEATS_TO_BE_REAL_ID      3
#endif
extern volatile uint8_t i2cRdy;
extern float lBand, rBand;
extern uint8_t Volume;
extern uint8_t BassBoost;
extern uint8_t Band;// = 2;


#pragma pack(push,1)
typedef struct rec_evt_t {
	int evt;
	uint32_t attr;
} rec_evt_t;
#pragma pack(pop)


#if defined(SET_BLE) || defined(SET_AUDIO)

#define MAX_QREC 16

#pragma pack(push,1)
typedef struct rec_msg_t {
	char *msg;
} rec_msg_t;
#pragma pack(pop)


#pragma pack(push,1)
typedef struct que_rec_t {
	int8_t id;
	char *adr;
} que_rec_t;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct s_recq_t {
	volatile uint8_t lock;
	uint8_t put;
	uint8_t get;
	que_rec_t rec[MAX_QREC];
} s_recq_t;
#pragma pack(pop)

	#ifdef SET_BLE
		#define BLE_WAKEUP_DOWN() BLE_WAKEUP_GPIO_Port->BSRR = (BLE_WAKEUP_Pin << 16)
		#define BLE_WAKEUP_UP() BLE_WAKEUP_GPIO_Port->BSRR = BLE_WAKEUP_Pin
	#endif

#endif


#ifdef SET_SLEEP
	#define WAIT_BEFORE_SLEEP 30
#endif

#ifdef SET_DISPLAY
	#define ON_DISPLAY() DISPLAY_ON_GPIO_Port->BSRR = (DISPLAY_ON_Pin << 16)
	#define OFF_DISPLAY() DISPLAY_ON_GPIO_Port->BSRR = DISPLAY_ON_Pin
#endif


#ifdef SET_IRED

	#define MAX_IRED_KEY 21

	enum {
		key_ch_minus = 0,
		key_ch,
		key_ch_plus,
		key_left,
		key_right,
		key_sp,
		key_minus,
		key_plus,
		key_eq,
		key_100,
		key_200,
		key_0,
		key_1,
		key_2,
		key_3,
		key_4,
		key_5,
		key_6,
		key_7,
		key_8,
		key_9
	};

	typedef struct {
		char name[8];
		uint32_t code;
	} one_key_t;

	extern TIM_HandleTypeDef *portIRED;//htim3; // таймер для приёма

#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
