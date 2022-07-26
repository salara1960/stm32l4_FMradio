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
	devFIFO = 0x10,
	devSYS = 0x20,
	devSPI = 0x40,
	devLCD = 0x80,
	devRDA = 0x100,
	devFS = 0x200
#ifdef SET_BLE
	,
	devBLE = 0x400,
	devQUE = 0x800
#endif
};

enum {
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
	cmdWakeUp
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
	evt_WakeUp
};



#ifdef SET_W25FLASH
	#include "w25.h"
#endif
#ifdef SET_DISPLAY
	#include "ST7565.h"
#endif
#ifdef SET_RDA_CHIP
	#include "rda5807.h"
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
#define _130ms (_10ms * 13)
#define _150ms (_10ms * 15)
#define _160ms (_10ms * 16)
#define _170ms (_10ms * 17)
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
#ifdef SET_BLE
	#define MAX_BLE_BUF  256
#endif
#define MAX_CMDS          20
#define MAX_LIST          25
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



/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
void Report(const uint8_t addTime, const char *fmt, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIK_LED_Pin GPIO_PIN_0
#define TIK_LED_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_1
#define KEY0_GPIO_Port GPIOC
#define KEY0_EXTI_IRQn EXTI1_IRQn
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOC
#define KEY1_EXTI_IRQn EXTI2_IRQn
#define ERR_LED_Pin GPIO_PIN_3
#define ERR_LED_GPIO_Port GPIOC
#define LOG_TX_Pin GPIO_PIN_2
#define LOG_TX_GPIO_Port GPIOA
#define LOG_RX_Pin GPIO_PIN_3
#define LOG_RX_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_12
#define GREEN_LED_GPIO_Port GPIOC
#define SPI1_RST_Pin GPIO_PIN_2
#define SPI1_RST_GPIO_Port GPIOD
#define SPI1_DC_Pin GPIO_PIN_5
#define SPI1_DC_GPIO_Port GPIOB
#define WAKEUP_Pin GPIO_PIN_8
#define WAKEUP_GPIO_Port GPIOB
#define BLE_STAT_Pin GPIO_PIN_9
#define BLE_STAT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern I2C_HandleTypeDef hi2c1;

extern uint16_t devError;
extern SPI_HandleTypeDef *portFLASH;
extern SPI_HandleTypeDef *portLED;
extern I2C_HandleTypeDef *portI2C;
extern UART_HandleTypeDef *cmdPort;
extern uint8_t uartRdy;
extern uint8_t rxByte;
extern uint16_t rxInd;
extern char rxBuf[MAX_UART_BUF];

extern uint32_t spi_cnt;
extern uint8_t spiRdy;

#ifdef SET_RDA_CHIP
	extern volatile uint8_t i2cRdy;
	extern float lBand, rBand;
	extern uint8_t Volume;
	extern uint8_t BassBoost;
	extern uint8_t Band;// = 2;
#endif


#ifdef SET_BLE

#define MAX_QREC 32

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

#define WAKEUP_DOWN() WAKEUP_GPIO_Port->BSRR = (WAKEUP_Pin << 16)
#define WAKEUP_UP() WAKEUP_GPIO_Port->BSRR = WAKEUP_Pin;

#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
