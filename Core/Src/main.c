/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  ******************************************************************************
  ******************************************************************************

  arm-none-eabi-objcopy -O ihex "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.hex" && arm-none-eabi-objcopy -O binary "${BuildArtifactFileBaseName}.elf" "${BuildArtifactFileBaseName}.bin" && ls -la | grep "${BuildArtifactFileBaseName}.*"

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defTask */
osThreadId_t defTaskHandle;
const osThreadAttr_t defTask_attributes = {
  .name = "defTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for evtQue */
osMessageQueueId_t evtQueHandle;
const osMessageQueueAttr_t evtQue_attributes = {
  .name = "evtQue"
};
/* Definitions for cmdQue */
osMessageQueueId_t cmdQueHandle;
const osMessageQueueAttr_t cmdQue_attributes = {
  .name = "cmdQue"
};
/* Definitions for ackQue */
osMessageQueueId_t ackQueHandle;
const osMessageQueueAttr_t ackQue_attributes = {
  .name = "ackQue"
};
/* Definitions for itSem */
osSemaphoreId_t itSemHandle;
const osSemaphoreAttr_t itSem_attributes = {
  .name = "itSem"
};
/* USER CODE BEGIN PV */

//const char *ver = "0.1 07.07.2022";
//const char *ver = "0.2 07.07.2022";
//const char *ver = "0.3 07.07.22";//add spi2 data flash (w25q16 chip on the board)
//const char *ver = "0.4 07.07.22";//add spi1 display st7565
//const char *ver = "0.5 08.07.22";
//const char *ver = "0.5 08.07.22";//new st7565_spi_proc for HAL_MODE
//const char *ver = "0.6 12.07.22";
//const char *ver = "0.7 15.07.22";
//const char *ver = "0.8 16.07.22";//fixed major bug in support st7565
//const char *ver = "0.8.1 16.07.22";
//const char *ver = "0.9 16.07.22";//new rda7565 lib
//const char *ver = "0.9.1 19.07.22";//new rda7565 lib
//const char *ver = "0.9.2 19.07.22";//add scan and set freq functions support
//const char *ver = "0.9.3 19.07.22";//add show_line function for radio params
//const char *ver = "0.9.4 20.07.22";
//const char *ver = "1.0 21.07.22";//add two button KEY0 & KEY1 (scan_up & scan_down)
//const char *ver = "1.1 21.07.22";//add new command 'list'
//const char *ver = "1.2 22.07.22";//add select band feature + fatfs
//const char *ver = "1.3 23.07.22";
//const char *ver = "1.4 24.07.22";// new feature for 'list' command
//const char *ver = "1.4.2 25.07.22";// without FatFs release
//const char *ver = "1.5 26.07.22";// add bluetooth device 'JDY-25M'
//const char *ver = "1.5.1 26.07.22";// add sleep/wakeup features for BLE device
//const char *ver = "1.5.2 27.07.22";// add sleep/wakeup features for CPU+BLE
//const char *ver = "1.5.3 28.07.22";// add ON/OFF dislay pin
//const char *ver = "1.6 29.07.22";// add Infrared control
//const char *ver = "1.6.1 29.07.22";// add read RDS (first step)
//const char *ver = "1.7 01.08.22";// remove BLE and add bluetooth_audio device
//const char *ver = "1.7.1 02.08.22";// fixed minor bug in add to queue record
//const char *ver = "1.7.2 03.08.22";
//const char *ver = "1.8 03.08.22";// add FreeRTOS
//const char *ver = "1.8.1 04.08.22";// minor changes in callback_timer_6 (support infrared)
//const char *ver = "1.8.2 05.08.22";//fixed minor error from portBLE at startup (when power on)
//const char *ver = "1.8.3 07.08.22";//set ANT_TYPE to External
//const char *ver = "1.8.4 08.08.22";
//const char *ver = "1.8.5 09.08.22";//add new command for support infrared
//const char *ver = "1.8.6 10.08.22";//add new radio_station in play_list
//const char *ver = "1.9 11.08.22";//rds support - done !
//const char *ver = "1.9.1 12.08.22";//rds - set on by start
//const char *ver = "2.0 12.08.22";//add encoder !!!
const char *ver = "2.1 13.08.22";//encoder support done



osThreadId_t irdTaskHandle;
const osThreadAttr_t irdTask_attributes = {
  .name = "irdTask",
  .stack_size = 1536 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,//osPriorityNormal,
};
bool waitBit = true;

char stx[MAX_UART_BUF] = {0};
char tmp[128] = {0};
char cmdBuf[MAX_UART_BUF] = {0};
char strf[MAX_UART_BUF] = {0};
uint16_t devError = HAL_OK;
uint16_t last_devError = HAL_OK;
const uint16_t all_devErr[MAX_ERR_CODE] = {
		devTIK,
		devUART,
		devMEM,
		devRTC,
		devEVT,
		devSYS,
		devSPI,
		devLCD,
		devRDA,
		devFS
#if defined(SET_BLE) || defined(SET_AUDIO)
		,devBLE,
		devQUE
#endif
};


volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t msCounter = 0;//period 250ms

TIM_HandleTypeDef *tikPort = &htim4;
SPI_HandleTypeDef *portFLASH = &hspi2;
SPI_HandleTypeDef *portLED = &hspi1;
I2C_HandleTypeDef *portI2C = &hi2c1;
UART_HandleTypeDef *cmdPort = &huart2;

volatile uint8_t uartRdy = 1;
uint8_t rxByte = 0;
uint16_t rxInd = 0;
char rxBuf[MAX_UART_BUF] = {0};
volatile uint8_t restart = 0;

static uint32_t epoch = 1660401165;//1660343410;//1660296769;//1660238159;//1660232569;
//1660165305;//1660052289;//1659974469;//1659886879;//1659874060;//1659702715;//1659624810;
//1659614411;//1659558535;//1659552520;//1659535529;//1659476485;//1659465512;//1659390226;//1659381664;
//1659130699;//1659116379;//1659105660;//1659040054;//1659015162;//1659001909;
//1658961169;//1658870659;//1658868340;//1658836899;//1658775452;//1658774189;//1658673059;//1658665853;
//1658587329;//1658581090;//1658579999;//1658573857;//1658529249;//1658521643;//1658501279;
//1658489899;//1658432922;//1658402955;//1658326638;//1658248185;//1658240652;//1658227367;//1657985710;
//1657971799;1657915595;1657635512;1657313424;//1657283440;//1657234028;//1657200272;//1657194633;//1657144926;
bool setDate = false;
uint8_t tZone = 0;


const char *s_cmds[MAX_CMDS] = {
	"help",
	"restart",
	"epoch:",
	"inputerr",
	"read",
	"erase",
	"next",
	"write",
	"sec",
	"ver",
	"clr",
	"scan",
	"freq:",
	"vol:",
	"mute",
	"bass:",
	"list",
	"band:",
	"cfg",
#ifdef SET_BLE
	"wakeup",
#endif
	"exitsleep",
	"sleep",
	"sleepcont",
#ifdef SET_RDS
	"rds",
#endif
	"qevt",
	"qack",
	"qcmd"
#ifdef SET_IRED
	,"ired"
#endif
#ifdef SET_ENC
	,
	"enckey",
	"encinc",
	"encdec"
#endif
};
const char *str_cmds[MAX_CMDS] = {
	"Help",
	"Restart",
	"Epoch",
	"InputErr",
	"spiRead",
	"spiErase",
	"spiNext",
	"spiWrite",
	"Seconda",
	"Version",
	"ClearErr",
	"Scan",
	"setFreq",
	"Volume",
	"muteRadio",
	"BassBoost",
	"nextStation",
	"Band",
	"cfgStations",
#ifdef SET_BLE
	"BleWakeUp",
#endif
	"ExitSleep",
	"Sleep",
	"SleepCont",
#ifdef SET_RDS
	"readRDS",
#endif
	"queEvt",
	"queAck",
	"queCmd"
#ifdef SET_IRED
	,"iredShow"
#endif
#ifdef SET_ENC
	,
	"encKey",
	"encInc",
	"encDec"
#endif
};


int evt = evt_None;
int next_evt = evt_None;
volatile uint8_t cntEvt = 0;

uint32_t spi_cnt = 0;
volatile uint8_t spiRdy = 1;
//
#ifdef SET_W25FLASH
	int adr_sector = 0, offset_sector = 0, list_sector = 0, len_write = 0;
	int cmd_sector = sNone, last_cmd_sector = sNone;
	uint8_t byte_write = 0xff;
	bool flag_sector = false;
	unsigned char fs_work[_MAX_SS] = {0};
	bool chipPresent = false;
	bool validChipID = false;
	uint32_t btime = 0, etime = 0;
	uint32_t cfgSector = 0;
#endif

#ifdef SET_DISPLAY
	volatile bool startSec = false;
	FontDef_t *lfnt = NULL;
#endif


float Freq = 94.0;//96.3;//95.1;
float newFreq = 95.1;
float lBand = 0.0;
float rBand = 0.0;
uint8_t Band = 2;// :2
uint8_t newBand = 2;
uint8_t Step = 0;// 100 КГц
uint8_t newStep = 0;
uint16_t Chan = 0;
uint16_t RSSI = 0;
volatile uint8_t i2cRdy = 1;
uint8_t rdaID = 0;
volatile uint8_t scan = 0;
volatile uint8_t seek_up = 1;
uint8_t Volume = 5;
uint8_t newVolume = 5;
uint8_t BassBoost = 0;
uint8_t newBassBoost = 0;
bool stereo = false;
uint8_t noMute = 1;
uint8_t dataRDS[8] = {0};
bool syncRds = false;
bool readyRds = false;
//
const char *noneStation = "???";
static const rec_t def_list[MAX_LIST] = {
	//Band:3 65-76
	{3, 68.5, "Маяк"},// Маяк
	{3, 72.1, "Шансон"},// Шансон
	//Band:2,1 76-108, 87-108
	{2, 92.8, "Радио_DFM"},//Радио DFM в Калининграде
	{2, 93.6, "Радио_7"},// Радио 7
	{2, 94, "Комеди_Радио"},// Комеди Радио
	{2, 95.1, "Вести_ФМ"},// Вести ФМ
	{2, 95.5, "Ретро_ФМ"},// Ретро ФМ
	{2, 96.3, "Русское_Радио"},// Русское Радио
	{2, 97, "Радио_Вера"},// Радио Книга
	{2, 97.7, "Серебр.Дождь"},// Серебрянный Дождь
	{2, 98.5, "Радио_Энергия"},// Радио Энергия
	{2, 99.5, "Радио_Звезда"},// Радио Звезда
	{2, 100.1, "Авто_Радио"},// АвтоРадио
	{2, 100.6, "Русский_Край"},// Русский Край
	{2, 100.9, "Монте-Карло"},// Монте-Карло
	{2, 101.3, "Наше_Радио"},// Наше Радио
	{2, 101.8, "Бизнес_ФМ"},// Бизнес ФМ
	{2, 102.5, "Маяк"},// Маяк
	{2, 102.9, "Любимое_Радио"},// Любимое Радио
	{2, 103.4, "Студия_21"},// Студия 21
	{2, 103.9, "Радио_России"},// Радио России
	{2, 104.5, "Европа_Плюс"},// Европа Плюс
	{2, 105.2, "Балтик_Плюс"},// Балтик Плюс
	{2, 105.9, "Дорожное_Радио"},// Дорожное Радио
	{2, 106.4, "Радио_Максим"},// Радио Максим
	{2, 107.2, "Радио_КП"}// Комсомольская Правда
};
rec_t list[MAX_LIST];
uint16_t listSize = 0;
	//
const char *allBands[MAX_BAND] = {
	"87-108 MHz",// (US/Europe)",
	"76-91 MHz",// (Japan)",
	"76-108 MHz",// (world wide)",
	"65-76 MHz"// (East Europe) or 50-65MHz"
};

const step_t allSteps[MAX_STEP] = {
	{0.1, "100"},
	{0.2, "200"},
	{0.05, "50"},
	{0.025, "25"}
};



#if defined(SET_BLE) || defined(SET_AUDIO)
	UART_HandleTypeDef *blePort = &huart3;
	//
	volatile uint8_t bleRdy = 1;
	uint8_t rxbByte = 0;
	uint16_t rxbInd = 0;
	char rxbBuf[MAX_BLE_BUF] = {0};
	char txbBuf[MAX_BLE_BUF] = {0};
	char bleBuf[MAX_BLE_BUF] = {0};
	char bleRxBuf[MAX_BLE_BUF] = {0};
	uint8_t ble_withDMA = 1;
	//volatile uint8_t bleReady = 1;
	uint8_t ble_stat = 0;
	uint8_t adone = 0;
	//
	s_recq_t bleQueAck;
	s_recq_t bleQueCmd;
	bool bleQueAckFlag = false;
	bool bleQueCmdFlag = false;
	const char *ble_statName[] = {"Disconnected", "Connected"};
	rec_msg_t _ack;
	rec_msg_t _cmd;
#endif

#ifdef SET_SLEEP
	bool sleep_mode = false;
#endif

#ifdef SET_IRED

	const one_key_t keyAll[MAX_IRED_KEY] = {
			{"irCH-",   0x2C090B43},//e318261b},
			{"irCH",    0x494202E3},//00511dbb},
			{"irCH+",   0x377952A7},//ee886d7f},
			{"irLEFT",  0x9B94B947},//52a3d41f},
			{"irRIGHT", 0x20D93043},//d7e84b1b},
			{"irSP",    0x69EF32E3},//20fe4dbb},
			{"ir-",     0x3967A663},//f076c13b},
			{"ir+",     0xECB9D303},//a3c8eddb},
			{"irEQ",    0x2EC0A2A7},//e5cfbd7f},
			{"ir100+",  0xE0392123},//97483bfb},
			{"ir200+",  0x39B4FB6B},//f0c41643},
			{"ir0",     0x09F2CAA3},//c101e57b},
			{"ir1",     0xE007A367},//9716be3f},
			{"ir2",     0x868BC91F},//3d9ae3f7},
			{"ir3",     0xAA72E743},//6182021b},
			{"ir4",     0xD5134AA3},//8c22657b},
			{"ir5",     0x918021E3},//488f3cbb},
			{"ir6",     0x4D3ACCC7},//0449e79f},
			{"ir7",     0x7BB7E31F},//32c6fdf7},
			{"ir8",     0x64B0FAA3},//1bc0157b},
			{"ir9",     0x87B4E143}//3ec3fc1b}
	};

	TIM_HandleTypeDef *portIRED = &htim6; // таймер для приёма
	char stline[40] = {0};
	bool ired_show = true;
#endif

uint8_t prio = 0;
volatile bool ird_exit = true;


#ifdef SET_RDS

	const char *namePTy[MAX_SPTY] = {
		"No program type or undefined",
		"News",
		"Current affairs",
		"Information",
		"Sport",
		"Education",
		"Drama",
		"Culture",
		"Science",
		"Varied",
		"Pop music",
		"Rock music",
		"Easy listening",
		"Light classical",
		"Serious classical",
		"Other music",
		"Weather",
		"Finance",
		"Children’s programs",
		"Social affairs",
		"Religion",
		"Phone-in",
		"Travel",
		"Leisure",
		"Jazz music",
		"Country music",
		"National music",
		"Oldies music",
		"Folk music",
		"Documentary",
		"Alarm test",
		"Alarm"
	};
	uint16_t sID = 0; // ID радиостанции
	uint16_t MaybeThisIDIsReal = 0; // Предыдущее значение ID
	uint8_t IDRepeatCounter = 0; // Счетчик повторений ID
	uint8_t errLevelB, errLevelC, errLevelD, groupType, groupVer;
	uint8_t PTy = 255;
	bool PTy_printed = false;
	char PSName[9]; // Значение PSName
	char PSName_prev[9];
	uint8_t PSNameUpdated = 0; // Для отслеживания изменений в PSName
#endif


#ifdef SET_ENC
	TIM_HandleTypeDef *encPort = &htim8;
	uint32_t tikStart = 0;
	bool encKeyPressed = false;
	uint32_t Encoder = MIN_ENC_VALUE;
	uint32_t lastEncoder = MIN_ENC_VALUE;
	uint32_t encKeyCnt = 0;
	uint8_t encKeyCntTmp = 0;
	GPIO_PinState encState = GPIO_PIN_SET;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
void StartTask(void *argument);

/* USER CODE BEGIN PFP */
void irdTask(void *argument);

uint32_t get_tmr(uint32_t sec);
bool check_tmr(uint32_t sec);
uint64_t get_mstmr(uint64_t hs);
bool check_mstmr(uint64_t hs);
void errLedOn(bool on);
void set_Date(uint32_t usec);
int sec2str(char *st);
void Report(const uint8_t addTime, const char *fmt, ...);
void showLine(char *msg, uint16_t lin, int *lil, bool update);
const char *nameStation(float fr);
float getNextList(float fr, uint8_t up, uint8_t *band);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//------------------------------------------------------------------------------------------
void showCfg()
{
	*strf = '\0';
	for (int i = 0; i < MAX_LIST; i++) {
		sprintf(strf+strlen(strf), "%u:%.1f:%s\r\n", list[i].band, list[i].freq, list[i].name);
	}
	Report(0, "%s", strf);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------

#ifdef SET_BLE
//-------------------------------------------------------------------------------------------
void bleWakeUp()
{
	BLE_WAKEUP_DOWN();
	HAL_Delay(50);
	BLE_WAKEUP_UP();
}
//-------------------------------------------------------------------------------------------
uint8_t get_bleStat()
{
	return HAL_GPIO_ReadPin(BLE_STAT_GPIO_Port, BLE_STAT_Pin);
}
//-------------------------------------------------------------------------------------------
#endif


#if defined(SET_BLE) || defined(SET_AUDIO)
//-------------------------------------------------------------------------------------------
void bleWrite(const char *str, bool prn)
{
	if (sleep_mode) return;

	if (ble_withDMA) {
		while (!bleRdy) {};
		bleRdy = 0;
		if (HAL_UART_Transmit_DMA(blePort, (uint8_t *)str, strlen(str)) != HAL_OK) devError |= devBLE;
		while (!bleRdy) {};
		/*while (HAL_UART_GetState(blePort) != HAL_UART_STATE_READY) {
			if (HAL_UART_GetState(blePort) == HAL_UART_STATE_BUSY_RX) break;
			//HAL_Delay(1);
		}*/
	} else {
		if (HAL_UART_Transmit(blePort, (uint8_t *)str, strlen(str), 1000) != HAL_OK) devError |= devBLE;
	}

	if (prn) Report(1, "[BLE_tx] %s", str);

}
//------------------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
static char *errName(uint16_t err)
{

	switch (err) {
		case devTIK:// = 1,
			return "devTIK";
		case devUART:// = 2,
			return "devUART";
		case devMEM://= 4,
			return "devMEM";
		case devRTC:// = 8,
			return "devRTC";
		case devEVT:// = 0x10,
			return "devEVT";
		case devSYS:// = 0x20,
			return "devSYS";
		case devSPI:// = 0x40,
			return "devSPI";
		case devLCD:// = 0x80,
			return "devLCD";
		case devRDA:// = 0x100,
			return "devRDA";
		case devFS:// = 0x200
			return "devFS";
#if defined(SET_BLE) || defined(SET_AUDIO)
		case devBLE:// = 0x400,
			return "devBLE";
		case devQUE:// = 0x800
			return "devQUE";
#endif
	}

	return "???";
}
//-------------------------------------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


    if (HAL_TIM_Base_Start_IT(tikPort) != HAL_OK) devError |= devTIK;

    if (HAL_UART_Receive_IT(cmdPort, &rxByte, 1) != HAL_OK) devError |= devUART;
#if defined(SET_BLE) || defined(SET_AUDIO)
    if (HAL_UART_Receive_IT(blePort, &rxbByte, 1) != HAL_OK) devError |= devBLE;
#endif

#ifdef SET_ENC
    HAL_TIM_Encoder_Start_IT(encPort, TIM_CHANNEL_ALL);
    HAL_Delay(100);
    lastEncoder = Encoder = encPort->Instance->CNT;//TIM8->CNT;
#endif


    for (int8_t i = 0; i < 4; i++) {
    	errLedOn(true);
    	HAL_Delay(100);
    	errLedOn(false);
    	HAL_Delay(100);
    }

    set_Date(epoch);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of itSem */
  itSemHandle = osSemaphoreNew(1, 1, &itSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of evtQue */
  evtQueHandle = osMessageQueueNew (8, sizeof(rec_evt_t), &evtQue_attributes);

  /* creation of cmdQue */
#if defined(SET_BLE) || defined(SET_AUDIO)
  cmdQueHandle = osMessageQueueNew (8, sizeof(rec_msg_t), &cmdQue_attributes);

  /* creation of ackQue */
  ackQueHandle = osMessageQueueNew (8, sizeof(rec_msg_t), &ackQue_attributes);
#endif
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defTask */
  defTaskHandle = osThreadNew(StartTask, NULL, &defTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  HAL_Delay(250);
  irdTaskHandle = osThreadNew(irdTask, NULL, &irdTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  //osStat = osKernelStart();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702D95;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JULY;
  sDate.Date = 0x6;
  sDate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  //set_Date(epoch);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  // interupt every 10 ms

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = MIN_ENC_VALUE;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = MAX_ENC_VALUE;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISPLAY_ON_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TIK_LED_GPIO_Port, TIK_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DISPLAY_ON_Pin */
  GPIO_InitStruct.Pin = DISPLAY_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISPLAY_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY0_Pin KEY1_Pin */
  GPIO_InitStruct.Pin = KEY0_Pin|KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ERR_LED_Pin */
  GPIO_InitStruct.Pin = ERR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ERR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TIK_LED_Pin */
  GPIO_InitStruct.Pin = TIK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TIK_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_KEY_Pin */
  GPIO_InitStruct.Pin = ENC_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRED_Pin */
  GPIO_InitStruct.Pin = IRED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_DC_Pin */
  GPIO_InitStruct.Pin = SPI1_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

//*******************************************************************************************

//-------------------------------------------------------------------------------------------
//      Преобразует два символа строки из hex-формата в двоичный
//
uint8_t hexToBin(char *sc)
{
char st = 0, ml = 0;

	if ((sc[0] >= '0') && (sc[0] <= '9')) st = (sc[0] - 0x30);
	else
	if ((sc[0] >= 'A') && (sc[0] <= 'F')) st = (sc[0] - 0x37);
	else
	if ((sc[0] >= 'a') && (sc[0] <= 'f')) st = (sc[0] - 0x57);

	if ((sc[1] >= '0') && (sc[1] <= '9')) ml = (sc[1] - 0x30);
	else
	if ((sc[1] >= 'A') && (sc[1] <= 'F')) ml = (sc[1] - 0x37);
	else
	if ((sc[1] >= 'a') && (sc[1] <= 'f')) ml = (sc[1] - 0x57);

	return ((st << 4) | (ml & 0x0f));

}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
uint32_t getQueCount(osMessageQueueId_t que)
{
	return osMessageQueueGetCount(que);
}
//-------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------
#ifdef SET_DISPLAY

//-------------------------------------------------------------------------------------------
const char *nameStation(float fr)
{
int8_t ik = -1;

	for (int8_t i = 0; i < MAX_LIST; i++) {
		if (list[i].freq == fr) {
			ik = i;
			break;
		}
	}

	if (ik != -1) return list[ik].name;
			 else return noneStation;
}
//-------------------------------------------------------------------------------------------
float getNextList(float fr, uint8_t up, uint8_t *band)
{
float ret = fr;
int8_t ik = -1;

	for (int8_t i = 0; i < MAX_LIST; i++) {
		if (list[i].freq == fr) {
			ik = i;
			break;
		}
	}
	if (ik != -1) {
		if (up) {
			if (++ik == MAX_LIST) ik = 0;
		} else {
			if (ik != 0) ik--; else ik = MAX_LIST - 1;
		}
	} else {
		if (up) {// seek_up
			for (int8_t i = ik; i < MAX_LIST; i++) {
				if (list[i].freq > fr) {
					ik = i;
					break;
				}
			}
		} else {// seek_down
			for (int8_t i = ik; i <= 0; i--) {
				if (list[i].freq < fr) {
					ik = i;
					break;
				}
			}
		}
		if (ik == -1) ik = 0;
	}
	ret = list[ik].freq;
	*band = list[ik].band;
	Report(1, "[%s] up=%u ik=%d, fr=%.1f ret=%.1f band=%u\r\n", __func__, up, ik, fr, ret, *band);

	return ret;
}
//-------------------------------------------------------------------------------------------
void showLine(char *msg, uint16_t lin, int *last_len, bool update)
{
int il = strlen(msg);
int xf = ( (SCREEN_WIDTH - (lfnt->FontWidth * il) ) >> 1);// & 0x7f;
bool yes = false;

	if (*last_len > il) {
		ST7565_DrawFilledRectangle(2, lin, SCREEN_WIDTH - 4, lfnt->FontHeight, PIX_OFF);
		yes = true;
	}
	*last_len = il;
	if (*msg > 0x7f) {
		xf += il;
	}
	if (!yes) ST7565_DrawFilledRectangle(2, lin, SCREEN_WIDTH - 4, lfnt->FontHeight, PIX_OFF);

	if ((!xf) || (xf > (SCREEN_WIDTH - 4))) xf = 1;
	ST7565_Print(xf, lin, msg, lfnt, 1, PIX_ON);
	if (update) ST7565_Update();
}
#endif
//-------------------------------------------------------------------------------------------
void errLedOn(bool on)
{
	if (on)
		HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_SET);//LED ON
	else
		HAL_GPIO_WritePin(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_PIN_RESET);//LED OFF
}
//------------------------------------------------------------------------------------
uint32_t get_secCounter()
{
	return secCounter;
}
//-----------------------------------------------------------------------------
void inc_secCounter()
{
	secCounter++;
}
//-----------------------------------------------------------------------------
uint64_t get_msCounter()
{
	return msCounter;
}
//-----------------------------------------------------------------------------
void inc_msCounter()
{
	msCounter++;
}
//------------------------------------------------------------------------------------------
uint32_t get_tmr(uint32_t sec)
{
	return (get_secCounter() + sec);
}
//------------------------------------------------------------------------------------------
bool check_tmr(uint32_t sec)
{
	return (get_secCounter() >= sec ? true : false);
}
//------------------------------------------------------------------------------------------
uint64_t get_mstmr(uint64_t hs)
{
	return (get_msCounter() + hs);
}
//------------------------------------------------------------------------------------------
bool check_mstmr(uint64_t hs)
{
	return (get_msCounter() >= hs ? true : false);
}
//------------------------------------------------------------------------------------------
void toUppers(char *st)
{
int i;

    for (i = 0; i < strlen(st); i++) *(st + i) = toupper(*(st + i));
}
//------------------------------------------------------------------------------------------
void set_Date(uint32_t usec)
{
struct tm ts;
time_t ep = (time_t)usec;

	gmtime_r(&ep, &ts);

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sDate.WeekDay = ts.tm_wday;
	sDate.Month   = ts.tm_mon + 1;
	sDate.Date    = ts.tm_mday;
	sDate.Year    = ts.tm_year;
	sTime.Hours   = ts.tm_hour + tZone;
	sTime.Minutes = ts.tm_min;
	sTime.Seconds = ts.tm_sec;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
	else {
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) devError |= devRTC;
		else {
			setDate = true;
		}
	}
}
//-----------------------------------------------------------------------------------------
uint32_t getSecRTC(RTC_HandleTypeDef *hrtc)
{
time_t ep = 0;
RTC_DateTypeDef sDate;

	if (HAL_RTC_GetDate(hrtc, &sDate, RTC_FORMAT_BIN)) devError |= devRTC;
	else {
		RTC_TimeTypeDef sTime;
		if (HAL_RTC_GetTime(hrtc, &sTime, RTC_FORMAT_BIN)) devError |= devRTC;
		else {
			struct tm ts;
			ts.tm_sec = sTime.Seconds;
			ts.tm_min = sTime.Minutes;
			ts.tm_hour = sTime.Hours;
			ts.tm_mday = sDate.Date;
			ts.tm_mon = sDate.Month - 1;
			ts.tm_year = sDate.Year;
			ts.tm_wday = sDate.WeekDay;
			ep = mktime(&ts);
		}
	}

	return ep;
}
//------------------------------------------------------------------------------------------
int sec2str(char *st)
{
int ret = 0;

	if (!setDate) {
		uint32_t sec = get_secCounter();

		uint32_t day = sec / (60 * 60 * 24);
		sec %= (60 * 60 * 24);
		uint32_t hour = sec / (60 * 60);
		sec %= (60 * 60);
		uint32_t min = sec / (60);
		sec %= 60;

		ret = sprintf(st, "%lu.%02lu:%02lu:%02lu", day, hour, min, sec);
	} else {
		RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;
		if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)) devError |= devRTC;
		else {
			if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)) devError |= devRTC;
			else {
				ret = sprintf(st, "%02u.%02u %02u:%02u:%02u",
								sDate.Date, sDate.Month,
								sTime.Hours, sTime.Minutes, sTime.Seconds);
			}
		}
	}

	return ret;
}
//--------------------------------------------------------------------------------------------
void Report(const uint8_t addTime, const char *fmt, ...)
{
#if defined(SET_BLE) || defined(SET_AUDIO)
	if(sleep_mode) return;
#endif


	size_t len = MAX_UART_BUF;
	char *buf = &cmdBuf[0];

/*
	uint8_t cnt = 32;
	uint32_t stim = HAL_GetTick();
	uint32_t etim = stim;
	while (!uartRdy && cnt) {
		etim = HAL_GetTick();
		if (etim - stim) {
			stim = HAL_GetTick();
			cnt--;
		}
	}
*/
	//if (buf) {
		*buf = '\0';
		int dl = 0;
		if (addTime) {
			dl = sec2str(buf);
			strcat(buf, " | ");
			dl += 3;
		}

		va_list args;
		va_start(args, fmt);
		vsnprintf(buf + dl, len - dl, fmt, args);

		uartRdy = false;
		if (osSemaphoreAcquire(itSemHandle, 2000) == osOK) {
			if (HAL_UART_Transmit_DMA(cmdPort, (uint8_t *)buf, strlen(buf)) != HAL_OK) devError |= devUART;
			while (!uartRdy) {} //HAL_Delay(1)
			osSemaphoreRelease(itSemHandle);
		}

		va_end(args);

	//	free(buf);
	//} else {
	//	devError |= devMEM;
	//}

}
//-------------------------------------------------------------------------------------------
#ifdef SET_RDS
//-------------------------------------------------------------------------------------------
void MJDDecode(unsigned long MJD, uint16_t *y, uint8_t *m, uint8_t *d)
{
unsigned long L = 2400000 + MJD + 68570;
unsigned long N = (L * 4) / 146097;

	L -= ((146097.0 * N + 3) / 4);
	uint16_t year = 4000 * (L + 1) / 1461001;
	L -= (1461 * year / 4 + 31);
	uint8_t month = 80.0 * L / 2447.0;
	uint8_t day = L - 2447 * month / 80;
	L = month / 11;
	month = month + 2 - 12 * L;
	year = 100 * (N - 49) + year + L;
	*y = year;
	*m = month;
	*d = day;
}
//-------------------------------------------------------------------------------------------
void rds_init()
{
//
	sID = 0;
	MaybeThisIDIsReal = 0;
	IDRepeatCounter = 0;
	errLevelB = 0, errLevelC = 0, errLevelD = 0, groupType = 0, groupVer = 0;
	PTy = 255;
	PTy_printed = false;
	memset(PSName, 0, sizeof(PSName)); // Значение PSName
	memset(PSName_prev, 0, sizeof(PSName));
	PSNameUpdated = 0;
//
}
//-------------------------------------------------------------------------------------------

#endif
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		uartRdy = 1;
	}
#if defined(SET_BLE) || defined(SET_AUDIO)
	else
	if (huart->Instance == USART3) {
		bleRdy = 1;
	}
#endif
}
//-------------------------------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		devError |= devUART;
	}
#if defined(SET_BLE) || defined(SET_AUDIO)
	else
	if (huart->Instance == USART3) {
		devError |= devBLE;
	}
#endif
}
//-------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if defined(SET_BLE) || defined(SET_AUDIO)
	if (huart->Instance == USART3) {
		rxbBuf[rxbInd++] = (char)rxbByte;
		if (rxbByte == 0x0a) {// '\n'
			rxbBuf[--rxbInd] = '\0';
			int len = strlen(rxbBuf);
			// Блок помещает в очередь ответов на команду очередное сообщение от модуля BLE
			if (len > 1) {
				char *from_audio = (char *)calloc(1, MAX_BLE_BUF);//len + 1);
				//char *from_audio = (char *)pvPortMalloc(256);
				if (from_audio) {
					strncpy(from_audio, rxbBuf, MAX_BLE_BUF - 1);
					rec_msg_t ac;
					ac.msg = from_audio;
					if (osMessageQueuePut(ackQueHandle, (const void *)&ac, 0, 0) != osOK) {
						devError |= devQUE;
						free(from_audio);//vPortFree(from_audio);
					} else {
						if (devError & devQUE) devError &= ~devQUE;
					}
				} else {
					devError |= devMEM;
				}
			}
			//-----------------------------------------------------------------------------
			rxbInd = 0;
			memset(rxbBuf, 0, sizeof(rxbBuf));
		}
		//
		if (HAL_UART_Receive_IT(huart, &rxbByte, 1) != HAL_OK) devError |= devBLE;
	}
	else
#endif
	if (huart->Instance == USART2) {
		rxBuf[rxInd++] = (char)rxByte;
		if (rxByte == 0x0a) {//end of line
			rxBuf[--rxInd] = '\0';

			int i, ev = -1;
			if (strlen(rxBuf) > 2) {
#if defined(SET_BLE) || defined(SET_AUDIO)
				if ( (strstr(rxBuf, "at+")) || (strstr(rxBuf, "AT+")) ) {
					//int len = strlen(rxBuf);
					// Блок помещает в очередь команд очередную команду модулю BLE
					char *to_audio = (char *)calloc(1, MAX_BLE_BUF);
					if (to_audio) {
						strncpy(to_audio, rxBuf, MAX_BLE_BUF - 1);
						toUppers(to_audio);
						rec_msg_t cd;
						cd.msg = to_audio;
						if (osMessageQueuePut(cmdQueHandle, (const void *)&cd, 0, 0) != osOK) {
							devError |= devQUE;
							free(to_audio);
						} else {
							if (devError & devQUE) devError &= ~devQUE;
						}
					} else {
						devError |= devMEM;
					}
					ev = -2;
				} else {
#endif
					for (i = 0; i < MAX_CMDS; i++) {
						if (!strncmp(rxBuf, s_cmds[i], strlen(s_cmds[i]))) {
							char *uk = rxBuf + strlen(s_cmds[i]);
							ev = -1;
							switch (i) {
								case cmdBand://"band:2"
									if (strlen(uk) >= 1) {
										newBand = atol(uk);
										if (newBand != Band) {
											ev = i;
										}
									}
								break;
								case cmdVol:
									if (strlen(uk) >= 1) {
										uint8_t nv = Volume;
										if (strstr(uk, "up")) {
											nv++;
										} else if (strstr(uk, "down")) {
											nv--;
										} else {
											nv = (uint8_t)atol(uk);
										}
										if ((nv >= 0) && (nv <= 15)) {
											newVolume = nv;
											ev = i;
										}
									}
								break;
								case cmdBass:
									if (strlen(uk) >= 1) {
										newBassBoost = (uint8_t)atol(uk);
										ev = i;
									}
								break;
								case cmdFreq://"freq:95.1"
									if (strlen(uk) >= 2) {
										newFreq = (float)atof(uk);
										if (newFreq != Freq) {
											ev = i;
										}
									}
								break;
								case cmdScan://"scan"
								case cmdList://"list"
									seek_up = 1;
									ev = i;
									char *uki = strchr(uk, ':');
									if (uki) {
										if ((*(char *)(uki + 1) == '0') || strstr(uki + 1, "down")) seek_up = 0;
									}
								break;
								case cmdClr://"clr"
								case cmdHelp://"help"
								case cmdVer://"ver"
								case cmdMute://"mute"
								case cmdCfg://"cfg"
								case cmdRestart://"restart" -> restart = 1;
#ifdef SET_BLE
								case cmdWakeUp://"wakeup"
#endif
								case cmdSleep://"sleep" -> goto sleep mode
#ifdef SET_RDS
								case cmdRds://"rds"
#endif
								case cmdEvt://"qevt"
								case cmdAck://"qack"
								case cmdCmd://"qcmd"
#ifdef SET_IRED
								case cmdiRed://"ired"
#endif
									ev = i;
								break;
								case cmdEpoch://"epoch:1657191323"
									if (strlen(uk) >= 10) {
										char *uki = strchr(uk, ':');
										if (uki) {
											tZone = (uint8_t)atol(uki + 1);
											*uki = '\0';
										} else {
											tZone = 0;
										}
										epoch = (uint32_t)atol(uk);
										ev = i;
									}
								break;
								case cmdsRead:// read:0
								case cmdsErase:// erase:0
									if (i == cmdsRead) cmd_sector = cmdsRead;
												  else cmd_sector = cmdsErase;
									if (*uk == ':') {
										int sek = atoi(++uk);
										if ( ((sek >= 0) && (sek < W25qxx_getSectorCount())) || (sek == -1) ) {
											adr_sector = sek;
											offset_sector = 0;
											if (sek == -1) {
												if (cmd_sector == cmdsErase) ev = i;
											} else {
												ev = i;
											}
										}
									}
								break;
								case cmdsWrite:// write:0:a5 | write:0:a5:256
									if (*uk == ':') {
										uk++;
										int sek = atoi(uk);
										if ((sek >= 0) && (sek < W25qxx_getSectorCount())) {
											char *ukn = strchr(uk, ':');
											if (ukn) {
												len_write = -1;
												ukn++;
												byte_write = hexToBin(ukn);
												uk = strchr(ukn, ':');
												if (uk) {
													int l = atoi(++uk);
													if ((l > 0) && (l < W25qxx_getSectorSize())) len_write = l;
												}
												adr_sector = sek;
												offset_sector = 0;
												ev = i;//flag_sector = true;
											}
										}
									}
								break;
								case cmdsNext:// next
									if ((last_cmd_sector == cmdsRead) || (last_cmd_sector == cmdsNext)) {
										if ((offset_sector + list_sector) < W25qxx_getSectorSize()) {
											offset_sector += list_sector;
											ev = i;//flag_sector = true;
										}
									}
								break;
							}
							break;
						}
					}
#if defined(SET_BLE) || defined(SET_AUDIO)
				}
#endif
				//
				if (ev != -2) {
					if (ev == -1) ev = cmdErr;
					if (osMessageQueuePut(evtQueHandle, (const void *)&ev, 0, 0) != osOK) devError |= devEVT;
				}
				//
			}

			rxInd = 0;
			*rxBuf = '\0';
		}

		if (HAL_UART_Receive_IT(huart, &rxByte, 1) != HAL_OK) devError |= devUART;
	}
}
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//    Функция устанавливает в начальное состояние 'готов' служебные параметры порта SPI1:
//
void spiDone(SPI_HandleTypeDef *hspi)
{
#ifdef SET_W25FLASH
	if (hspi->Instance == SPI2) {
		W25_UNSELECT();
		spiRdy = 1;
	}
#endif

#ifdef SET_DISPLAY
	else
	if (hspi->Instance == SPI1) {
		lcdRdy = 1;
	}
#endif
}
//-------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении приема данных от порта SPI
//
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//-------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении передачи данных в порт SPI
//
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//-------------------------------------------------------------------------------------------
//    CallBack функция, вызывается по завершении приема/передачи данных порта SPI
//
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
}
//--------------------------------------------------------------------------------------------
//    CallBack функция, вызывается при возникновении ошибки у модуля SPI
//
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	spiDone(hspi);
	devError |= devSPI;
}
//--------------------------------------------------------------------------------------------
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) i2cRdy = 1;
}
//--------------------------------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) devError |= devRDA;
}
//--------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	rec_evt_t ev = {cmdNone, 0};
#ifdef SET_ENC
	if (GPIO_Pin == ENC_KEY_Pin) {
			encState = HAL_GPIO_ReadPin(ENC_KEY_GPIO_Port, ENC_KEY_Pin);
			if (encState == GPIO_PIN_SET) {//released key
				if (tikStart) {
					if ((HAL_GetTick() - tikStart) > TIME_encKeyPressed) {
						tikStart = 0;
						encKeyPressed = false;
						encKeyCnt++;
						ev.evt = cmdEncKey;
					}
				}
			} else {//pressed key
				encKeyPressed = true;
				if (!tikStart) tikStart = HAL_GetTick();
			}
			//HAL_GPIO_WritePin(ENC_LED_GPIO_Port, ENC_LED_Pin, encKeyPressed);
	} else {
#endif
#ifdef SET_SLEEP
		if (sleep_mode) {
			if ((HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) ||
					(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)) {
				sleep_mode = false;
				HAL_PWR_DisableSleepOnExit();
				ev.evt = cmdExitSleep;
			}
			return;
		}
#endif
		if ((GPIO_Pin == KEY0_Pin) || (GPIO_Pin == KEY1_Pin)) {
			if (GPIO_Pin == KEY0_Pin) seek_up = 1;
			else
			if (GPIO_Pin == KEY1_Pin) seek_up = 0;
			ev.evt = cmdScan;
		}
#ifdef SET_ENC
	}
#endif
	if (ev.evt != cmdNone) {
		if (osMessageQueuePut(evtQueHandle, (const void *)&ev, 0, 0)) devError |= devEVT;
	}
}
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
void irdTask(void *argument)
{
#ifdef SET_IRED

	ird_exit = false;

	bool ep_start = false;
	char ep_str[16] = {0};
	uint32_t ep_tmr = 0;
	uint32_t tmr_ired = 0;


	if (waitBit);

	enIntIRED();


  while (!restart) {

	if (!tmr_ired) {
		if (decodeIRED(&results)) {

			tmr_ired = get_mstmr(_300ms);
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
			int8_t kid = -1;
			for (int8_t i = 0; i < MAX_IRED_KEY; i++) {
				if (results.value == keyAll[i].code) {
					kid = i;
					break;
				}
			}
			//
			stline[0] = '\0';
			if (kid == -1) {
				if (ired_show) sprintf(stline, "CODE:%08lX", results.value);
			} else {
				sprintf(stline, "irKEY: %s", keyAll[kid].name);
			}
			if (strlen(stline)) Report(1, "[que:%u] %s\r\n", cntEvt, stline);
			//
			if (kid != -1) {
				int ird = evt_None;
				switch (kid) {
					case key_ch:
						ird = evt_Restart;
					break;
					case key_ch_plus:
						seek_up = 1;
						ird = evt_Scan;
					break;
					case key_ch_minus:
						seek_up = 0;
						ird = evt_Scan;
					break;
					case key_minus:
						if (Volume) {
							newVolume = Volume - 1;
							ird = evt_Vol;
						}
					break;
					case key_plus:
						if (Volume < 15) {
							newVolume = Volume + 1;
							ird = evt_Vol;
						}
					break;
					case key_left:
						seek_up = 0;
						ird = evt_List;
					break;
					case key_right:
						seek_up = 1;
						ird = evt_List;
					break;
					case key_eq:// enable/disable print via uart
						ird = evt_Mute;//evt_Sleep);
					break;
					case key_sp:
						if (!ep_start) {
							ep_start = true;
							memset(ep_str, 0, sizeof(ep_str));
							ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - lfnt->FontHeight, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_OFF);
							sprintf(tmp, "Time:");
							ST7565_Print(0, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
							ST7565_Update();
							ep_tmr = get_tmr(20);
						} else {
							ep_start = false;
							ep_tmr = 0;
							epoch = atoi(ep_str);
							ird = evt_Epoch;
						}
					break;
					case key_100://bandUp();
						if (Band < MAX_BAND) {
							newBand = Band + 1;
							ird = evt_Band;
						}
					break;
					case key_200://bandDown();
						if (Band) {
							newBand = Band - 1;
							ird = evt_Band;
						}
					break;
					case key_0:
					case key_1:
					case key_2:
					case key_3:
					case key_4:
					case key_5:
					case key_6:
					case key_7:
					case key_8:
					case key_9:
						if (ep_start) {
							if (strlen(ep_str) < 10) {
								char ch = (kid - key_0) + 0x30;
								sprintf(ep_str+strlen(ep_str), "%c", ch);
								ST7565_Print(32, SCREEN_HEIGHT - lfnt->FontHeight, ep_str, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
								ST7565_Update();
								ep_tmr = get_tmr(20);
							}
						} else {
							newFreq = list[kid - key_0 + 2].freq;//for band=2 only !!!
							ird = evt_Freq;
						}
					break;
				}//switch (kid)
				if (ird != evt_None) {
					if (osMessageQueuePut(evtQueHandle, (const void *)&ird, 0, 0) != osOK) devError |= devEVT;
				}
			}//if (kid != -1)
		}//if (decodeIRED(&results))
	}

	if (ep_tmr) {
		if (check_tmr(ep_tmr)) {
			ep_tmr = 0;
			ep_start = false;
		}
	}
	if (tmr_ired) {
		if (check_mstmr(tmr_ired)) {
			tmr_ired = 0;
			resumeIRED();
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
		}
	}

  }//while

  ird_exit = true;

  vTaskDelete(NULL);//osThreadExit();

#endif
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask */
/**
  * @brief  Function implementing the defTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask */
void StartTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */


    Report(1, "[que:%u] Start application ver.%s\r\n", getQueCount(evtQueHandle), ver);

    uint16_t lastErr = devOK;
    int evt;
    rec_evt_t evts = {cmdNone, 0};

#ifdef SET_W25FLASH
    chipPresent = W25qxx_Init();
    uint32_t cid = W25qxx_getChipID();
    if ( chipPresent && ((cid >= W25Q10) && (cid <= W25Q128)) ) validChipID = true;
    list_sector = W25qxx_getPageSize() << 1;
    //
    listSize = sizeof(rec_t) * MAX_LIST;
    memset((uint8_t *)&list[0].band, 0, listSize);
    //
    cfgSector = W25qxx_getSectorCount() - 1;
    if (W25qxx_IsEmptySector(cfgSector, 0, listSize)) {//sector is empty -> need write data to sector
    	if (!(devError & devSPI)) {
    		W25qxx_WriteSector((uint8_t *)&def_list[0].band, cfgSector, 0, listSize);
    		Report(1, "Writen cfg_stations_data (%lu bytes) to cfgSector #%lu\r\n", listSize, cfgSector);
      	}
    }
    if (!W25qxx_IsEmptySector(cfgSector, 0, listSize)) {//in sector	present any data
    	if (!(devError & devSPI)) {
    		W25qxx_ReadSector((uint8_t *)&list[0].band, cfgSector, 0, listSize);
    		Report(1, "Readed cfg_stations_data (%lu bytes) from cfgSector #%lu\r\n", listSize, cfgSector);
    	} else {
    		memcpy((uint8_t *)&list[0].band, (uint8_t *)&def_list[0].band, listSize);
    		Report(1, "Copy cfg_stations_data (%lu bytes) from def_list\r\n", listSize);
    	}
    }
#endif


rdaID = rda5807_init(&Freq, Band, Step);
RSSI = rda5807_rssi();
rda5807_SetVolume(Volume);
rda5807_SetBassBoost(BassBoost);
stereo = rda5807_Get_StereoMonoFlag();
Chan = rda5807_Get_Channel();


#ifdef SET_DISPLAY

	#ifdef FONT_6x8
  		FontDef_t Font_6x8 = { 6, 8, Font6x8 };
  		lfnt = &Font_6x8;
	#endif

  	uint16_t lin1 = 1;
  	uint16_t lin2 = lin1 + Font_6x8.FontHeight;//chipID...
  	uint16_t lin3 = lin2 + Font_6x8.FontHeight + 1;//Band...
  	uint16_t lin4 = lin3 + Font_6x8.FontHeight + 1;//Volume...//Freq...
  	uint16_t lin5 = lin4 + Font_6x8.FontHeight + 1;//Freq...//Volume...
  	uint16_t lin6 = lin5 + Font_6x8.FontHeight + 1;//Station
  	char st[64];
  	char sta[32];
  	char stb[32];

  	ST7565_Reset();
  	ST7565_Init();

    int dl = sprintf(tmp, "Ver.%s", ver);
    uint16_t x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    ST7565_Print(x, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)

    int il = sprintf(st, "RDA5807 chipID:0x%x", rdaID);
    uint16_t xf = ((SCREEN_WIDTH - (lfnt->FontWidth * il)) >> 1) & 0x7f;
    if (!xf) xf = 1;
    ST7565_Print(xf, lin2, st, lfnt, 1, PIX_ON);

    int it = sprintf(stb, "FM Band:%s", allBands[Band]);//(uint16_t)lBand, (uint16_t)rBand);
    int lit = it;
    xf = ((SCREEN_WIDTH - (lfnt->FontWidth * it)) >> 1) & 0x7f;
    if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    ST7565_Print(xf, lin3, stb, lfnt, 1, PIX_ON);

    int im = sprintf(st, "Bass:%u Vol:%u", BassBoost, Volume);
    int lim = im;
    xf = ((SCREEN_WIDTH - (lfnt->FontWidth * im)) >> 1) & 0x7f;
    if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    ST7565_Print(xf, lin4, st, lfnt, 1, PIX_ON);

    if (stereo)
    	il = sprintf(st, "Sig:%u Mhz:%.2f S", RSSI, Freq);
    else
    	il = sprintf(st, "Sig:%u Mhz:%.2f", RSSI, Freq);
    int lil = il;
    xf = ((SCREEN_WIDTH - (lfnt->FontWidth * il)) >> 1) & 0x7f;
    if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    ST7565_Print(xf, lin5, st, lfnt, 1, PIX_ON);

    int ia = sprintf(sta, "%s", nameStation(Freq));
    int lia = ia;
    xf = ((SCREEN_WIDTH - (lfnt->FontWidth * ia)) >> 1) & 0x7f;
    if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    ST7565_Print(xf, lin6, sta, lfnt, 1, PIX_ON);

    Report(1, "ChipID:0x%x Chan:%u Freq:%.3f %s Rssi:%u Band:%s Vol:%u BassEn:%u\r\n",
    			rdaID, Chan, Freq, sta, RSSI, allBands[Band], Volume, BassBoost);

    ST7565_DrawRectangle(0, lfnt->FontHeight, SCREEN_WIDTH - 1, SCREEN_HEIGHT - (lfnt->FontHeight << 1) - 2, PIX_ON);
    ST7565_DrawFilledRectangle(0, 0, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_ON);
    ST7565_Update();

    startSec = true;

#endif


#ifdef SET_IRED
    waitBit = false;
#endif

#ifdef SET_RDS
    blocks_t *blk = NULL;
    const uint64_t rdsWait = _30ms;
    bool rdsFlag = false;
    uint64_t rdsTime = 0;
    //rds_init();
    //
    evts.evt = evt_Rds;
    if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 20) != osOK) devError |= devEVT;
#endif

	evts.evt = evt_Freq;
	if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 20) != osOK) devError |= devEVT;


    while (!restart) {

		evts.evt = evt_None;
		if (osMessageQueueGet(evtQueHandle, &evts, NULL, 1) == osOK) {
			evt = evts.evt;
    		cntEvt = getQueCount(evtQueHandle);
    		if (evt != evt_Sec) {
    			//Report(1, "[que:%u] get event '%s'\r\n", cntEvt, str_cmds[evt]);
	#ifdef SET_DISPLAY
    			ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - lfnt->FontHeight, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_OFF);
    			int dl = sprintf(tmp, "evt(%u) : %s", cntEvt, str_cmds[evt]);
    			int x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    			ST7565_Print(x, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
    			ST7565_Update();
	#endif
    		}
    		switch (evt) {
#ifdef SET_ENC
    			case evt_EncKey:
    			{
    				newStep = (Step + 1) & 3;
    				Step = newStep;
    				//rda5807_Set_Step(Step);
    				Report(1, "[que:%u] encKey released now -> set new step to %u '%s КГц'\r\n", cntEvt, Step, allSteps[Step].name);
    				//
#ifdef SET_DISPLAY
    				ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - lfnt->FontHeight, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_OFF);
    				int dl = sprintf(tmp, "newStep : %s КГц", allSteps[Step].name);
    				int x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    				ST7565_Print(x, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
    				ST7565_Update();
#endif
    			}
    			break;
    			case evt_IncFreq:
    			case evt_DecFreq:
    				if (evt == evt_IncFreq) {
    					Report(1, "[que:%u] incFreq %.3f + step %s МГц = %.3f\r\n", cntEvt, newFreq, allSteps[Step].name, newFreq + allSteps[Step].freq);
    					newFreq += allSteps[Step].freq;
    				} else {
    					Report(1, "[que:%u] decFreq %.3f - step %s МГц = %.3f\r\n", cntEvt, newFreq, allSteps[Step].name, newFreq - allSteps[Step].freq);
    					newFreq -= allSteps[Step].freq;
    				}
    				//
    				evts.evt = evt_Freq;
    				if (osMessageQueuePut(evtQueHandle, (const void *)&evts, 1, 10) != osOK) devError |= devEVT;
    			break;
#endif
#ifdef SET_IRED
    			case evt_iRed:
    				if (ired_show) {
    					ired_show = false;
    					Report(1, "[que:%u] iRed Show code is hide\r\n", cntEvt);
    				} else {
    					ired_show = true;
    					Report(1, "[que:%u] iRed Show code is show\r\n", cntEvt);
    				}
    			break;
#endif
#ifdef SET_RDS
    			case evt_Rds:
    				rds_init();
    				if (!rdsFlag) {
    					Report(1, "[que:%u] RDS monitoring start\r\n", cntEvt);
    					rdsFlag = true;
    					rdsTime = get_mstmr(rdsWait);
    				} else {
    					Report(1, "[que:%u] RDS monitoring stop\r\n", cntEvt);
    					rdsFlag = false;
    					rdsTime = 0;
    				}
    			break;
#endif
    			case evt_SleepCont:
    				sleep_mode = true;
    				//
    				HAL_SuspendTick();
    				HAL_PWR_EnableSleepOnExit();
    				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    				HAL_ResumeTick();
    			break;
    			case evt_Sleep:
    				Report(1, "Going into SLEEP MODE...\r\n");// in 1 second\r\n");
	#ifdef SET_BLE
    				bleWrite("AT+SLEEP1\r\n", 1);
	#endif
	#ifdef SET_DISPLAY
    				ST7565_CMD_DISPLAY(CMD_DISPLAY_OFF);
	#endif
    				HAL_Delay(250);
    				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
    				evts.evt = evt_SleepCont;
    				if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 10) != osOK) devError |= devEVT;
    			break;
    			case evt_ExitSleep:
    				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
	#ifdef SET_DISPLAY
    				ST7565_CMD_DISPLAY(CMD_DISPLAY_ON);
	#endif
	#ifdef SET_BLE
    				bleWakeUp();
	#endif
    				Report(1, "Exit from SLEEP MODE\r\n");
    			break;
	#ifdef SET_BLE
    			case evt_WakeUp:
    				bleWakeUp();
    			break;
	#endif
    			case evt_Band:
    				Band = newBand;
    				if (!rda5807_Set_Band(Band)) {
    					sprintf(stb, "FM Band:%s", allBands[Band]);//(uint16_t)lBand, (uint16_t)rBand);
    					showLine(stb, lin3, &lit, true);
    					Report(1, "[que:%u] set new band=%u '%s'\r\n", cntEvt, Band, allBands[Band]);
    					if (next_evt == evt) {
    						if ((Freq < lBand) || (Freq > rBand)) {
    							newFreq = lBand;
    							evts.evt = evt_Freq;
    							if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 10) != osOK) devError |= devEVT;
    						}
    					} else {
    						next_evt = evt;
    						evts.evt = evt_Freq;
    						if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 10) != osOK) devError |= devEVT;
    					}
    				}
    			break;
    			case evt_Cfg:
    				showCfg();
    			break;
    			case evt_List:
    				next_evt = evt_Freq;
    				newFreq = getNextList(Freq, seek_up, &newBand);
					if (newBand == Band) {
						Report(1, "Band = newBand = %u -> goto set newFreq to %.3f (up = %u)\r\n", newBand, newFreq, seek_up);
						evts.evt = evt_Freq;
					} else {
						Report(1, "Band = %u -> goto set newBand to %u (newFreq to %.3f up = %u)\r\n", Band, newBand, newFreq, seek_up);
						evts.evt = evt_Band;
					}
					if (osMessageQueuePut(evtQueHandle, (const void *)&evts, prio, 10) != osOK) devError |= devEVT;
    			break;
    			case evt_Bass:
    				if (newBassBoost != BassBoost) {
    					BassBoost = newBassBoost;
    					rda5807_SetBassBoost(BassBoost);
    					//
    					if (noMute)
    						sprintf(st, "Bass:%u Vol:%u", BassBoost, Volume);
    					else
    						sprintf(st, "Bass:%u Vol:%u M", BassBoost, Volume);
    					showLine(st, lin4, &lim, true);
    					Report(1, "[que:%u] set new BassBoost to %u\r\n", cntEvt, BassBoost);
    				}
    			break;
    			case evt_Vol:
    				if (newVolume != Volume) {
    					Volume = newVolume;
    					rda5807_SetVolume(Volume);
    					//
    					if (noMute)
    						sprintf(st, "Bass:%u Vol:%u", BassBoost, Volume);
    					else
    						sprintf(st, "Bass:%u Vol:%u M", BassBoost, Volume);
    					showLine(st, lin4, &lim, true);
    					Report(1, "[que:%u] set new Volume to %u\r\n", cntEvt, Volume);
    				}
    			break;
    			case evt_Mute:
    				noMute = (~noMute) & 1;
    				rda5807_Set_Mute(noMute);
    				//
    				if (noMute)
    					sprintf(st, "Bass:%u Vol:%u", BassBoost, Volume);
    				else
    					sprintf(st, "Bass:%u Vol:%u M", BassBoost, Volume);
    				showLine(st, lin4, &lim, true);
    				Report(1, "[que:%u] set Mute to %u\r\n", cntEvt, (~noMute) & 1);
    			break;
    			case evt_Freq:
    				if ((newFreq >= lBand) && (newFreq <= rBand)) {
    					if (newFreq != Freq) {
    						Freq = newFreq;
    						uint16_t fr = (uint16_t)(Freq * 10);
    						rda5807_SetFreq_In100Khz(fr);
    						stereo = rda5807_Get_StereoMonoFlag();
    						Chan = rda5807_Get_Channel();
    						//
    						if (stereo)
    							sprintf(st, "Sig:%u Mhz:%.2f S", RSSI, Freq);
    						else
    							sprintf(st, "Sig:%u Mhz:%.2f", RSSI, Freq);
    						showLine(st, lin5, &lil, false);

    						sprintf(sta, "%s", nameStation(Freq));
    						Report(1, "[que:%u] set new Freq to %.3f МГц '%s' (Chan:%u)\r\n", cntEvt, Freq, sta, Chan);
    						showLine(sta, lin6, &lia, true);
#ifdef SET_RDS
    						if (rdsFlag) {
    							rds_init();
    							rdsTime = get_mstmr(rdsWait);
    						}
#endif
    					}
    				}
				break;
    			case evt_Scan:
    				if (!scan) {
    					scan = 1;
    					rda5807_StartSeek(seek_up);
    				}
    			break;
    			case evt_Ver:
    				Report(1, "Ver.%s\r\n", ver);
    			break;
    			case evt_Sec:
    			{
	#ifdef SET_DISPLAY
    				dl = sec2str(st);
    				x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    				ST7565_Print(x, lin1, st, lfnt, 0, PIX_OFF);
	#endif
    				//
    				if (scan) {
    					if (rda5807_Get_SeekTuneReadyFlag()) {
    						Freq = (float)rda5807_GetFreq_In100Khz();
    						Freq /= 10;
    						scan = 0;
    						Chan = rda5807_Get_Channel();
    						sprintf(sta, "%s", nameStation(Freq));
    						showLine(sta, lin6, &lia, true);
    						Report(1, "[que:%u] set new Freq to %.3f МГц %s (Chan:%u)\r\n", cntEvt, Freq, sta, Chan);
    						//
#ifdef SET_RDS
    						if (rdsFlag) {
    							rds_init();
    							rdsTime = get_mstmr(rdsWait);
    						}
#endif
    					}
    				}
    				//
    				uint16_t rssi = rda5807_rssi();
    				if (rssi != RSSI) {
    					RSSI = rssi;
    					stereo = rda5807_Get_StereoMonoFlag();
	#ifdef SET_DISPLAY
    					if (stereo)
    						sprintf(st, "Sig:%u Mhz:%.2f S", RSSI, Freq);
    					else
    						sprintf(st, "Sig:%u Mhz:%.2f", RSSI, Freq);
    					showLine(st, lin5, &lil, false);

    					//sprintf(sta, "'%s'", nameStation(Freq));
    					//showLine(sta, lin6, &lia, true);
    					////Report(1, "ChipID:0x%x Chan:%u Freq:%.2f RSSI:%u\r\n", rdaID, Chan, Freq, RSSI);
	#endif
    				}
    				//
    				if (devError) {
    					dl = sprintf(tmp, "devError : 0x%04X", devError);
    					lastErr = devError;
    				} else {
    					if (lastErr) {
    						dl = sprintf(tmp, "Ver.%s", ver);
    						lastErr = devOK;
    					} else dl = 0;
    				}
	#ifdef SET_DISPLAY
    				if (dl) {
    					ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - lfnt->FontHeight, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_OFF);
    					x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    					ST7565_Print(x, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);
    					ST7565_Update();
    				}
    				//
    				ST7565_Update();
	#endif
    			}
    			break;
    			case evt_Evt:
    				Report(1, "evtQueue counter : %u\r\n", getQueCount(evtQueHandle));
    			break;
    			case evt_Ack:
    				Report(1, "ackQueue counter : %u\r\n", getQueCount(ackQueHandle));
    			break;
    			case evt_Cmd:
    				Report(1, "cmdQueue counter : %u\r\n", getQueCount(cmdQueHandle));
    			break;
    			case evt_Clr:
    				devError = devOK;
    				Report(1, "[que:%u] Clear all Errors...\r\n", cntEvt);
    			break;
    			case evt_Help:
    				stx[0] = '\0';
    				for (int8_t i = 0; i < MAX_CMDS; i++) sprintf(stx+strlen(stx), "\t%s\r\n", s_cmds[i]);
    				Report(0, "%s", stx);
    			break;
    			case evt_Restart:
    				restart = 1;
    				Report(1, "[que:%u] Restart system...\r\n", cntEvt);
    			break;
    			case evt_Epoch:
    				set_Date(epoch);
    				Report(1, "[que:%u] Set Unix TimeStamp to %lu\r\n", cntEvt, epoch);
    			break;
    			case evt_Err:
    				Report(1, "[que:%u] Error input from uart\r\n", cntEvt);
    			break;
	#ifdef SET_W25FLASH
    			case evt_sRead:
    			case evt_sNext:
    			{
    				uint32_t w25_adr = (adr_sector * W25qxx_getSectorSize()) + offset_sector;
    				uint32_t dlin = list_sector;
    				int step = 32;
    				uint32_t ind = 0;
    				W25qxx_ReadSector(fs_work, adr_sector, offset_sector, dlin);
    				Report(0, "Read sector:%d offset:%d len:%u\r\n", adr_sector, offset_sector, dlin);
    				while (ind < dlin) {
    					strf[0] = '\0';
    					while (1) {
    						sprintf(strf+strlen(strf), "%06X ", (unsigned int)w25_adr);
    						for (int i = 0; i < step; i++) sprintf(strf+strlen(strf), " %02X", fs_work[i + ind]);
    						strcat(strf, "\r\n");
    						w25_adr += step;
    						ind += step;
    						if (!(ind % W25qxx_getPageSize())) break;
    					}
    					Report(0, "%s", strf);
    				}
    			}
    			break;
    			case evt_sWrite:
    			{
    				uint32_t ss = W25qxx_getSectorSize();
    				if (!W25qxx_IsEmptySector(adr_sector, 0, ss)) W25qxx_EraseSector(adr_sector);
    				memset(fs_work, byte_write, ss);
    				if (len_write != -1) ss = len_write;
    				W25qxx_WriteSector(fs_work, adr_sector, offset_sector, ss);
    				Report(0, "Fill sector:%d byte:%02X len:%d done\r\n", adr_sector, byte_write, ss);
    			}
    			break;
    			case evt_sErase:
    				if (adr_sector == -1) {
    					Report(1, "Erase flash");
    					flag_sector = true;
    					btime = HAL_GetTick();
    				} else {
    					W25qxx_EraseSector(adr_sector);
    					Report(1, "Erase sector:%d done\r\n", adr_sector);
    				}
    			break;
	#endif
    		}
    		if ((evt >= evt_sRead) && (evt <= evt_sWrite)) {
    			last_cmd_sector =  evt;//cmd_sector;
    			cmd_sector = sNone;
    		}
    	}//evtQueueGet(....)


#ifdef SET_W25FLASH
    	if (flag_sector) {
    		adr_sector++;
    		if (adr_sector >= W25qxx_getSectorCount()) {
    			flag_sector = false;
    			etime = HAL_GetTick();
    			Report(0, " done (%lu sec)\r\n", (etime - btime) / 1000);
    		} else {
    			W25qxx_EraseSector(adr_sector);
    			if (!(adr_sector % 8)) Report(0, ".");
    		}
    	}
#endif


#if defined(SET_BLE) || defined(SET_AUDIO)
    	if (ackQueHandle) {
    		if (!osMessageQueueGet(ackQueHandle, (void *)&_ack, NULL, 1)) {
    			if (_ack.msg) {
    				strcpy(bleRxBuf, _ack.msg);
    				free(_ack.msg);//vPortFree(_ack.msg);
    				Report(1, "[BLE_rx] %s\r\n", bleRxBuf);
    			}
    		}
    	}
    	if (cmdQueHandle) {
    		if (!osMessageQueueGet(cmdQueHandle, (void *)&_cmd, NULL, 1)) {
    			if (_cmd.msg) {
    				strcpy(bleBuf, _cmd.msg);
    				free(_cmd.msg);
    				strcat(bleBuf, "\r\n");
    				bleWrite(bleBuf, 1);
    			}
    		}
    	}
#endif
    	//
    	//
    	//
#ifdef SET_RDS
    	if (rdsFlag && rdsTime) {
    		if (check_mstmr(rdsTime)) {
    			rdsTime = get_mstmr(rdsWait);
    			if (!readyRds && rda5807_Get_RDSReady()) {
    				memset(dataRDS, 0, sizeof(dataRDS));
    				blk = (blocks_t *)&dataRDS;
    				blk->blockA = rda5807_Get_reg(RDA5807M_REG_BLOCK_A);
    				//Report(1, "[RDS] %04X %04X %04X %04X\r\n", blk->blockA, blk->blockB, blk->blockC, blk->blockD);
    				//

    				// Сравним содержимое блока A (ID станции) с предыдущим значением
    				if (blk->blockA == MaybeThisIDIsReal) {
    					if (IDRepeatCounter < REPEATS_TO_BE_REAL_ID) {
    						IDRepeatCounter++; // Значения совпадают, отразим это в счетчике
    						if (IDRepeatCounter == REPEATS_TO_BE_REAL_ID) sID = MaybeThisIDIsReal;// Определились с ID станции
    				    }
    				} else {
    					IDRepeatCounter = 0; // Значения не совпадают, считаем заново
    					MaybeThisIDIsReal = blk->blockA;
    				}
    				//
    				if (!sID || (blk->blockA != sID)) {//Пока не определимся с ID, разбирать RDS не будем
    												   //ID не совпадает. Пропустим эту RDS группу
    					continue;
    				}
    				// ID станции не скачет, вероятность корректности группы в целом выше
    				//Report(1, "[RDS] ID:0x%X\r\n", sID);
    				//
    				errLevelB = rda5807_Get_reg(0x0B) & RDA5807M_BLERB_MASK;
    				if (errLevelB < 3) {
    					// Блок B корректный, можем определить тип и версию группы
    					blk->blockB = rda5807_Get_reg(RDA5807M_REG_BLOCK_B);
    					if (!PTy_printed) { // Но сначала считаем PTy
    						if (PTy == (blk->blockB & RDS_ALL_PTY_MASK) >> RDS_ALL_PTY_SHIFT) {
    							Report(1, "[RDS] PlayType: %s\r\n", namePTy[PTy]);
    							PTy_printed = true;
    						} else {
    							PTy = (blk->blockB & RDS_ALL_PTY_MASK) >> RDS_ALL_PTY_SHIFT;
    				        }
    				    }
    					groupType = (blk->blockB & RDS_ALL_GROUPTYPE_MASK) >> RDS_ALL_GROUPTYPE_SHIFT;
    					groupVer = (blk->blockB & RDS_ALL_GROUPVER) > 0;
    					uint16_t reg10 = rda5807_Get_reg(RDA5807M_REG_BLER_CD);//getRegister(RDA5807M_REG_BLER_CD);
    					errLevelC = (reg10 & RDA5807M_BLERC_MASK) >> RDA5807M_BLERC_SHIFT;
    					errLevelD = (reg10 & RDA5807M_BLERD_MASK) >> RDA5807M_BLERD_SHIFT;
    					// ************* 0A, 0B - PSName, PTY ************
    					if ((groupType == 0) && (errLevelD < 3)) {
    						// Сравним новые символы PSName со старыми:
    						blk->blockD = rda5807_Get_reg(RDA5807M_REG_BLOCK_D);
    						char c = blk->blockD >> 8; // новый символ
    						uint8_t i = (blk->blockB & RDS_GROUP0_C1C0_MASK) << 1; // его позиция в PSName
    						if (PSName[i] != c) { // символы различаются
    							PSNameUpdated &= ~(1 << i);//!(1 << i); // сбросим флаг в PSNameUpdated
    							PSName[i] = c;
    						} else {// символы совпадают, установим флаг в PSNameUpdated:
    							PSNameUpdated |= 1 << i;
    				        }
    						// Аналогично для второго символа
    						c = blk->blockD & 255;
    						i++;
    						if (PSName[i] != c) {
    							PSNameUpdated &= ~(1 << i);//!(1 << i);
    							PSName[i] = c;
    				        } else {
    				        	PSNameUpdated |= 1 << i;
    				        }
    						// Когда все 8 флагов в PSNameUpdated установлены, считаем что PSName получено полностью
    						if (PSNameUpdated == 255) {
    							// Дополнительное сравнение с предыдущим значением
    							if (strcmp(PSName, PSName_prev) != 0) {
    								Report(1, "[RDS] Station: %s\r\n", PSName);
    								strcpy(PSName_prev, PSName);
    								//
#ifdef SET_DISPLAY
    								int lens = strlen(PSName);
    								if (lens > 15) lens = 15;
    								ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - lfnt->FontHeight, SCREEN_WIDTH - 1, lfnt->FontHeight, PIX_OFF);
    								int dl = sprintf(tmp, "RDS : %.*s", lens, PSName);
    								int x = ((SCREEN_WIDTH - (lfnt->FontWidth * dl)) >> 1) & 0x7f;
    								ST7565_Print(x, SCREEN_HEIGHT - lfnt->FontHeight, tmp, lfnt, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
    								ST7565_Update();
#endif
    								//
    				        	}
    				        }
    				    } // PSName, PTy end
    					// ******** 4A - Clock time and date ********
    					if ((groupType == 4) && (groupVer == 0) && (errLevelC < 3) && (errLevelD < 3)) {
    						blk->blockC = rda5807_Get_reg(RDA5807M_REG_BLOCK_C);
    						blk->blockD = rda5807_Get_reg(RDA5807M_REG_BLOCK_D);
    						uint16_t year;
    						uint8_t month, day;
    						unsigned long MJD = (blk->blockB & RDS_GROUP4A_MJD15_16_MASK);
    						MJD = (MJD << 15) | (blk->blockC >> RDS_GROUP4A_MJD0_14_SHIFT);
    						Report(1, "[RDS] Date: ");
    						if ((MJD < 58844) || (MJD > 62497)) {
    							Report(0, "decode error\r\n");
    				        } else {
    				        	MJDDecode(MJD, &year, &month, &day);
    				        	if ((day <= 31) && (month <= 12)) {
    				        		Report(0, "%02u.%02u.%04u\r\n", day, month, year);
    				            } else {
    				            	Report(0, "decode error\r\n");
    				            }
    				        }
    						uint8_t hours = (blk->blockC & RDS_GROUP4A_HOURS4_MASK) << 4;
    						hours |= (blk->blockD & RDS_GROUP4A_HOURS0_3_MASK) >> RDS_GROUP4A_HOURS0_3_SHIFT;
    						uint8_t minutes = (blk->blockD & RDS_GROUP4A_MINUTES_MASK) >> RDS_GROUP4A_MINUTES_SHIFT;
    						if ((hours > 23) || (minutes > 59))
    							Report(1, "[RDS] Time: decode error\r\n");
    				        else {
    				        	long timeInMinutes = hours * 60 + minutes;
    				        	//uint8_t LTO = blk->blockD & RDS_GROUP4A_LTO_MASK;
    				        	if (blk->blockD & RDS_GROUP4A_LTO_SIGN_MASK) {
    				        		timeInMinutes -= (blk->blockD & RDS_GROUP4A_LTO_MASK) * 30;
    				        		if (timeInMinutes < 0) timeInMinutes += 60 * 24;
    				            } else {
    				            	timeInMinutes += (blk->blockD & RDS_GROUP4A_LTO_MASK) * 30;
    				            	if (timeInMinutes > 60 * 24) timeInMinutes -= 60 * 24;
    				            }
    				        	hours = timeInMinutes / 60;
    				        	minutes = timeInMinutes % 60;
    				        	Report(1, "[RDS] Time: %02u:%02u\r\n", hours, minutes);
    				        }
    				    }
    				}
    			}
    			readyRds = rda5807_Get_RDSReady();
    		}
    	}
#endif
    	//
    	//
    	//
    	if (devError) {
    		errLedOn(true);
    		HAL_Delay(50);
    		errLedOn(false);
    		if (last_devError != devError) {
    			last_devError = devError;
    			tmp[0] = '\0';
    			uint16_t er = 0;
    			for (int8_t i = 0; i < MAX_ERR_CODE; i++) {
    				er = devError & all_devErr[i];
    				if (er) sprintf(tmp+strlen(tmp), " '%s'", errName(er));
    			}
    			Report(1, "Error 0x%04X %s\r\n", devError, tmp);
    		}
    	} else {
    		if (HAL_GPIO_ReadPin(ERR_LED_GPIO_Port, ERR_LED_Pin)) errLedOn(false);
    	}
    	//
    	//
    	//
    }//while (!restart)

    uint8_t sch = 3;
    while (!ird_exit && sch) {
    	HAL_Delay(1000);
    	sch--;
    }

    Report(1, "[que:%u] Stop application...\r\n", cntEvt);

    HAL_Delay(350);


    NVIC_SystemReset();


  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	else
	if (htim->Instance == TIM4) {
		msCounter++;//inc_msCounter();
		rec_evt_t ev = {cmdNone, 0};
		//
#ifdef SET_ENC
		if (!(msCounter % _300ms)) {
			Encoder = (encPort->Instance->CNT) >> 1;
			if (lastEncoder != Encoder) {
				if ((lastEncoder== MIN_ENC_VALUE) && (Encoder == MAX_ENC_VALUE)) ev.evt = cmdDecFreq;//dec
				else
				if ((lastEncoder == MAX_ENC_VALUE) && (Encoder == MIN_ENC_VALUE)) ev.evt = cmdIncFreq;//inc
				else
				if (lastEncoder < Encoder) ev.evt = cmdIncFreq;//inc
				else
				if (lastEncoder > Encoder) ev.evt = cmdDecFreq;//dec
				lastEncoder = Encoder;
				if (ev.evt != cmdNone)	{
					if (osMessageQueuePut(evtQueHandle, (const void *)&ev, 0, 0) != osOK) devError |= devEVT;
				}
			}
		}
#endif
		//
		if (!(msCounter % _1s)) {// 1 seconda
			secCounter++;
		  	HAL_GPIO_TogglePin(TIK_LED_GPIO_Port, TIK_LED_Pin);
#ifdef SET_DISPLAY
		  	if (startSec) {
		  		ev.evt = evt_Sec;
		  		if (osMessageQueuePut(evtQueHandle, (const void *)&ev, 0, 0) != osOK) devError |= devEVT;
		  	}
#endif
	  	}
	}
#ifdef SET_IRED
	else
	if (htim->Instance == TIM6) {
		uint8_t irdata = RECIV_PIN; // пин для приёма
		irparams.timer++;  // One more 50uS tick
		if (irparams.rawlen >= RAWBUF) irparams.rcvstate = STATE_OVERFLOW;  // Buffer overflow

		switch (irparams.rcvstate) {
			case STATE_IDLE: // In the middle of a gap
				if (irdata == MARK) {
					if (irparams.timer < GAP_TICKS) { // Not big enough to be a gap.
						irparams.timer = 0;
					} else {
						// Gap just ended; Record duration; Start recording transmission
						irparams.overflow = 0;
						irparams.rawlen  = 0;
						irparams.rawbuf[irparams.rawlen++] = irparams.timer;
						irparams.timer = 0;
						irparams.rcvstate = STATE_MARK;
					}
				}
			break;
			case STATE_MARK:  // Timing Mark
				if (irdata == SPACE) {// Mark ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_SPACE;
				}
			break;
			case STATE_SPACE:  // Timing Space
				if (irdata == MARK) {// Space just ended; Record time
					irparams.rawbuf[irparams.rawlen++] = irparams.timer;
					irparams.timer = 0;
					irparams.rcvstate = STATE_MARK;
				} else if (irparams.timer > GAP_TICKS) {// Space
					irparams.rcvstate = STATE_STOP;
				}
			break;
			case STATE_STOP:  // Waiting; Measuring Gap
			 	if (irdata == MARK) irparams.timer = 0;  // Reset gap timer
			break;
			case STATE_OVERFLOW:  // Flag up a read overflow; Stop the State Machine
				irparams.overflow = 1;
				irparams.rcvstate = STATE_STOP;
			break;
		}
		//
	}
#endif
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  devError |= devSYS;
	  errLedOn(true);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
