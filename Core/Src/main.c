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

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_tx;

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
const char *ver = "1.5.3 28.07.22";// add ON/OFF dislay pin




char stx[MAX_UART_BUF] = {0};
char tmp[128] = {0};
char cmdBuf[MAX_UART_BUF] = {0};
char strf[MAX_UART_BUF] = {0};
uint16_t devError = HAL_OK;

volatile static uint32_t secCounter = 0;//period 1s
volatile static uint64_t msCounter = 0;//period 250ms

TIM_HandleTypeDef *tikPort = &htim4;
SPI_HandleTypeDef *portFLASH = &hspi2;
SPI_HandleTypeDef *portLED = &hspi1;
I2C_HandleTypeDef *portI2C = &hi2c1;


UART_HandleTypeDef *cmdPort = &huart2;
uint8_t uartRdy = 1;
uint8_t rxByte = 0;
uint16_t rxInd = 0;
char rxBuf[MAX_UART_BUF] = {0};
volatile uint8_t restart = 0;

static uint32_t epoch = 1659015162;//1659001909;
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
	"wakeup",
	"exitsleep",
	"sleep"
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
	"BleWakeUp",
	"ExitSleep",
	"Sleep"
};

#ifdef SET_FIFO_MODE
	static int evt_fifo[MAX_FIFO_SIZE] = {evt_None};
	uint8_t rd_evt_adr = 0;
	uint8_t wr_evt_adr = 0;
	uint8_t wr_evt_err = 0;
	uint8_t cnt_evt = 0;
	uint8_t max_evt = 0;
	bool lock_fifo = false;
	int evt = evt_None;
	int next_evt = evt_None;
	volatile uint8_t cntEvt = 0;
#endif


//     Служебные переменные для внутренних модулей SPI
uint32_t spi_cnt = 0;
uint8_t spiRdy = 1;
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

#ifdef SET_RDA_CHIP
	float Freq = 76;//94.0;//96.3;//95.1;
	float newFreq = 95.1;
	float lBand = 0.0;
	float rBand = 0.0;
	uint8_t Band = 2;// :2
	uint8_t newBand = 2;
	uint16_t Chan = 0;
	uint16_t RSSI = 0;
	volatile uint8_t i2cRdy = 1;
	uint8_t rdaID = 0;
	volatile uint8_t scan = 0;
	volatile uint8_t seek_up = 1;
	uint8_t Volume = 8;
	uint8_t newVolume = 5;
	uint8_t BassBoost = 0;
	uint8_t newBassBoost = 0;
	bool stereo = false;
	uint8_t noMute = 1;
	//
	const char *noneStation = "???";
	static const rec_t def_list[MAX_LIST] = {
		//Band:3 65-76
		{3, 68.5, "Маяк"},// Маяк
		{3, 72.1, "Шансон"},// Шансон
		//Band:2,1 76-108, 87-108
		{2, 93.6, "Радио_7"},// Радио 7
		{2, 94.0, "Комеди_Радио"},// Комеди Радио
		{2, 95.1, "Вести_ФМ"},// Вести ФМ
		{2, 95.5, "Ретро_ФМ"},// Ретро ФМ
		{2, 96.3, "Русское_Радио"},// Русское Радио
		{2, 97.0, "Радио_Вера"},// Радио Книга
		{2, 97.9, "Серебр.Дождь"},// Серебрянный Дождь
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

#endif


#ifdef SET_BLE
	UART_HandleTypeDef *blePort = &huart3;
	uint8_t bleRdy = 1;
	uint8_t rxbByte = 0;
	uint16_t rxbInd = 0;
	char rxbBuf[MAX_BLE_BUF] = {0};
	char txbBuf[MAX_BLE_BUF] = {0};
	char bleBuf[MAX_BLE_BUF] = {0};
	char bleRxBuf[MAX_BLE_BUF] = {0};
	uint8_t ble_withDMA = 1;
	volatile uint8_t bleReady = 1;
	uint8_t ble_stat = 0;
	uint8_t adone = 0;
	//
	s_recq_t bleQueAck;
	s_recq_t bleQueCmd;
	bool bleQueAckFlag = false;
	bool bleQueCmdFlag = false;
	const char *ble_statName[] = {"Disconnected", "Connected"};
#endif

#ifdef SET_SLEEP
	bool sleep_mode = false;
	//uint32_t tms = 0;
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
/* USER CODE BEGIN PFP */

#ifdef SET_FIFO_MODE
	uint8_t getEvtCount();
	void putEvt(int evt);
	int getEvt();
#endif
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

	if (prn) Report(1, "[BLE] %s", str);

}
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
bool initRECQ(s_recq_t *q)
{
	q->put = q->get = 0;
	q->lock = 0;
	for (uint8_t i = 0; i < MAX_QREC; i++) {
		q->rec[i].id = i;
		q->rec[i].adr = NULL;
	}

	return true;
}
//-------------------------------------------------------------------------------------------
bool clearRECQ(s_recq_t *q)
{
	while (q->lock) {}
	q->lock = 1;

	q->put = q->get = 0;
	for (uint8_t i = 0; i < MAX_QREC; i++) {
		q->rec[i].id = i;
		free(q->rec[i].adr);
		q->rec[i].adr = NULL;
	}

	q->lock = 0;

	return false;
}
//-------------------------------------------------------------------------------------------
int8_t putRECQ(char *adr, s_recq_t *q)
{
int8_t ret = -1;

	while (q->lock) {}
	q->lock = 1;

	if (q->rec[q->put].adr == NULL) {
		q->rec[q->put].adr = adr;
		ret = q->rec[q->put].id;
		q->put++;   if (q->put >= MAX_QREC) q->put = 0;
	}

	q->lock = 0;

	return ret;
}
//-------------------------------------------------------------------------------------------
int8_t getRECQ(char *dat, s_recq_t *q)
{
int8_t ret = -1;
int len = 0;

	while (q->lock) {}
	q->lock = 1;

	if (q->rec[q->get].adr != NULL) {
		len = strlen(q->rec[q->get].adr);
		ret = q->rec[q->get].id;
		if (dat) memcpy(dat, q->rec[q->get].adr, len);
		free(q->rec[q->get].adr);
		q->rec[q->get].adr = NULL;
	}

	if (ret >= 0) {
		if (dat) *(dat + len) = '\0';
		q->get++;   if (q->get >= MAX_QREC) q->get = 0;
	}

	q->lock = 0;

	return ret;
}
//-------------------------------------------------------------------------------------------

#endif

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
  /* USER CODE BEGIN 2 */


    if (HAL_TIM_Base_Start_IT(tikPort) != HAL_OK) devError |= devTIK;

    for (int8_t i = 0; i < 4; i++) {
    	errLedOn(true);
    	HAL_Delay(100);
    	errLedOn(false);
    	HAL_Delay(100);
    }

    if (HAL_UART_Receive_IT(cmdPort, &rxByte, 1) != HAL_OK) devError |= devUART;
#ifdef SET_BLE
    if (HAL_UART_Receive_IT(blePort, &rxbByte, 1) != HAL_OK) devError |= devBLE;
#endif

    set_Date(epoch);

    HAL_Delay(100);

    Report(1, "[que:%u] Start application ver.%s\r\n", cntEvt, ver);

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
    } else {//in sector	present any data
    	if (!(devError & devSPI)) {
    		W25qxx_ReadSector((uint8_t *)&list[0].band, cfgSector, 0, listSize);
    		Report(1, "Readed cfg_stations_data (%lu bytes) from cfgSector #%lu\r\n", listSize, cfgSector);
      	} else {
      		memcpy((uint8_t *)&list[0].band, (uint8_t *)&def_list[0].band, listSize);
      	}
    }
#endif


#ifdef SET_RDA_CHIP

    rdaID = rda5807_init(&Freq);
    RSSI = rda5807_rssi();
    rda5807_SetVolume(Volume);
    rda5807_SetBassBoost(BassBoost);
    stereo = rda5807_Get_StereoMonoFlag();
    Chan = rda5807_Get_Channel();

#endif

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
    uint16_t x = ((SCREEN_WIDTH - (Font_6x8.FontWidth * dl)) >> 1) & 0x7f;
    ST7565_Print(x, SCREEN_HEIGHT - Font_6x8.FontHeight, tmp, &Font_6x8, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)

	#ifdef SET_RDA_CHIP
    	int il = sprintf(st, "RDA5807 chipID:0x%x", rdaID);
    	uint16_t xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * il)) >> 1) & 0x7f;
    	if (!xf) xf = 1;
    	ST7565_Print(xf, lin2, st, &Font_6x8, 1, PIX_ON);

    	int it = sprintf(stb, "FM Band:%s", allBands[Band]);//(uint16_t)lBand, (uint16_t)rBand);
    	int lit = it;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * it)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin3, stb, &Font_6x8, 1, PIX_ON);

    	int im = sprintf(st, "Bass:%u Vol:%u", BassBoost, Volume);
    	int lim = im;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * im)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin4, st, &Font_6x8, 1, PIX_ON);

    	if (stereo)
    		il = sprintf(st, "Rssi:%u Freq:%.1f S", RSSI, Freq);
    	else
    		il = sprintf(st, "Rssi:%u Freq:%.1f", RSSI, Freq);
    	int lil = il;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * il)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin5, st, &Font_6x8, 1, PIX_ON);

    	int ia = sprintf(sta, "%s", nameStation(Freq));
    	int lia = ia;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * ia)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin6, sta, &Font_6x8, 1, PIX_ON);

    	Report(1, "ChipID:0x%x Chan:%u Freq:%.2f %s RSSI:%u Band:%s Vol:%u BassEn:%u\r\n",
    			rdaID, Chan, Freq, sta, RSSI, allBands[Band], Volume, BassBoost);
	#endif

    ST7565_DrawRectangle(0, Font_6x8.FontHeight, SCREEN_WIDTH - 1, SCREEN_HEIGHT - (Font_6x8.FontHeight << 1) - 2, PIX_ON);
    ST7565_DrawFilledRectangle(0, 0, SCREEN_WIDTH - 1, Font_6x8.FontHeight, PIX_ON);
    ST7565_Update();

    startSec = true;

#endif

#ifdef SET_BLE
    bleWakeUp();

    bleQueAckFlag   = initRECQ(&bleQueAck);
    bleQueCmdFlag   = initRECQ(&bleQueCmd);

    bleWrite("AT+RESET\r\n", 1);
    ble_stat = get_bleStat();
    Report(1, "[BLE] stat(%u) '%s'\r\n", ble_stat, ble_statName[ble_stat & 1]);
#endif


    uint16_t lastErr = devOK;

    putEvt(evt_Freq);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    while (!restart) {


#ifdef SET_FIFO_MODE
    	evt = getEvt();
    	if (evt != evt_None) {
    		cntEvt = getEvtCount();
    		if (evt != evt_Sec) {
    			Report(1, "[que:%u] get event '%s'\r\n", cntEvt, str_cmds[evt]);
#ifdef SET_DISPLAY
    			ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - Font_6x8.FontHeight, SCREEN_WIDTH - 1, Font_6x8.FontHeight, PIX_OFF);
    			dl = sprintf(tmp, "evt(%u) : %s", cntEvt, str_cmds[evt]);
    			x = ((SCREEN_WIDTH - (Font_6x8.FontWidth * dl)) >> 1) & 0x7f;
    			ST7565_Print(x, SCREEN_HEIGHT - Font_6x8.FontHeight, tmp, &Font_6x8, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)
    			ST7565_Update();
#endif
    		}
    		switch (evt) {
    			case evt_Sleep:
    				Report(1, "Going into SLEEP MODE...\r\n");// in 1 second\r\n");
#ifdef SET_BLE
    				bleWrite("AT+SLEEP1\r\n", 1);
#endif
#ifdef SET_DISPLAY
    				ST7565_CMD_DISPLAY(CMD_DISPLAY_OFF);
#endif
    				HAL_Delay(500);
    				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
    				sleep_mode = true;
    				//
    				HAL_SuspendTick();
    				HAL_PWR_EnableSleepOnExit();
    				HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    				HAL_ResumeTick();
    			break;
    			case evt_ExitSleep:
    				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
#ifdef SET_DISPLAY
    				ST7565_CMD_DISPLAY(CMD_DISPLAY_ON);
#endif
#ifdef SET_BLE
    				bleWakeUp();//putEvt(evt_WakeUp);
#endif
    				Report(1, "Exit from SLEEP MODE\r\n");
    			break;
    			case evt_WakeUp:
#ifdef SET_BLE
    				bleWakeUp();
#endif
    			break;
    			case evt_Band:
    				Band = newBand;
    				if (!rda5807_Set_Band(Band)) {
    					sprintf(stb, "FM Band:%s", allBands[Band]);//(uint16_t)lBand, (uint16_t)rBand);
    					showLine(stb, lin3, &lit, true);
    					Report(1, "[que:%u] set new band=%u '%s'\r\n", cntEvt, Band, allBands[Band]);
    					if (next_evt == evt) {
    						if ((Freq < lBand) || (Freq > rBand)) {
    							newFreq = lBand;
    							putEvt(evt_Freq);
    						}
    					} else {
    						next_evt = evt;
    						putEvt(evt_Freq);
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
						Report(1, "Band = newBand = %u -> goto set newFreq to %.1f (up = %u)\r\n", newBand, newFreq, seek_up);
    					putEvt(evt_Freq);
					} else {
						Report(1, "Band = %u -> goto set newBand to %u (newFreq to %.1f up = %u)\r\n", Band, newBand, newFreq, seek_up);
    					putEvt(evt_Band);
					}
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
    							sprintf(st, "Rssi:%u Freq:%.1f S", RSSI, Freq);
    						else
    							sprintf(st, "Rssi:%u Freq:%.1f", RSSI, Freq);
    						showLine(st, lin5, &lil, false);

    						sprintf(sta, "%s", nameStation(Freq));
    						showLine(sta, lin6, &lia, true);
    						Report(1, "[que:%u] set new Freq to %.1f %s (Chan:%u)\r\n", cntEvt, Freq, sta, Chan);
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
    				x = ((SCREEN_WIDTH - (Font_6x8.FontWidth * dl)) >> 1) & 0x7f;
    				ST7565_Print(x, lin1, st, &Font_6x8, 0, PIX_OFF);
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
    						Report(1, "[que:%u] set new Freq to %.1f %s (Chan:%u)\r\n", cntEvt, Freq, sta, Chan);
    					}
    				}
    				//
    				uint16_t rssi = rda5807_rssi();
    				if (rssi != RSSI) {
    					RSSI = rssi;
    					stereo = rda5807_Get_StereoMonoFlag();
#ifdef SET_DISPLAY
    					if (stereo)
    						sprintf(st, "Rssi:%u Freq:%.1f S", RSSI, Freq);
    					else
    						sprintf(st, "Rssi:%u Freq:%.1f", RSSI, Freq);
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
    					ST7565_DrawFilledRectangle(0, SCREEN_HEIGHT - Font_6x8.FontHeight, SCREEN_WIDTH - 1, Font_6x8.FontHeight, PIX_OFF);
    					x = ((SCREEN_WIDTH - (Font_6x8.FontWidth * dl)) >> 1) & 0x7f;
    					ST7565_Print(x, SCREEN_HEIGHT - Font_6x8.FontHeight, tmp, &Font_6x8, 1, PIX_ON);
    					ST7565_Update();
    				}
    				//
    				ST7565_Update();
#endif
    			}
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
    	}
#endif

#ifdef SET_W25FLASH
    	if (flag_sector) {
    		adr_sector++;
    		if (adr_sector >= W25qxx_getSectorCount()) {
    			flag_sector = false;
    			etime = HAL_GetTick();
    			Report(0, " done (%lu sec)\r\n", (etime - btime) / 1000);
    		} else {
    			//putEvt(evt_sErase);
    			W25qxx_EraseSector(adr_sector);
    			if (!(adr_sector % 8)) Report(0, ".");
    		}
    	}
#endif


#ifdef SET_BLE
    	if (bleQueAckFlag) {
    		if (getRECQ(bleRxBuf, &bleQueAck) >= 0) {
    			Report(1, "[BLE] %s\r\n", bleRxBuf);
    		}
    	}
    	//
    	if (bleQueCmdFlag) {//command to GSM module queue is ready
    		if (getRECQ(bleBuf, &bleQueCmd) >= 0) {
    			strcat(bleBuf, "\r\n");
    			bleWrite(bleBuf, 1);
    		}
    	}
#endif


    	if (devError) {
    		errLedOn(true);
    		HAL_Delay(50);
    		errLedOn(false);
    	} else {
    		if (HAL_GPIO_ReadPin(ERR_LED_GPIO_Port, ERR_LED_Pin)) errLedOn(false);
    	}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    }//while (!restart)

    HAL_TIM_Base_Stop_IT(tikPort);

/*#ifdef SET_DISPLAY
    ST7565_Reset();
    ST7565_CMD_DISPLAY(CMD_DISPLAY_OFF);
#endif*/


    Report(1, "[que:%u] Stop application...\r\n", cntEvt);

    HAL_Delay(250);

    NVIC_SystemReset();


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
  hi2c1.Init.Timing = 0x10909EEE;
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
  huart3.Init.BaudRate = 115200;
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
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_IRQn);
  /* DMA2_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel6_IRQn, 0, 0);
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
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_DC_Pin|BLE_WAKEUP_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : CPU_WAKEUP_Pin */
  GPIO_InitStruct.Pin = CPU_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CPU_WAKEUP_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pin : BLE_WAKEUP_Pin */
  GPIO_InitStruct.Pin = BLE_WAKEUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(BLE_WAKEUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_STAT_Pin */
  GPIO_InitStruct.Pin = BLE_STAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BLE_STAT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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


#ifdef SET_FIFO_MODE
//-------------------------------------------------------------------------------------------
uint8_t getEvtCount()
{
	return cnt_evt;
}
//-------------------------------------------------------------------------------------------
void putEvt(int evt)
{

	//while (lock_fifo);
	//lock_fifo = true;

	if (cnt_evt > (MAX_FIFO_SIZE - 3)) {
		devError |= devFIFO;
		//lock_fifo = false;
		return;
	}

	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(TIM4_IRQn);

	if (cnt_evt >= MAX_FIFO_SIZE) {
			wr_evt_err++;
		} else {
			evt_fifo[wr_evt_adr] = evt;
			cnt_evt++;
			if (wr_evt_adr < (MAX_FIFO_SIZE - 1) ) {
				wr_evt_adr++;
			} else  {
				wr_evt_adr = 0;
			}
			wr_evt_err = 0;
			if (cnt_evt > max_evt) max_evt = cnt_evt;
		}

		if (wr_evt_err) devError |= devFIFO;
				   else devError &= ~devFIFO;

		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		HAL_NVIC_EnableIRQ(USART2_IRQn);

		//lock_fifo = false;
}
//-------------------------------------------------------------------------------------------
int getEvt()
{
int ret = evt_None;

	//while (lock_fifo);
	//lock_fifo = true;

	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	if (cnt_evt) {
		ret = evt_fifo[rd_evt_adr];
		if (cnt_evt) cnt_evt--;
		if (rd_evt_adr < (MAX_FIFO_SIZE - 1) ) {
			rd_evt_adr++;
		} else {
			rd_evt_adr = 0;
		}
	}

	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	//lock_fifo = false;

	return ret;
}
//-------------------------------------------------------------------------------------------
#endif

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
#ifdef SET_BLE
	if(sleep_mode) return;
#endif

	size_t len = MAX_UART_BUF;
	char *buf = &cmdBuf[0];

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
		if (HAL_UART_Transmit_DMA(cmdPort, (uint8_t *)buf, strlen(buf)) != HAL_OK) devError |= devUART;
		while (!uartRdy) {} //HAL_Delay(1)

		va_end(args);

	//	free(buf);
	//} else {
	//	devError |= devMEM;
	//}

}
//------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		msCounter++;//inc_msCounter();
		/*
#ifdef SET_SLEEP
		if (sleep_mode) {
			if (!(msCounter % _200ms)) {
				if (HAL_GPIO_ReadPin(CPU_WAKEUP_GPIO_Port, CPU_WAKEUP_Pin) == GPIO_PIN_SET) {
					sleep_mode = false;
					HAL_PWR_DisableSleepOnExit();
					putEvt(cmdExitSleep);
				}
			}
		}
#endif
		*/
		if (!(msCounter % _1s)) {// 1 seconda
			secCounter++;
		  	HAL_GPIO_TogglePin(TIK_LED_GPIO_Port, TIK_LED_Pin);
#ifdef SET_DISPLAY
		  	if (startSec) putEvt(evt_Sec);
#endif
	  	}
	}
}
//--------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		uartRdy = 1;
	}
#ifdef SET_BLE
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
#ifdef SET_BLE
	else
	if (huart->Instance == USART3) {
		devError |= devBLE;
	}
#endif
}
//-------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef SET_BLE
	if (huart->Instance == USART3) {
		if ((rxbByte > 0x0D) && (rxbByte < 0x80)) {
			if (rxbByte >= 0x20) adone = 1;
			if (adone) rxbBuf[rxbInd++] = (char)rxbByte;
		}
		if (adone) {
		//rxbBuf[rxbInd++] = (char)rxbByte;
			if (rxbByte == 0x0a) {// '\n'
				//rxbBuf[--rxbInd] = '\0';
				if (bleQueAckFlag) {
					int len = strlen(rxbBuf);
					// Блок помещает в очередь ответов на команду очередное сообщение от модуля BLE
					char *from = (char *)calloc(1, len + 1);
					if (from) {
						memcpy(from, rxbBuf, len);
						if (putRECQ(from, &bleQueAck) < 0) {
							devError |= devQUE;
							free(from);
						} else {
							if (devError & devQUE) devError &= ~devQUE;
						}
					} else {
						devError |= devMEM;
					}
					//-----------------------------------------------------------------------------
				}
				rxbInd = 0;
				adone = 0;
				memset(rxbBuf, 0, sizeof(rxbBuf));
			}
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
//#ifdef SET_SLEEP
//				start_sleep = get_tmr(WAIT_BEFORE_SLEEP);
//#endif
#ifdef SET_BLE
				if ( (strstr(rxBuf, "at+")) || (strstr(rxBuf, "AT+")) ) {
					if (bleQueCmdFlag) {
						int len = strlen(rxBuf);
						// Блок помещает в очередь команд очередную команду модулю BLE
						char *to = (char *)calloc(1, len + 3);
						if (to) {
							memcpy(to, rxBuf, len);
							toUppers(to);
							if (putRECQ(to, &bleQueCmd) < 0) {
								devError |= devQUE;
								free(to);
							} else {
								if (devError & devQUE) devError &= ~devQUE;
							}
						} else {
							devError |= devMEM;
						}
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
								case cmdWakeUp://"wakeup"
								case cmdSleep://"sleep" -> goto sleep mode
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
#ifdef SET_BLE
				}
#endif
				//
				if (ev != -2) {
					if (ev == -1) ev = cmdErr;
					putEvt(ev);
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
#ifdef SET_RDA_CHIP
	if (hi2c->Instance == I2C1) {
		i2cRdy = 1;
	}
#endif
}
//--------------------------------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
#ifdef SET_RDA_CHIP
	if (hi2c->Instance == I2C1) {
		devError |= devRDA;
	}
#endif
}
//--------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#ifdef SET_SLEEP
	if (sleep_mode) {
		if ((HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_SET) ||
				(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_SET)) {
			sleep_mode = false;
			HAL_PWR_DisableSleepOnExit();
			putEvt(cmdExitSleep);
		}
		return;
	}
#endif
	if ((GPIO_Pin == KEY0_Pin) || (GPIO_Pin == KEY1_Pin)) {
		if (GPIO_Pin == KEY0_Pin) seek_up = 1;
		else
		if (GPIO_Pin == KEY1_Pin) seek_up = 0;
		putEvt(cmdScan);
	}
}
//--------------------------------------------------------------------------------------------


//*******************************************************************************************

/*********************************   SLEEP MODE   ***********************************
uint8_t Rx_data;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_IT(huart, &Rx_data, 1);
    str = "WakeUP from SLEEP by UART\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    str = "WakeUP from SLEEP by EXTI\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
    HAL_PWR_DisableSleepOnExit ();
}

int main ()
{
 ......
 ......
 HAL_UART_Receive_IT(&huart2, &Rx_data, 1);
 while (1)
  {
	  str = "Going into SLEEP MODE in 5 seconds\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
  	  HAL_Delay(5000);

//    Suspend Tick increment to prevent wakeup by Systick interrupt.
//	  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)
//
	  HAL_SuspendTick();

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);  // Just to indicate that the sleep mode is activated

	  HAL_PWR_EnableSleepOnExit ();

//	  Enter Sleep Mode , wake up is done once User push-button is pressed
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);



//	  Resume Tick interrupt if disabled prior to sleep mode entry
	  HAL_ResumeTick();

	  str = "WakeUP from SLEEP\r\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

	  for (int i=0; i<20; i++)
	  {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  HAL_Delay(100);
	  }

  }
}
*********************************************************************************************/

/* USER CODE END 4 */

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
