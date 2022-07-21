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
DMA_HandleTypeDef hdma_usart2_tx;

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
const char *ver = "1.1 21.07.22";//add new command 'list'



char stx[MAX_UART_BUF] = {0};
char tmp[128] = {0};
char cmdBuf[MAX_UART_BUF] = {0};
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

static uint32_t epoch = 1658432922;//1658402955;//1658326638;//1658248185;//1658240652;//1658227367;//1657985710;
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
	"bass:",
	"list"
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
	"BassBoost",
	"nextStation"
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
	volatile uint8_t cntEvt = 0;
#endif


//     Служебные переменные для внутренних модулей SPI
uint32_t spi_cnt = 0;
uint8_t spiRdy = 1;
//
#ifdef SET_W25FLASH
	/*const char *_read  = "read";
	const char *_write  = "write";
	const char *_erase  = "erase";
	const char *_next  = "next";*/
	int adr_sector = 0, offset_sector = 0, list_sector = 0, len_write = 0;
	int cmd_sector = sNone, last_cmd_sector = sNone;
	uint8_t byte_write = 0xff;
	bool flag_sector = false;
	unsigned char fs_work[_MAX_SS] = {0};
	char strf[1024] = {0};
	bool chipPresent = false;
	bool validChipID = false;
	//
	const char *noneStation = "???";
	static const rec_t list[MAX_LIST] = {
		{72.1, "Shanson"},// Шансон
		{93.6, "Radio_7"},// Радио 7
		{94.0, "ComedyRadio"},// Комеди Радио
		{95.1, "VestiFM"},// Вести ФМ
		{95.5, "RetroFM"},// Ретро ФМ
		{96.3, "RussianRadio"},// Русское Радио
		{97.0, "RadioBook"},// Радио Книга
		{97.9, "SilverRain"},// Серебрянный Дождь
		{98.5, "RadioENERGY"},// Радио Энергия
		{99.5, "RadioStar"},// Радио Звезда
		{100.1, "AutoRadio"},// АвтоРадио
		{100.6, "RussianArea"},// Русский Край
		{100.9, "Monte-Carlo"},// Монте-Карло
		{101.3, "OurtRadio"},// Наше Радио
		{101.8, "BussinessFM"},// Бизнес ФМ
		{102.5, "Маяк"},// Маяк
		{102.9, "LoveRadio"},// Любимое Радио
		{103.4, "Studio21"},// Студия 21
		{103.9, "RadioRussian"},// Радио России
		{104.5, "Europe+"},// Европа Плюс
		{105.2, "Baltic+"},// Балтик Плюс
		{105.9, "RoadRadio"},// Дорожное Радио
		{106.4, "RadioMaxim"},// Радио Максим
		{107.2, "Coms.True"}// Комсомольская Правда
	};

#endif

#ifdef SET_DISPLAY
	volatile bool startSec = false;
	FontDef_t *lfnt = NULL;
#endif

#if defined(SET_RDA_CHIP) || defined(SET_NEW_RDA)
	float Freq = 94.0;//96.3;//95.1;
	float newFreq = 95.1;
	float lBand = 0.0;
	float rBand = 0.0;
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
float getNextList(float fr);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */


  if (HAL_TIM_Base_Start_IT(tikPort) != HAL_OK) devError |= devTIK;

  for (int8_t i = 0; i < 4; i++) {
	  errLedOn(true);
	  HAL_Delay(150);
	  errLedOn(false);
	  HAL_Delay(150);
  }

  if (HAL_UART_Receive_IT(cmdPort, &rxByte, 1) != HAL_OK) devError |= devUART;

  set_Date(epoch);

  HAL_Delay(250);

  Report(1, "[que:%u] Start application ver.%s\r\n", cntEvt, ver);

#ifdef SET_W25FLASH
    chipPresent = W25qxx_Init();
    uint32_t cid = W25qxx_getChipID();
    if ( chipPresent && ((cid >= W25Q10) && (cid <= W25Q128)) ) validChipID = true;
    list_sector = W25qxx_getPageSize() << 1;//2;
#endif


#ifdef SET_RDA_CHIP

    RDA5807m_Reset();

    if (!(devError & devRDA)) rdaID = RDA5807m_ID();

    if (!(devError & devRDA)) RDA5807m_Init();

    if (!(devError & devRDA)) RDA5807m_SetBand(BAND_76_108);
    if (!(devError & devRDA)) RDA5807m_SetStep(STEP_100);
    if (!(devError & devRDA)) RDA5807m_SetFreq(Freq);
    if (!(devError & devRDA)) RDA5807m_Seek();

    if (!(devError & devRDA)) Chan = RDA5807m_GetChan();
    if (!(devError & devRDA)) RSSI = RDA5807m_GetRSSI();
    if (!(devError & devRDA)) Freq = RDA5807m_GetFreq();

#endif

#ifdef SET_NEW_RDA

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
  	uint16_t lin2 = lin1 + Font_6x8.FontHeight + 3;//chipID...
  	uint16_t lin3 = lin2 + Font_6x8.FontHeight;//Band...
  	uint16_t lin4 = lin3 + Font_6x8.FontHeight;//Volume...//Freq...
  	uint16_t lin5 = lin4 + Font_6x8.FontHeight;//Freq...//Volume...
  	uint16_t lin6 = lin5 + Font_6x8.FontHeight;//Station
  	char st[64];
  	char sta[64];

  	ST7565_Reset();
  	ST7565_Init();

  	ST7565_CMD_DISPLAY(CMD_DISPLAY_ON);

    int dl = sprintf(tmp, "Ver.%s", ver);
    uint16_t x = ((SCREEN_WIDTH - (Font_6x8.FontWidth * dl)) >> 1) & 0x7f;
    ST7565_Print(x, SCREEN_HEIGHT - Font_6x8.FontHeight, tmp, &Font_6x8, 1, PIX_ON);//печатаем надпись с указаным шрифтом и цветом(PIX_ON-белый, PIX_OFF-черный)

	#if defined(SET_RDA_CHIP) || defined(SET_NEW_RDA)
    	int il = sprintf(st, "RDA5807 chipID:0x%x", rdaID);
    	uint16_t xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * il)) >> 1) & 0x7f;
    	if (!xf) xf = 1;
    	ST7565_Print(xf, lin2, st, &Font_6x8, 1, PIX_ON);

    	il = sprintf(st, "FM Band:%.1f-%.1f", lBand, rBand);
    	int lib = il;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * il)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin3, st, &Font_6x8, 1, PIX_ON);

    	int im = sprintf(st, "Vol:%u Bass:%u", Volume, BassBoost);
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

    	int ia = sprintf(sta, "'%s'", nameStation(Freq));
    	int lia = ia;
    	xf = ((SCREEN_WIDTH - (Font_6x8.FontWidth * ia)) >> 1) & 0x7f;
    	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
    	ST7565_Print(xf, lin6, sta, &Font_6x8, 1, PIX_ON);

    	Report(1, "ChipID:0x%x Chan:%u Freq:%.2f %s RSSI:%u Band:%.1f-%.1f Vol:%u BassEn:%u\r\n",
    			rdaID, Chan, Freq, sta, RSSI, lBand, rBand, Volume, BassBoost);
	#endif

    ST7565_DrawRectangle(0, Font_6x8.FontHeight, SCREEN_WIDTH - 1, SCREEN_HEIGHT - (Font_6x8.FontHeight << 1) - 2, PIX_ON);
    ST7565_DrawFilledRectangle(0, 0, SCREEN_WIDTH - 1, Font_6x8.FontHeight, PIX_ON);
    ST7565_Update();

    startSec = true;

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
    			case evt_List:
    				newFreq = getNextList(Freq);
    				putEvt(evt_Freq);
    			break;
    			case evt_Bass:
    				if (newBassBoost != BassBoost) {
    					BassBoost = newBassBoost;
    					rda5807_SetBassBoost(BassBoost);
    					//
    					sprintf(st, "Vol:%u Bass:%u", Volume, BassBoost);
    					showLine(st, lin4, &lib, true);
    					Report(1, "[que:%u] set new BassBoost to %u\r\n", cntEvt, BassBoost);
    				}
    			break;
    			case evt_Vol:
    				if (newVolume != Volume) {
    					Volume = newVolume;
    					rda5807_SetVolume(Volume);
    					//
    					sprintf(st, "Vol:%u BassEn:%u", Volume, BassBoost);
    					showLine(st, lin4, &lim, true);
    					Report(1, "[que:%u] set new Volume to %u\r\n", cntEvt, Volume);
    				}
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

    						sprintf(sta, "'%s'", nameStation(Freq));
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
    					if (rda5807_Get_SeekTuneReadyFlag()) {//RadioNewState(Idle, 10);
    						Freq = (float)rda5807_GetFreq_In100Khz();
    						Freq /= 10;
    						scan = 0;
    						Chan = rda5807_Get_Channel();
    						sprintf(sta, "'%s'", nameStation(Freq));
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
    					showLine(sta, lin6, &lia, true);
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
    				W25qxx_EraseSector(adr_sector);
    				Report(0, "Erase sector:%d done\r\n", adr_sector);
    				break;
#endif
    		}
    		if ((evt >= evt_sRead) && (evt <= evt_sWrite)) {
    			last_cmd_sector =  evt;//cmd_sector;
    			cmd_sector = sNone;
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

    }

    HAL_TIM_Base_Stop_IT(tikPort);

#ifdef SET_DISPLAY
    ST7565_Reset();
    ST7565_CMD_DISPLAY(CMD_DISPLAY_OFF);
#endif
#if defined(SET_RDA_CHIP) || defined(SET_NEW_RDA)

#endif
    Report(1, "[que:%u] Stop application...\r\n", cntEvt);

    HAL_Delay(500);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
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
  HAL_GPIO_WritePin(GPIOC, TIK_LED_Pin|ERR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS_Pin|SPI1_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : TIK_LED_Pin */
  GPIO_InitStruct.Pin = TIK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TIK_LED_GPIO_Port, &GPIO_InitStruct);

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
float getNextList(float fr)
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
		if (++ik == MAX_LIST) ik = 0;
		ret = list[ik].freq;
	} else {
		for (int8_t i = 0; i < MAX_LIST; i++) {
			if (list[i].freq > fr) {
				ik = i;
				ret = list[i].freq;
				break;
			}
		}
		if (ik == -1) ret = list[0].freq;
	}

	return ret;
}
//-------------------------------------------------------------------------------------------
void showLine(char *msg, uint16_t lin, int *lil, bool update)
{
int il = strlen(msg);

	//if (*lil > il) {
		ST7565_DrawFilledRectangle(2, lin, SCREEN_WIDTH - 4, lfnt->FontHeight, PIX_OFF);
	//}
	*lil = il;
	int xf = ((SCREEN_WIDTH - (lfnt->FontWidth * il)) >> 1) & 0x7f;
	if ((!xf) || (xf > (SCREEN_WIDTH - 3))) xf = 1;
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
//         Функция приводит к верхнему регистру все символы строки
//
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
size_t len = MAX_UART_BUF;
char *buf = &cmdBuf[0];
uint32_t cnt = 16;
uint32_t stim = HAL_GetTick();

	while (!uartRdy && cnt) {
		if (HAL_GetTick() - stim) cnt--;
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
}
//-------------------------------------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		rxBuf[rxInd++] = (char)rxByte;
		if (rxByte == 0x0a) {//end of line
			rxBuf[--rxInd] = '\0';

			int i, ev = -1;
			if (strlen(rxBuf) > 2) {
				for (i = 0; i < MAX_CMDS; i++) {
					if (!strncmp(rxBuf, s_cmds[i], strlen(s_cmds[i]))) {
						char *uk = rxBuf + strlen(s_cmds[i]);
						ev = -1;
						switch (i) {
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
								seek_up = 1;
								ev = i;
								char *uki = strchr(uk, ':');
								if (uki) {
									if (*(char *)(uki + 1) == '0') seek_up = 0;
								}
							break;
							case cmdClr://"clr"
							case cmdHelp://"help"
							case cmdVer://"ver"
							case cmdList:
								ev = i;
							break;
							case cmdRestart://"restart" -> restart = 1;
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
									if ((sek >= 0) && (sek < W25qxx_getSectorCount())) {
										adr_sector = sek;
										offset_sector = 0;
										ev = i;//flag_sector = true;
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
				//
				if (ev == -1) ev = cmdErr;
				putEvt(ev);
				//
			}

			rxInd = 0;
			*rxBuf = '\0';
		}

		if (HAL_UART_Receive_IT(huart, &rxByte, 1) != HAL_OK) devError |= devUART;
	}
}
//-------------------------------------------------------------------------------------------
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
		devError |= devUART;
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
#if defined(SET_RDA_CHIP) || defined(SET_NEW_RDA)
	if (hi2c->Instance == I2C1) {
		i2cRdy = 1;
	}
#endif
}
//--------------------------------------------------------------------------------------------
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
#if defined(SET_RDA_CHIP) || defined(SET_NEW_RDA)
	if (hi2c->Instance == I2C1) {
		devError |= devRDA;
	}
#endif
}
//--------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin != KEY0_Pin) && (GPIO_Pin != KEY1_Pin)) return;

	if (GPIO_Pin == KEY0_Pin) seek_up = 1;
	else
	if (GPIO_Pin == KEY1_Pin) seek_up = 0;

	putEvt(cmdScan);
}
//--------------------------------------------------------------------------------------------


//*******************************************************************************************

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
