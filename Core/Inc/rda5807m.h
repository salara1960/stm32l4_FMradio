/*

RDA9807m.H - register definition for RDA9807m

*/

//#include "stm32f4xx.h"
// I2C_HANDLER - ����� �������� i2c

#include "hdr.h"

#ifdef SET_RDA_CHIP

#define SET_RDA_DEBUG


//#define I2C_HANDLER portI2C

// CTRL_REG (0x02) - transmitted first

#define CHIP_ADDR     0x0010

#define CTRL_ENABLE	  0x0001
#define CTRL_SOFT_RST 0x0002
#define CTRL_NEW_MTD  0x0004
#define	CTRL_RDS_EN	  0x0008
#define	CTRL_CLK	  0x0060 //Always 000
#define	CTRL_SKMODE	  0x0080
#define	CTRL_SEEK	  0x0100
#define	CTRL_SEEKUP	  0x0200
#define	CTRL_RCLK	  0x0400
#define	CTRL_RCLK_NON 0x0800
#define	CTRL_BASS	  0x1000
#define	CTRL_MONO	  0x2000
#define	CTRL_DMUTE	  0x4000
#define	CTRL_DHIZ	  0x8000

#define CTRL_DEFAULT (CTRL_DHIZ | CTRL_DMUTE | CTRL_BASS | CTRL_ENABLE)
#define CHAN_DEFAULT (CHAN_STEP & STEP_50) | (CHAN_BAND & BAND_76_108)

// CHAN_REG (0x03)

#define	CHAN_STEP	  0x0003
#define	CHAN_BAND	  0x000C
#define	CHAN_TUNE	  0x0010
#define	CHAN_DIR_MODE 0x0020
#define	CHAN_CHAN	  0xFFC0

#define BAND_87_108	  0x00
#define BAND_76_91	  0x04
#define BAND_76_108	  0x08
#define BAND_65_76	  0x0C

#define STEP_100	  0x00
#define STEP_200	  0x01
#define STEP_50		  0x02
#define STEP_25		  0x03


#define HTONS(x) ((uint16_t)((x >> 8) | ((x << 8) & 0xff00)))


//void SwapBytes(uint16_t *Dst,uint16_t *Src);
void RDA5807m_Reset();
uint16_t RDA5807m_ID();
void RDA5807m_Send();
void RDA5807m_Init();
void RDA5807m_SetFreq(float Freq);
void RDA5807m_Seek();
void RDA5807m_SetBand(unsigned char Band);
void RDA5807m_SetStep(unsigned char Step);

uint16_t RDA5807m_GetChan(); //Reads CHAN[9:0] from register 0x0A
uint16_t RDA5807m_GetRSSI(); //Reads RSSI[6:0] from register 0x0B
float RDA5807m_GetFreq();  	 //Converts CHAN, BAND and STEP to frequency

#endif
