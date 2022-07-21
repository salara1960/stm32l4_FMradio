

#include "hdr.h"

#ifdef SET_RDA_CHIP

#include "main.h"
#include "rda5807m.h"


uint16_t RDA5807m_CTRL_REG = 0x0000;
uint16_t RDA5807m_CHAN_REG = 0x0000;

uint16_t RDA5807m_I2C_OutBuff[2];
uint16_t RDA5807m_I2C_InBuff[2];

uint8_t rda_with_dma = 1;
uint16_t chipADDR = CHIP_ADDR;
uint8_t reg = 0;
uint32_t rda_wait = 500;

//--------------------------------------------------------------------------------------------------

void RDA5807m_Reset()
{
	//RDA5807m_CTRL_REG = 0x00;
	RDA5807m_CTRL_REG = CTRL_SOFT_RST;
	//RDA5807m_CTRL_REG |= CTRL_SOFT_RST;

	RDA5807m_Send();
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_Init()
{
	RDA5807m_CTRL_REG = CTRL_DEFAULT;
    RDA5807m_CHAN_REG = CHAN_DEFAULT; // ��������� � ������ CHAN_DEFAULT
	RDA5807m_CHAN_REG |= (208 << 6) | CHAN_TUNE; // ���� ��������� �� ������� �����

	RDA5807m_Send();
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_Seek()
{
	RDA5807m_CTRL_REG = 0x0000;
	RDA5807m_CHAN_REG &=	 ~0xFFF0; // �� ������� �������� � ���, ��������� �������
	RDA5807m_CTRL_REG |= (CTRL_DEFAULT | CTRL_SEEK | CTRL_SEEKUP);

	RDA5807m_Send();
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_SetBand(unsigned char Band)
{
	//RDA5807m_CTRL_REG = 0x0000;
	RDA5807m_CTRL_REG = CTRL_DEFAULT;
	RDA5807m_CHAN_REG &= ~0x000C;
	RDA5807m_CHAN_REG |= Band;

	RDA5807m_Send();		
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_SetStep(unsigned char Step)
{
	//RDA5807m_CTRL_REG = 0x0000;
	RDA5807m_CTRL_REG = CTRL_DEFAULT;
	RDA5807m_CHAN_REG &= ~0x0003;
	RDA5807m_CHAN_REG |= Step;

	RDA5807m_Send();		
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_SetFreq(float Freq)
{
uint16_t FreqVal = 0;
float Step = 0.0;
	
	switch (RDA5807m_CHAN_REG & CHAN_STEP) {
		case STEP_200:
			Step = 0.2;
		break;
		case STEP_100:
			Step = 0.1;
		break;
		case STEP_50:
			Step = 0.05;
		break;
		case STEP_25:
			Step = 0.025;
		break;
	}
	
	switch (RDA5807m_CHAN_REG & CHAN_BAND) {
		case BAND_76_108:
			if ((Freq < 76.0) || (Freq > 108.0)) return;
			FreqVal = (int)((Freq - 76.0) / Step);
		break;
		case BAND_87_108:
			if ((Freq < 87.0) || (Freq > 108.0)) return;
			FreqVal = (int)((Freq - 87.0) / Step);
		break;
		case BAND_76_91:
			if ((Freq < 76.0) || (Freq > 91.0)) return;
			FreqVal = (int)((Freq - 76.0) / Step);
		break;
		case BAND_65_76:
			if((Freq < 65.0) || (Freq > 91.0)) return;
			FreqVal = (int)((Freq - 65.0) / Step);
		break;
	}
	
	//RDA5807m_CTRL_REG = 0x0000;
	RDA5807m_CTRL_REG = CTRL_DEFAULT;
	RDA5807m_CHAN_REG &= ~0xFFC0;
	RDA5807m_CHAN_REG |= (FreqVal << 6) | CHAN_TUNE;

	RDA5807m_Send();	
}
//--------------------------------------------------------------------------------------------------
uint16_t RDA5807m_ID()
{
uint8_t data[2] = {0};

	reg = 0;
	if (HAL_I2C_Mem_Read(portI2C, (chipADDR << 1), reg, sizeof(chipADDR), (uint8_t *)data, 2, rda_wait) != HAL_OK) devError |= devRDA;
#ifdef SET_RDA_DEBUG
	Report(1, "[%s] rx_mem: 0x%x 0x%x\r\n", __func__, data[0], data[1]);
#endif
	uint16_t ret;
	memcpy(&ret, data, 2);
	HAL_Delay(10);

	return HTONS(ret);
}
//--------------------------------------------------------------------------------------------------
void RDA5807m_Send()
{
	RDA5807m_I2C_OutBuff[0] = HTONS(RDA5807m_CTRL_REG);//SwapBytes(&RDA5807m_I2C_OutBuff[0], &RDA5807m_CTRL_REG);
	RDA5807m_I2C_OutBuff[1] = HTONS(RDA5807m_CHAN_REG);//SwapBytes(&RDA5807m_I2C_OutBuff[1], &RDA5807m_CHAN_REG);

#ifdef SET_RDA_DEBUG
	Report(1, "[%s] tx: 0x%04x 0x%04x\r\n", __func__, RDA5807m_I2C_OutBuff[0], RDA5807m_I2C_OutBuff[1]);
#endif

	if (rda_with_dma) {
		i2cRdy = 0;
		if (HAL_I2C_Master_Transmit_DMA(portI2C,
										(CHIP_ADDR << 1),
										(uint8_t *)&RDA5807m_I2C_OutBuff,
										sizeof(RDA5807m_I2C_OutBuff)) != HAL_OK) devError |= devRDA;
		while (!i2cRdy) {} //HAL_Delay(1)
	} else {
		if (HAL_I2C_Master_Transmit(portI2C,
									(CHIP_ADDR << 1),
									(uint8_t *)&RDA5807m_I2C_OutBuff,
									sizeof(RDA5807m_I2C_OutBuff),
									rda_wait) != HAL_OK) devError |= devRDA;
	}
	HAL_Delay(10);
}
//--------------------------------------------------------------------------------------------------
/*
void SwapBytes(uint16_t *Dst, uint16_t *Src) // Src is ABCD, Dst is CDAB
{
	uint16_t temp = ((*Src & 0x00FF) << 8);
	temp += ((*Src & 0xFF00) >> 8);
	*Dst = temp;
}
*/
//--------------------------------------------------------------------------------------------------
uint16_t RDA5807m_GetChan()
{
	memset(RDA5807m_I2C_InBuff, 0, sizeof(RDA5807m_I2C_InBuff));
	if (HAL_I2C_Master_Receive(portI2C,
								(CHIP_ADDR << 1),
								(uint8_t *)&RDA5807m_I2C_InBuff,
								sizeof(RDA5807m_I2C_InBuff),
								rda_wait) != HAL_OK) devError |= devRDA;
	uint16_t chan = HTONS(RDA5807m_I2C_InBuff[0]) & 0x3FF;
	HAL_Delay(10);
	
	return chan;
}
//--------------------------------------------------------------------------------------------------
uint16_t RDA5807m_GetRSSI()
{
	memset(RDA5807m_I2C_InBuff, 0, sizeof(RDA5807m_I2C_InBuff));
	if (HAL_I2C_Master_Receive(portI2C,
								(CHIP_ADDR << 1),
								(uint8_t *)&RDA5807m_I2C_InBuff,
								sizeof(RDA5807m_I2C_InBuff),
								rda_wait)  != HAL_OK) devError |= devRDA;
	uint16_t rssi = (HTONS(RDA5807m_I2C_InBuff[1]) & 0x7F00) >> 9;
	HAL_Delay(10);

	return rssi;
}
//--------------------------------------------------------------------------------------------------
float RDA5807m_GetFreq()
{
float Step = 0.0;
uint16_t Chan = 0;
uint8_t LowerFreq = 0;
	
	switch (RDA5807m_CHAN_REG & CHAN_STEP) {
		case STEP_100:
			Step = 0.1;
		break;
		case STEP_200:
			Step = 0.2;
		break;
		case STEP_50:
			Step = 0.05;
		break;
		case STEP_25:
			Step = 0.025;
		break;
	}
	
	switch (RDA5807m_CHAN_REG & CHAN_BAND) {
		case BAND_76_108:
			LowerFreq = 76;
		break;
		case BAND_87_108:
			LowerFreq = 87;
		break;
		case BAND_76_91:
			LowerFreq = 76;
		break;
		case BAND_65_76:
			LowerFreq = 65;
		break;
	}
	
	Chan = RDA5807m_GetChan();
	
	return Chan * Step + LowerFreq;
	
}
//--------------------------------------------------------------------------------------------------


#endif

