
#include "hdr.h"

#ifdef SET_W25FLASH

#include "main.h"
#include "w25.h"

//------------------------------------------------------------------------------------------

const uint32_t min_wait_ms = 250;
const uint32_t max_wait_ms = 1000;

w25qxx_t w25qxx;
const char *all_chipID[] = {
	"???",//0
	"W25Q10",//1
	"W25Q20",//2
	"W25Q40",//3
	"W25Q80",//4
	"W25Q16",//5
	"W25Q32",//6
	"W25Q64",//7
	"W25Q128",//8
	"W25Q256",//9
	"W25Q512"//10
};
const uint32_t all_chipBLK[] = {
	0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024
};

uint8_t pageTmp[PAGE_BUF_SIZE + PAGE_HDR_BYTES + 1] = {0};
uint8_t w25_withDMA = 0;

extern void Report(const uint8_t addTime, const char *fmt, ...);

//------------------------------------------------------------------------------------------

void W25_SELECT()   { W25_SEL(); } //HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_RESET);//set to 0
void W25_UNSELECT() { W25_UNSEL(); } //HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_SET);//set to 1
//void W25_SELECT()   { HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_RESET); }//set to 0
//void W25_UNSELECT() { HAL_GPIO_WritePin(W25_CS_GPIO_Port, W25_CS_Pin, GPIO_PIN_SET); }//set to 1

//------------------------------------------------------------------------------------------
uint8_t W25qxx_Spi(uint8_t Data)
{
uint8_t ret;

    if (HAL_SPI_TransmitReceive(portFLASH, &Data, &ret, 1, min_wait_ms) != HAL_OK) devError |= devSPI;

    return ret;
}
//------------------------------------------------------------------------------------------
void W25qxx_Reset(void)
{
	W25qxx_Delay(100);

	W25_SELECT();
		W25qxx_Spi(EN_RESET);
		W25qxx_Spi(CHIP_RESET);
	W25_UNSELECT();

	W25qxx_Delay(100);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_ReadID(void)
{
uint32_t Temp[3] = {0};

    W25_SELECT();//set to 0

    W25qxx_Spi(JEDEC_ID);
    Temp[0] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    Temp[1] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    Temp[2] = W25qxx_Spi(W25QXX_DUMMY_BYTE);

    W25_UNSELECT();//set to 1

    return ((Temp[0] << 16) | (Temp[1] << 8) | Temp[2]);
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadUniqID(void)
{
	uint8_t dat[] = {READ_UID, W25QXX_DUMMY_BYTE, W25QXX_DUMMY_BYTE, W25QXX_DUMMY_BYTE, W25QXX_DUMMY_BYTE};
    W25_SELECT();

    //W25qxx_Spi(READ_UID);
    //for (uint8_t i = 0; i < 4; i++) W25qxx_Spi(W25QXX_DUMMY_BYTE);
    //for (uint8_t i = 0; i < 8; i++) w25qxx.UniqID[i] = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    if (HAL_SPI_Transmit(portFLASH, dat, sizeof(dat), min_wait_ms) != HAL_OK) devError |= devSPI;
    if (HAL_SPI_Receive(portFLASH, w25qxx.UniqID, 8, min_wait_ms) != HAL_OK) devError |= devSPI;

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteEnable(void)
{
    W25_SELECT();

    W25qxx_Spi(WRITE_ENABLE);

    W25_UNSELECT();

    W25qxx_Delay(1);
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteDisable(void)
{
    W25_SELECT();

    W25qxx_Spi(WRITE_DISABLE);

    W25_UNSELECT();

    W25qxx_Delay(1);
}
//------------------------------------------------------------------------------------------
uint8_t W25qxx_ReadStatusRegister(uint8_t SelectStatusReg)
{
uint8_t status = 0;

    W25_SELECT();

    switch (SelectStatusReg) {
        case 1:
            W25qxx_Spi(READ_STAT_REG1);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister1 = status;
        break;
        case 2:
            W25qxx_Spi(READ_STAT_REG2);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister2 = status;
        break;
        default : {
            W25qxx_Spi(READ_STAT_REG3);
            status = W25qxx_Spi(W25QXX_DUMMY_BYTE);
            w25qxx.StatusRegister3 = status;
        }
    }

    W25_UNSELECT();

    return status;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteStatusRegister(uint8_t SelectStatusReg, uint8_t Data)
{
    W25_SELECT();

    switch (SelectStatusReg) {
        case 1 :
            W25qxx_Spi(WRITE_STAT_REG1);
            w25qxx.StatusRegister1 = Data;
        break;
        case 2 :
            W25qxx_Spi(WRITE_STAT_REG2);
            w25qxx.StatusRegister2 = Data;
        break;
        default : {
            W25qxx_Spi(WRITE_STAT_REG3);
            w25qxx.StatusRegister3 = Data;
        }
    }

    W25qxx_Spi(Data);

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
void W25qxx_WaitForWriteEnd(void)
{
    W25qxx_Delay(1);

    W25_SELECT();

    W25qxx_Spi(READ_STAT_REG1);
    do
    {
        w25qxx.StatusRegister1 = W25qxx_Spi(W25QXX_DUMMY_BYTE);
        W25qxx_Delay(1);
    } while ((w25qxx.StatusRegister1 & 0x01) == 0x01);

    W25_UNSELECT();
}
//------------------------------------------------------------------------------------------
bool W25qxx_Init(void)
{

	W25qxx_Reset();


    w25qxx.Lock = 1;
    bool ret = false;

    W25_UNSELECT();

    uint32_t id = W25qxx_ReadID() & 0xffff;
//#ifdef W25QXX_DEBUG
    Report(1, "w25qxx Init Begin... Chip ID:0x%X\r\n", id);
//#endif
    id &= 0xff;
    id -= 0x10;//0x4010;
    if (id > 0x0a) id = 0;
    w25qxx.ID         = id;              //W25Q10..W25Q512
    w25qxx.BlockCount = all_chipBLK[id]; //0..1024;
//#ifdef W25QXX_DEBUG
    Report(1, "Chip %s\r\n", all_chipID[id]);
//#endif

    if (id) {
    	w25qxx.PageSize = 256;
    	w25qxx.SectorSize = 0x1000;
    	w25qxx.SectorCount = w25qxx.BlockCount * 16;
    	w25qxx.PageCount = (w25qxx.SectorCount * w25qxx.SectorSize) / w25qxx.PageSize;
    	w25qxx.BlockSize = w25qxx.SectorSize * 16;
    	w25qxx.CapacityInKiloByte = (w25qxx.SectorCount * w25qxx.SectorSize) / 1024;
    	W25qxx_ReadUniqID();
    	W25qxx_ReadStatusRegister(1);
    	W25qxx_ReadStatusRegister(2);
    	W25qxx_ReadStatusRegister(3);
    	ret = true;
//#ifdef W25QXX_DEBUG
    	Report(0,"\tPage Size:\t%u bytes\r\n"
                 "\tPage Count:\t%u\r\n"
                 "\tSector Size:\t%u bytes\r\n"
                 "\tSector Count:\t%u\r\n"
                 "\tBlock Size:\t%u bytes\r\n"
                 "\tBlock Count:\t%u\r\n"
                 "\tCapacity:\t%u KBytes\r\n",
                 w25qxx.PageSize,
                 w25qxx.PageCount,
                 w25qxx.SectorSize,
                 w25qxx.SectorCount,
                 w25qxx.BlockSize,
                 w25qxx.BlockCount,
                 w25qxx.CapacityInKiloByte);
//#endif
    }

    w25qxx.Lock = 0;

    return ret;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_getChipID()
{
	return (uint32_t)w25qxx.ID;
}
uint32_t W25qxx_getSectorCount()
{
	return w25qxx.SectorCount;
}
uint32_t W25qxx_getSectorSize()
{
	return w25qxx.SectorSize;
}
uint32_t W25qxx_getPageCount()
{
	return w25qxx.PageCount;
}
uint32_t W25qxx_getPageSize()
{
	return w25qxx.PageSize;
}
uint32_t W25qxx_getBlockCount()
{
	return w25qxx.BlockCount;
}
uint32_t W25qxx_getBlockSize()
{
	return w25qxx.BlockSize;
}
uint32_t W25qxx_getTotalMem()
{
	return w25qxx.CapacityInKiloByte;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseChip(void)
{
    while (w25qxx.Lock) W25qxx_Delay(1);//wait unlock device

    w25qxx.Lock = 1;//lock device

//#ifdef W25QXX_DEBUG
    uint32_t StartTime = HAL_GetTick();
    Report(1, "Begin...");
//#endif
    W25qxx_WriteEnable();

    W25_SELECT();

    W25qxx_Spi(CHIP_ERASE);

    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();
//#ifdef W25QXX_DEBUG
    uint32_t dur = HAL_GetTick() - StartTime;
    Report(0, " done after %u ms (%u sec)!\r\n", dur, dur / 1000);
//#endif
    W25qxx_Delay(10);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseSector(uint32_t SectorAddr)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

#ifdef W25QXX_DEBUG
    uint32_t StartTime = HAL_GetTick();
    Report(1, "%u Begin...", SectorAddr);
#endif

    W25qxx_WaitForWriteEnd();
    SectorAddr = SectorAddr * w25qxx.SectorSize;
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(SECTOR_ERASE);
    //if (w25qxx.ID >= W25Q256) W25qxx_Spi((SectorAddr & 0xFF000000) >> 24);
    W25qxx_Spi((SectorAddr & 0xFF0000) >> 16);
    W25qxx_Spi((SectorAddr & 0xFF00) >> 8);
    W25qxx_Spi(SectorAddr & 0xFF);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();

#ifdef W25QXX_DEBUG
    uint32_t dur = HAL_GetTick() - StartTime;
    Report(0, " done after %u ms (%u sec)!\r\n", dur, dur / 1000);
#endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_EraseBlock(uint32_t BlockAddr)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

#ifdef W25QXX_DEBUG
    Report(1, "%u Begin...", BlockAddr);
    W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
#endif

    W25qxx_WaitForWriteEnd();
    BlockAddr = BlockAddr * w25qxx.SectorSize * 16;
    W25qxx_WriteEnable();

    W25_SELECT();
    W25qxx_Spi(BLOCK64_ERASE);
    //if(w25qxx.ID >= W25Q256) W25qxx_Spi((BlockAddr & 0xFF000000) >> 24);
    W25qxx_Spi((BlockAddr & 0xFF0000) >> 16);
    W25qxx_Spi((BlockAddr & 0xFF00) >> 8);
    W25qxx_Spi(BlockAddr & 0xFF);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();

#ifdef W25QXX_DEBUG
    uint32_t dur = HAL_GetTick() - StartTime;
    Report(0, " done after %u ms (%u sec)!\r\n", dur, dur / 1000);
#endif
    W25qxx_Delay(1);

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_PageToSector(uint32_t PageAddress)
{
    return ((PageAddress * w25qxx.PageSize) / w25qxx.SectorSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_PageToBlock(uint32_t PageAddress)
{
    return ((PageAddress * w25qxx.PageSize) / w25qxx.BlockSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress)
{
    return ((SectorAddress * w25qxx.SectorSize) / w25qxx.BlockSize);
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_SectorToPage(uint32_t SectorAddress)
{
    return (SectorAddress * w25qxx.SectorSize) / w25qxx.PageSize;
}
//------------------------------------------------------------------------------------------
uint32_t W25qxx_BlockToPage(uint32_t BlockAddress)
{
    return (BlockAddress * w25qxx.BlockSize) / w25qxx.PageSize;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize)
{

    while(w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( ((NumByteToCheck_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) ||
            (!NumByteToCheck_up_to_PageSize) )
                        NumByteToCheck_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckPage:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Page_Address, Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize);
    //W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
#endif

    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for (i = OffsetInByte; i < w25qxx.PageSize; i += sizeof(pBuffer)) {

        W25_SELECT();
        WorkAddress = (i + Page_Address * w25qxx.PageSize);
        W25qxx_Spi(DATA_READ);//FAST_READ);
        //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        //W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), min_wait_ms);
        W25_UNSELECT();

        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.PageSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for ( ; i < w25qxx.PageSize; i++) {

        	W25_SELECT();
            WorkAddress = (i + Page_Address * w25qxx.PageSize);
            W25qxx_Spi(DATA_READ);//FAST_READ);
            //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            //W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, min_wait_ms);
            W25_UNSELECT();

            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckPage is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    //W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:

#ifdef W25QXX_DEBUG
	Report(1, "w25qxx CheckPage is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
	W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( (NumByteToCheck_up_to_SectorSize > w25qxx.SectorSize) || (!NumByteToCheck_up_to_SectorSize) )
                NumByteToCheck_up_to_SectorSize = w25qxx.SectorSize;

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckSector:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Sector_Address, Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize);
    //W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
#endif

    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for ( i = OffsetInByte; i < w25qxx.SectorSize; i += sizeof(pBuffer)) {

    	W25_SELECT();
        WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
        W25qxx_Spi(DATA_READ);//FAST_READ);
        //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        //W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), min_wait_ms);
        W25_UNSELECT();

        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if (pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.SectorSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for( ; i < w25qxx.SectorSize; i++) {

            W25_SELECT();
            WorkAddress = (i + Sector_Address * w25qxx.SectorSize);
            W25qxx_Spi(DATA_READ);//FAST_READ);
            //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            //W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, min_wait_ms);
            W25_UNSELECT();

            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckSector is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    //W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckSector is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    //W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( (NumByteToCheck_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToCheck_up_to_BlockSize )
                          NumByteToCheck_up_to_BlockSize = w25qxx.BlockSize;

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckBlock:0x%X(%u), Offset:%u, Bytes:%u begin...\r\n",
                 Block_Address, Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize);
    //W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
#endif

    uint8_t pBuffer[32];
    uint32_t i, WorkAddress;
    for (i = OffsetInByte; i < w25qxx.BlockSize; i += sizeof(pBuffer)) {

        W25_SELECT();
        WorkAddress = (i + Block_Address * w25qxx.BlockSize);
        W25qxx_Spi(DATA_READ);//FAST_READ);
        //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
        W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
        W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
        W25qxx_Spi(WorkAddress & 0xFF);
        //W25qxx_Spi(0);
        HAL_SPI_Receive(portFLASH, pBuffer, sizeof(pBuffer), min_wait_ms);
        W25_UNSELECT();

        for (uint8_t x = 0; x < sizeof(pBuffer); x++) {
            if(pBuffer[x] != 0xFF) goto NOT_EMPTY;
        }
    }
    if ((w25qxx.BlockSize + OffsetInByte) % sizeof(pBuffer) != 0) {
        i -= sizeof(pBuffer);
        for ( ; i < w25qxx.BlockSize; i++) {

            W25_SELECT();
            WorkAddress = (i + Block_Address * w25qxx.BlockSize);
            W25qxx_Spi(DATA_READ);//FAST_READ);
            //if (w25qxx.ID >= W25Q256) W25qxx_Spi((WorkAddress & 0xFF000000) >> 24);
            W25qxx_Spi((WorkAddress & 0xFF0000) >> 16);
            W25qxx_Spi((WorkAddress & 0xFF00) >> 8);
            W25qxx_Spi(WorkAddress & 0xFF);
            //W25qxx_Spi(0);
            HAL_SPI_Receive(portFLASH, pBuffer, 1, min_wait_ms);
            W25_UNSELECT();

            if (pBuffer[0] != 0xFF) goto NOT_EMPTY;
        }
    }

#ifdef W25QXX_DEBUG
    Report(1, "w25qxx CheckBlock is Empty in %u ms\r\n", HAL_GetTick() - StartTime);
    //W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return true;

NOT_EMPTY:

#ifdef W25QXX_DEBUG
	Report(1, "w25qxx CheckBlock is Not Empty in %u ms\r\n", HAL_GetTick() - StartTime);
	//W25qxx_Delay(100);
#endif

    w25qxx.Lock = 0;

    return false;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteByte(uint8_t pBuffer, uint32_t WriteAddr_inBytes)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

#ifdef W25QXX_DEBUG
    uint32_t StartTime = HAL_GetTick();
    Report(1, "%s 0x%02X at address %d begin...", __func__, pBuffer, WriteAddr_inBytes);
#endif

    W25qxx_WaitForWriteEnd();
    W25qxx_WriteEnable();

    uint8_t *uk = NULL;
    uint16_t len = 6;
    /*if (w25qxx.ID >= W25Q256) {
    	uint8_t dat[] = {PAGE_PROG,
    			(uint8_t)((WriteAddr_inBytes & 0xFF000000) >> 24),
				(uint8_t)((WriteAddr_inBytes & 0xFF0000) >> 16),
				(uint8_t)((WriteAddr_inBytes & 0xFF00) >> 8),
				(uint8_t)(WriteAddr_inBytes & 0xFF),
				pBuffer};
        uk = &dat[0];
    } else {*/
    	uint8_t dat[] = {PAGE_PROG,
    			(uint8_t)((WriteAddr_inBytes & 0xFF0000) >> 16),
				(uint8_t)((WriteAddr_inBytes & 0xFF00) >> 8),
				(uint8_t)(WriteAddr_inBytes & 0xFF),
				pBuffer};
    	uk = &dat[0];
        len--;
    /*}*/
    W25_SELECT();
    //W25qxx_Spi(PAGE_PROG);
    ////if (w25qxx.ID >= W25Q256) W25qxx_Spi((WriteAddr_inBytes & 0xFF000000) >> 24);
    //W25qxx_Spi((WriteAddr_inBytes & 0xFF0000) >> 16);
    //W25qxx_Spi((WriteAddr_inBytes & 0xFF00) >> 8);
    //W25qxx_Spi(WriteAddr_inBytes & 0xFF);
    //W25qxx_Spi(pBuffer);
    HAL_SPI_Transmit(portFLASH, uk, len, min_wait_ms);//100);
    W25_UNSELECT();

    W25qxx_WaitForWriteEnd();

#ifdef W25QXX_DEBUG
    Report(1, "%s done after %d ms\r\n", __func__, HAL_GetTick() - StartTime);
#endif

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ( ((NumByteToWrite_up_to_PageSize + OffsetInByte) > w25qxx.PageSize) || !NumByteToWrite_up_to_PageSize )
                NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;
    if ( (OffsetInByte + NumByteToWrite_up_to_PageSize) > w25qxx.PageSize )
                NumByteToWrite_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

#ifdef W25QXX_DEBUG
    Report(1, "%s WritePage:0x%X(%u), Offset:%u ,Writes %u Bytes, begin...\r\n",
                 __func__, Page_Address, Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize);
    uint32_t StartTime = HAL_GetTick();
#endif

    W25qxx_WaitForWriteEnd();
    W25qxx_WriteEnable();

    Page_Address = (Page_Address * w25qxx.PageSize) + OffsetInByte;

    /*W25_SELECT();
    W25qxx_Spi(PAGE_PROG);
    //if (w25qxx.ID >= W25Q256) W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Page_Address & 0xFF00) >> 8);
    W25qxx_Spi(Page_Address&0xFF);
    HAL_SPI_Transmit(portFLASH, pBuffer, NumByteToWrite_up_to_PageSize, min_wait_ms);
    W25_UNSELECT();*/

    uint16_t lens = NumByteToWrite_up_to_PageSize + PAGE_HDR_BYTES;
    int idx = 0;
    pageTmp[idx++] = PAGE_PROG;
    //if (w25qxx.ID >= W25Q256) pageTmp[idx++] = (Page_Address & 0xFF000000) >> 24;
    pageTmp[idx++] = (Page_Address & 0xFF0000) >> 16;
    pageTmp[idx++] = (Page_Address& 0xFF00) >> 8;
    pageTmp[idx++] = Page_Address & 0xFF;
    memcpy(&pageTmp[PAGE_HDR_BYTES], pBuffer, NumByteToWrite_up_to_PageSize);//w25qxx.PageSize);

    spiRdy = 0;

w25_withDMA = 1;
    W25_SELECT();
    if (w25_withDMA) {
    	HAL_SPI_Transmit_DMA(portFLASH, pageTmp, lens);
    	while (!spiRdy) {
    		W25qxx_Delay(1);
    	}
w25_withDMA = 0;
    } else {
    	HAL_SPI_Transmit(portFLASH, pageTmp, lens, min_wait_ms);

    	W25_UNSELECT();

    	W25qxx_WaitForWriteEnd();

    	spiRdy = 1;

#ifdef W25QXX_DEBUG
    	StartTime = HAL_GetTick() - StartTime;
    	for (uint32_t i = 0; i < NumByteToWrite_up_to_PageSize ; i++) {
    		if ( (i % 16 == 0) && (i > 2) ) Report(0, "\r\n");
    		Report(0, "0x%02X,", pBuffer[i]);
    	}
    	Report(0, "\r\n");
    	Report(1, "%s done after %u ms\r\n", __func__, StartTime);
#endif
    }

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize)
{
    if ((NumByteToWrite_up_to_SectorSize > w25qxx.SectorSize) || !NumByteToWrite_up_to_SectorSize)
                NumByteToWrite_up_to_SectorSize = w25qxx.SectorSize;

#ifdef W25QXX_DEBUG
    Report(1, "%s WriteSector:0x%X(%u), Offset:%u ,Write %u Bytes, begin...\r\n",
                 __func__, Sector_Address, Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize);
    //W25qxx_Delay(100);
#endif

    if (OffsetInByte >= w25qxx.SectorSize) {
#ifdef W25QXX_DEBUG
    	Report(1, "---w25qxx WriteSector Faild!\r\n");
    	//W25qxx_Delay(100);
#endif
        return;
    }

    int32_t BytesToWrite;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToWrite_up_to_SectorSize) > w25qxx.SectorSize)
        BytesToWrite = w25qxx.SectorSize - OffsetInByte;
    else
        BytesToWrite = NumByteToWrite_up_to_SectorSize;
    StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;

    do
    {
        W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
        StartPage++;
        BytesToWrite -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToWrite > 0);

#ifdef W25QXX_DEBUG
    Report(1, "%s Done\r\n", __func__);
    //W25qxx_Delay(100);
#endif

}
//------------------------------------------------------------------------------------------
void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize)
{
    if ((NumByteToWrite_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToWrite_up_to_BlockSize)
            NumByteToWrite_up_to_BlockSize = w25qxx.BlockSize;

#ifdef W25QXX_DEBUG
    Report(1, "%s WriteBlock:0x%X(%u), Offset:%u ,Write %u Bytes, begin...\r\n",
                 __func__, Block_Address, Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize);
    //W25qxx_Delay(100);
#endif

    if (OffsetInByte >= w25qxx.BlockSize) {
#ifdef W25QXX_DEBUG
    	Report(1, "%s Faild!\r\n", __func__);
    	//W25qxx_Delay(100);
#endif
        return;
    }

    int32_t BytesToWrite;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToWrite_up_to_BlockSize) > w25qxx.BlockSize)
        BytesToWrite = w25qxx.BlockSize - OffsetInByte;
    else
        BytesToWrite = NumByteToWrite_up_to_BlockSize;
    StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do
    {
        W25qxx_WritePage(pBuffer, StartPage, LocalOffset, BytesToWrite);
        StartPage++;
        BytesToWrite -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToWrite > 0);

#ifdef W25QXX_DEBUG
    Report(1, "%s done\r\n", __func__);
    //W25qxx_Delay(100);
#endif

}
//------------------------------------------------------------------------------------------
void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

#ifdef W25QXX_DEBUG
    uint32_t StartTime = HAL_GetTick();
    Report(1, "%s at address %u begin...\r\n", __func__, Bytes_Address);
#endif

    W25_SELECT();
    W25qxx_Spi(DATA_READ);//FAST_READ);
    //if (w25qxx.ID >= W25Q256) W25qxx_Spi((Bytes_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Bytes_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Bytes_Address& 0xFF00) >> 8);
    W25qxx_Spi(Bytes_Address & 0xFF);
    //W25qxx_Spi(0);
    *pBuffer = W25qxx_Spi(W25QXX_DUMMY_BYTE);
    W25_UNSELECT();

#ifdef W25QXX_DEBUG
    Report(1, "%s 0x%02X done after %u ms\r\n", __func__, *pBuffer, HAL_GetTick() - StartTime);
#endif

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

#ifdef W25QXX_DEBUG
    uint32_t StartTime = HAL_GetTick();
    Report(1, "%s at Address:0x%X(%u), %u Bytes  begin...\r\n",
    			__func__, ReadAddr, ReadAddr, NumByteToRead);
#endif

    W25_SELECT();
    W25qxx_Spi(DATA_READ);//FAST_READ);
    //if (w25qxx.ID >= W25Q256) W25qxx_Spi((ReadAddr & 0xFF000000) >> 24);
    W25qxx_Spi((ReadAddr & 0xFF0000) >> 16);
    W25qxx_Spi((ReadAddr& 0xFF00) >> 8);
    W25qxx_Spi(ReadAddr & 0xFF);
    //W25qxx_Spi(0);
    HAL_SPI_Receive(portFLASH, pBuffer, NumByteToRead, max_wait_ms);
    W25_UNSELECT();

#ifdef W25QXX_DEBUG
    StartTime = HAL_GetTick() - StartTime;
    for (uint32_t i = 0; i < NumByteToRead ; i++) {
    	if ((i % 8 == 0) && (i > 2)) {
    		Report(0, "\r\n");
    		//W25qxx_Delay(10);
    	}
    	Report(0, "0x%02X,", pBuffer[i]);
    }
    Report(0, "\r\n");
    Report(1, "%s done after %u ms\r\n", __func__, StartTime);
    //W25qxx_Delay(100);
#else
    //W25qxx_Delay(1);
#endif

    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize)
{
    while (w25qxx.Lock) W25qxx_Delay(1);

    w25qxx.Lock = 1;

    if ((NumByteToRead_up_to_PageSize > w25qxx.PageSize) || !NumByteToRead_up_to_PageSize)
        NumByteToRead_up_to_PageSize = w25qxx.PageSize;
    if ((OffsetInByte + NumByteToRead_up_to_PageSize) > w25qxx.PageSize)
        NumByteToRead_up_to_PageSize = w25qxx.PageSize - OffsetInByte;

#ifdef W25QXX_DEBUG
    Report(1, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Page_Address, Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize);
    //W25qxx_Delay(100);
    uint32_t StartTime = HAL_GetTick();
#endif

    Page_Address = Page_Address * w25qxx.PageSize + OffsetInByte;
    /*
    W25_SELECT();
    W25qxx_Spi(DATA_READ);//FAST_READ);
    //if (w25qxx.ID >= W25Q256) W25qxx_Spi((Page_Address & 0xFF000000) >> 24);
    W25qxx_Spi((Page_Address & 0xFF0000) >> 16);
    W25qxx_Spi((Page_Address& 0xFF00) >> 8);
    W25qxx_Spi(Page_Address & 0xFF);
    //W25qxx_Spi(0);
    HAL_SPI_Receive(portFLASH, pBuffer, NumByteToRead_up_to_PageSize, min_wait_ms);
    W25_UNSELECT();
    */
    memset(pageTmp, 0, sizeof(pageTmp));
    uint16_t lens = NumByteToRead_up_to_PageSize + PAGE_HDR_BYTES;// + 1;
    int idx = 0;
    pageTmp[idx++] = DATA_READ;//FAST_READ;
    //if (w25qxx.ID >= W25Q256) pageTmp[idx++] = (Page_Address & 0xFF000000) >> 24;
    pageTmp[idx++] = (Page_Address & 0xFF0000) >> 16;
    pageTmp[idx++] = (Page_Address& 0xFF00) >> 8;
    pageTmp[idx++] = Page_Address & 0xFF;
    //pageTmp[idx++] = 0;
    spiRdy = 0;

w25_withDMA = 1;
    W25_SELECT();
    if (w25_withDMA) {
    	HAL_SPI_TransmitReceive_DMA(portFLASH, pageTmp, pageTmp, lens);
    	while (!spiRdy) {
    		W25qxx_Delay(1);
    	}
w25_withDMA = 0;
    } else {
    	if (HAL_SPI_TransmitReceive(portFLASH, pageTmp, pageTmp, lens, min_wait_ms) != HAL_OK) devError |= devSPI;
    	W25_UNSELECT();

    	spiRdy = 1;
    }
	memcpy(pBuffer, &pageTmp[PAGE_HDR_BYTES], NumByteToRead_up_to_PageSize);//w25qxx.PageSize);
#ifdef W25QXX_DEBUG
    	StartTime = HAL_GetTick() - StartTime;
    	for (uint32_t i = 0; i < NumByteToRead_up_to_PageSize ; i++) {
    		if ((i % 16 == 0) && (i > 2)) Report(0, "\r\n");
    		Report(0, "0x%02X,", pBuffer[i]);
    	}
    	Report(0, "\r\n");
    	Report(1, "%s done after %u ms\r\n", __func__, StartTime);
#endif


    w25qxx.Lock = 0;
}
//------------------------------------------------------------------------------------------
void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
    if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || !NumByteToRead_up_to_SectorSize)
                NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;

#ifdef W25QXX_DEBUG
    Report(1, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Sector_Address, Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
    //W25qxx_Delay(100);
#endif

    if (OffsetInByte >= w25qxx.SectorSize) {
#ifdef W25QXX_DEBUG
    	Report(1, "---w25qxx ReadSector Faild!\r\n");
    	//W25qxx_Delay(100);
#endif
        return;
    }

    int32_t BytesToRead;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
        BytesToRead = w25qxx.SectorSize - OffsetInByte;
    else
        BytesToRead = NumByteToRead_up_to_SectorSize;
    StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do {
        W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
        StartPage++;
        BytesToRead -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToRead > 0);

#ifdef W25QXX_DEBUG
    Report(1, "%s done\r\n", __func__);
    //W25qxx_Delay(100);
#endif

}
//------------------------------------------------------------------------------------------
void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize)
{
    if ((NumByteToRead_up_to_BlockSize > w25qxx.BlockSize) || !NumByteToRead_up_to_BlockSize)
        NumByteToRead_up_to_BlockSize = w25qxx.BlockSize;

#ifdef W25QXX_DEBUG
    Report(1, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...\r\n",
                 __func__, Block_Address, Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize);
    //W25qxx_Delay(100);
#endif

    if (OffsetInByte >= w25qxx.BlockSize) {
#ifdef W25QXX_DEBUG
    	Report(1, "%s Faild!\r\n", __func__);
    	//W25qxx_Delay(100);
#endif
        return;
    }

    int32_t BytesToRead;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToRead_up_to_BlockSize) > w25qxx.BlockSize)
        BytesToRead = w25qxx.BlockSize - OffsetInByte;
    else
        BytesToRead = NumByteToRead_up_to_BlockSize;
    StartPage = W25qxx_BlockToPage(Block_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do {
        W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
        StartPage++;
        BytesToRead -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToRead > 0);

#ifdef W25QXX_DEBUG
    Report(1, "%s done\r\n", __func__);
    //W25qxx_Delay(100);
#endif

}
//------------------------------------------------------------------------------------------
//
void W25qxx_RdSec(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize)
{
    if ((NumByteToRead_up_to_SectorSize > w25qxx.SectorSize) || !NumByteToRead_up_to_SectorSize)
                NumByteToRead_up_to_SectorSize = w25qxx.SectorSize;

/*#ifdef W25QXX_DEBUG
    Report(NULL, true, "%s:0x%X(%u), Offset:%u ,Read %u Bytes, begin...",
                 __func__, Sector_Address, Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize);
    //W25qxx_Delay(100);
#endif*/

    if (OffsetInByte >= w25qxx.SectorSize) {
/*#ifdef W25QXX_DEBUG
    	Report(NULL, true, "---w25qxx ReadSector Faild!\r\n");
    	//W25qxx_Delay(100);
#endif*/
        return;
    }

    int32_t BytesToRead;
    uint32_t LocalOffset, StartPage;
    if ((OffsetInByte + NumByteToRead_up_to_SectorSize) > w25qxx.SectorSize)
        BytesToRead = w25qxx.SectorSize - OffsetInByte;
    else
        BytesToRead = NumByteToRead_up_to_SectorSize;
    StartPage = W25qxx_SectorToPage(Sector_Address) + (OffsetInByte / w25qxx.PageSize);
    LocalOffset = OffsetInByte % w25qxx.PageSize;
    do {
        W25qxx_ReadPage(pBuffer, StartPage, LocalOffset, BytesToRead);
        StartPage++;
        BytesToRead -= w25qxx.PageSize - LocalOffset;
        pBuffer += w25qxx.PageSize;
        LocalOffset = 0;
    } while(BytesToRead > 0);

/*#ifdef W25QXX_DEBUG
    Report(NULL, false, " done\r\n");
    W25qxx_Delay(1);//100);
#endif*/

}
//------------------------------------------------------------------------------------------

#endif
