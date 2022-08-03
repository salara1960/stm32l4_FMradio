# stm32l4_FMradio

STM32L476RGT6 board with flash W25X16 + RDA5807 + display GMG12864 + audio amplifier XY-SP5W + KCX_BT_EMITTER

#########################################################
#
#                  Project FM_Radio+
# STM32L476RGT6 board with spi_flash W25X16(2Mb) + RDA5807 +
# LCD GMG12864-06D + audio amplifier XY-SP5W +
# TL1838 Infrared Receiver +
# KCX_BT_EMITTER bluetooth audio transmitter
#
#########################################################


## Состав рабочего оборудования:

```
* STM32l476 (dev. board) - плата микроконтроллера STM32L476RGT6
* W25X16 - flash memory 2Mb (SPI + DMA + GPIO)
* RDA5807 - чип радиоприемника FM диапазона 65-108МГц (I2C)
* GMG12864-06D - LCD дисплей 128x64 (интерфейы SPI + DMA + GPIO)
* JDY-25M - BLE device (UART + DMA + GPIO)
* XY-SP5W - аудио усилитель + динамик (8ом 3Вт)
* TL1838 Infrared Receiver
* KCX_BT_EMITTER bluetooth audio transmitter
```


# Средства разработки:

```
* STM32CubeIDE - среда разработки под микроконтроллеры семейства STM32
  (https://www.st.com/en/development-tools/stm32cubeide.html).
* ST-LINK V2 - usb отладчик для микроконтроллеров семейства STM8/STM32.
* Saleae USB Logic 8ch - логический анализатор сигналов, 8 каналов , макс. частота дискретизации 24МГц
  (https://www.saleae.com/ru/downloads/)
```


# Функционал:
* Устройство предназначено для прослушивания радиостанций FM диапазонов (65 - 108 МГц).
* ПО построено по модели BARE METAL (без использования ОС) с использованием буфера событий типа fifo.
  События обслуживаются в основном цикле программы. Формируются события в callBack-функциях
  по завершении прерываний от используемых модулей микроконтроллера (USART, TIMER....)
* ПО реализовано средствами разработки STM32CubeIDE.
* Устройство инициализирует некоторые интерфейсы микроконтроллера :
  - GPIO : подключены два сетодиода : PA1 - секундный тик, PC3 - индикатор ошибки на устройстве,
           а также несколько других пинов для обслуживания W25X16, GMG12864, TL1838;
           в том числе три пользовательские кнопки KEY0(PC1), KEY1(PC2), WAKEUP(PA0) на плате микроконтроллера.
  - I2C1 : режим мастера с частотой 100Кгц (шина ослуживает радио-чип RDA5807).
  - USART2 : параметры порта 230400 8N1 - порт для логов и передачи команд устройству.
  - USART3 : параметры порта 115200 8N1 - порт для обслуживания bluetooth модуля KCX_BT_EMITTER.
  - TIM4 : таймер-счетчик временных интервалов в 10 мс., реализован в callback-функции.
  - TIM6 : таймер с интервалом 50 микросекунд для приема данных от инфракрасного приемника TL1838.
  - SPI1 : обслуживает дисплей GMG12864-06D.
  - SPI2 : обслуживает чип flash-памяти W25X16.
* Прием данных по последовательному порту (USART2) выполняется в callback-функции обработчика прерывания.

На дисплее отображается примерно следующая информация :

```
 RDA5807 chipID:0x58
  FM Band:76-108 MHz
    Bass:0 Vol:8
 Rssi:69 Freq:95.1 S
     Вести_ФМ
 evt(0):cfgStations
```


При успешном запуске в порту USART2 появятся следующие сообщения :


```
03.08 20:28:55 | [que:0] Start application ver.1.8 03.08.22
03.08 20:28:55 | w25qxx Init Begin... Chip ID:0x3015
03.08 20:28:55 | Chip W25Q16
        Page Size:      256 bytes
        Page Count:     8192
        Sector Size:    4096 bytes
        Sector Count:   512
        Block Size:     65536 bytes
        Block Count:    32
        Capacity:       2048 KBytes
03.08 20:28:55 | Readed cfg_stations_data (900 bytes) from cfgSector #511
03.08 20:28:55 | ChipID:0x58 Chan:180 Freq:94.00 Комеди_Радио RSSI:26 Band:76-108 MHz Vol:8 BassEn:0
03.08 20:28:56 | [que:0] set new Freq to 95.1 Вести_ФМ (Chan:191)
03.08 20:28:58 | [BLE_rx] ALL Devices=0
03.08 20:29:10 | [BLE_rx] ALL Devices=0
03.08 20:29:22 | [BLE_rx] ALL Devices=0
```


* Через USART2 можно отправлять команды на устройство, например :

```
ver
03.08 20:30:12 | Ver.1.8 03.08.22


help
        help
        restart
        epoch:
        inputerr
        read
        erase
        next
        write
        sec
        ver
        clr
        scan
        freq:
        vol:
        mute
        bass:
        list
        band:
        cfg
        wakeup
        exitsleep
        sleep
        sleepcont
        rds
        qevt
        qack
        qcmd


epoch:1659563875
03.08 21:57:55 | [que:0] Set Unix TimeStamp to 1659563875


scan
03.08 21:58:44 | [que:0] set new Freq to 96.3 Русское_Радио (Chan:203)
scan:0
03.08 21:58:50 | [que:0] set new Freq to 95.1 Вести_ФМ (Chan:191)


list
03.08 21:59:35 | [getNextList] up=1 ik=5, fr=95.1 ret=95.5 band=2
03.08 21:59:35 | Band = newBand = 2 -> goto set newFreq to 95.5 (up = 1)
03.08 21:59:35 | [que:0] set new Freq to 95.5 Ретро_ФМ (Chan:195)
list:down
03.08 21:59:40 | [getNextList] up=0 ik=4, fr=95.5 ret=95.1 band=2
03.08 21:59:40 | Band = newBand = 2 -> goto set newFreq to 95.1 (up = 0)
03.08 21:59:40 | [que:0] set new Freq to 95.1 Вести_ФМ (Chan:191)


vol:10
03.08 22:00:20 | [que:0] set new Volume to 10


mute
03.08 22:00:47 | [que:0] set Mute to 1
mute
03.08 22:00:50 | [que:0] set Mute to 0

band:3
03.08 22:01:57 | [que:0] set new band=3 '65-76 MHz'
band:2
03.08 22:02:03 | [que:0] set new band=2 '76-108 MHz'


freq:102.5
03.08 22:02:44 | [que:0] set new Freq to 102.5 Маяк (Chan:265)
freq:95.1
03.08 22:02:58 | [que:0] set new Freq to 95.1 Вести_ФМ (Chan:191)


sleep
03.08 22:03:30 | Going into SLEEP MODE...
...
03.08 22:03:34 | Exit from SLEEP MODE


read:511
Read sector:511 offset:0 len:512
1FF000  03 00 00 89 42 D0 9C D0 B0 D1 8F D0 BA 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
1FF020  00 00 00 00 03 33 33 90 42 D0 A8 D0 B0 D0 BD D1 81 D0 BE D0 BD 00 00 00 00 00 00 00 00 00 00 00
1FF040  00 00 00 00 00 00 00 00 02 33 33 BB 42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F 37 00 00 00 00 00 00 00
1FF060  00 00 00 00 00 00 00 00 00 00 00 00 02 00 00 BC 42 D0 9A D0 BE D0 BC D0 B5 D0 B4 D0 B8 5F D0 A0
1FF080  D0 B0 D0 B4 D0 B8 D0 BE 00 00 00 00 00 00 00 00 02 33 33 BE 42 D0 92 D0 B5 D1 81 D1 82 D0 B8 5F
1FF0A0  D0 A4 D0 9C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 00 00 BF 42 D0 A0 D0 B5 D1 82 D1
1FF0C0  80 D0 BE 5F D0 A4 D0 9C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 9A 99 C0 42 D0 A0 D1
1FF0E0  83 D1 81 D1 81 D0 BA D0 BE D0 B5 5F D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 00 00 00 00 00 00 02 00 00 C2
1FF100  42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 92 D0 B5 D1 80 D0 B0 00 00 00 00 00 00 00 00 00 00 00 00
1FF120  02 CD CC C3 42 D0 A1 D0 B5 D1 80 D0 B5 D0 B1 D1 80 2E D0 94 D0 BE D0 B6 D0 B4 D1 8C 00 00 00 00
1FF140  00 00 00 00 02 00 00 C5 42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 AD D0 BD D0 B5 D1 80 D0 B3 D0 B8
1FF160  D1 8F 00 00 00 00 00 00 02 00 00 C7 42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 97 D0 B2 D0 B5 D0 B7
1FF180  D0 B4 D0 B0 00 00 00 00 00 00 00 00 02 33 33 C8 42 D0 90 D0 B2 D1 82 D0 BE 5F D0 A0 D0 B0 D0 B4
1FF1A0  D0 B8 D0 BE 00 00 00 00 00 00 00 00 00 00 00 00 02 33 33 C9 42 D0 A0 D1 83 D1 81 D1 81 D0 BA D0
1FF1C0  B8 D0 B9 5F D0 9A D1 80 D0 B0 D0 B9 00 00 00 00 00 00 00 00 02 CD CC C9 42 D0 9C D0 BE D0 BD D1
1FF1E0  82 D0 B5 2D D0 9A D0 B0 D1 80 D0 BB D0 BE 00 00 00 00 00 00 00 00 00 00 02 9A 99 CA 42 D0 9D D0
next
Read sector:511 offset:512 len:512
1FF200  B0 D1 88 D0 B5 5F D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 00 00 00 00 00 00 00 00 00 00 00 00 02 9A 99 CB
1FF220  42 D0 91 D0 B8 D0 B7 D0 BD D0 B5 D1 81 5F D0 A4 D0 9C 00 00 00 00 00 00 00 00 00 00 00 00 00 00
1FF240  02 00 00 CD 42 D0 9C D0 B0 D1 8F D0 BA 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
1FF260  00 00 00 00 02 CD CC CD 42 D0 9B D1 8E D0 B1 D0 B8 D0 BC D0 BE D0 B5 5F D0 A0 D0 B0 D0 B4 D0 B8
1FF280  D0 BE 00 00 00 00 00 00 02 CD CC CE 42 D0 A1 D1 82 D1 83 D0 B4 D0 B8 D1 8F 5F 32 31 00 00 00 00
1FF2A0  00 00 00 00 00 00 00 00 00 00 00 00 02 CD CC CF 42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 A0 D0 BE
1FF2C0  D1 81 D1 81 D0 B8 D0 B8 00 00 00 00 00 00 00 00 02 00 00 D1 42 D0 95 D0 B2 D1 80 D0 BE D0 BF D0
1FF2E0  B0 5F D0 9F D0 BB D1 8E D1 81 00 00 00 00 00 00 00 00 00 00 02 66 66 D2 42 D0 91 D0 B0 D0 BB D1
1FF300  82 D0 B8 D0 BA 5F D0 9F D0 BB D1 8E D1 81 00 00 00 00 00 00 00 00 00 00 02 CD CC D3 42 D0 94 D0
1FF320  BE D1 80 D0 BE D0 B6 D0 BD D0 BE D0 B5 5F D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 00 00 00 00 02 CD CC D4
1FF340  42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 9C D0 B0 D0 BA D1 81 D0 B8 D0 BC 00 00 00 00 00 00 00 00
1FF360  02 66 66 D6 42 D0 A0 D0 B0 D0 B4 D0 B8 D0 BE 5F D0 9A D0 9F 00 00 00 00 00 00 00 00 00 00 00 00
1FF380  00 00 00 00 FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
1FF3A0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
1FF3C0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF
1FF3E0  FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF


cfg
3:68.5:Маяк
3:72.1:Шансон
2:93.6:Радио_7
2:94.0:Комеди_Радио
2:95.1:Вести_ФМ
2:95.5:Ретро_ФМ
2:96.3:Русское_Радио
2:97.0:Радио_Вера
2:97.9:Серебр.Дождь
2:98.5:Радио_Энергия
2:99.5:Радио_Звезда
2:100.1:Авто_Радио
2:100.6:Русский_Край
2:100.9:Монте-Карло
2:101.3:Наше_Радио
2:101.8:Бизнес_ФМ
2:102.5:Маяк
2:102.9:Ўбимое_Радио
2:103.4:Студия_21
2:103.9:Радио_России
2:104.5:Европа_Плюс
2:105.2:Балтик_Плюс
2:105.9:Дорожное_Радио
2:106.4:Радио_Максим
2:107.2:Радио_КП
```


* Пример установки текущего времени с помощью ИК-пульта :

```
29.07 14:41:31 | [que:0] irKEY: irSP
29.07 14:41:33 | [que:0] irKEY: ir1
29.07 14:41:37 | [que:0] irKEY: ir6
29.07 14:41:40 | [que:0] irKEY: ir5
29.07 14:41:42 | [que:0] irKEY: ir9
29.07 14:41:45 | [que:0] irKEY: ir1
29.07 14:41:46 | [que:0] irKEY: ir1
29.07 14:41:52 | [que:0] irKEY: ir6
29.07 14:41:54 | [que:0] irKEY: ir2
29.07 14:41:57 | [que:0] irKEY: ir9
29.07 14:41:58 | [que:0] irKEY: ir9
29.07 14:42:00 | [que:0] irKEY: irSP
29.07 14:42:00 | [que:0] get event 'Epoch'
29.07 17:38:19 | [que:0] Set Unix TimeStamp to 1659116299
```


* Устройство формирует слово состояния и отслеживает следующие ошибки :

```
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
devFS = 0x200,
devBLE = 0x400,
devQUE = 0x800
```


* Проект в процессе доработки


