
#include "epd2in13.h"


#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

#include "board.h"

 // for debug
//#include "inc/sdi_task.h"
#include "util.h"
 
#include <ti/drivers/PIN.h>


#define BLUE_LED_PIN             IOID_0     // low active

#define REED_PIN    IOID_13     // reed switch, ground when there is magnet
#define TEST_PIN    IOID_15     // on back of pcb with 'test'

#define EPD_POWER_PIN             IOID_20       // low active

#define EPD_RST_PIN             IOID_10
#define EPD_DC_PIN              IOID_11
#define EPD_BUSY_PIN            IOID_9
#define EPD_CS_PIN              IOID_12



/*********************************************************************
 * LOCAL PARAMETER
 */   
static PIN_Handle GPIOHandle = NULL;
static PIN_State GPIOState;
static PIN_Config GPIOTable[] =
{
  EPD_BUSY_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
  
  EPD_POWER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  
  EPD_DC_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_RST_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_CS_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
 
 
  PIN_TERMINATE
};

static SPI_Handle      SPIHandle = NULL;
static SPI_Params      SPIparams;


const unsigned char EPD_2IN13_lut_full_update[]= {
    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7

    0x03,0x03,0x00,0x00,0x02,                       // TP0 A~D RP0
    0x09,0x09,0x00,0x00,0x02,                       // TP1 A~D RP1
    0x03,0x03,0x00,0x00,0x02,                       // TP2 A~D RP2
    0x00,0x00,0x00,0x00,0x00,                       // TP3 A~D RP3
    0x00,0x00,0x00,0x00,0x00,                       // TP4 A~D RP4
    0x00,0x00,0x00,0x00,0x00,                       // TP5 A~D RP5
    0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D RP6

    0x15,0x41,0xA8,0x32,0x30,0x0A,
};

// quick and dirty way
// Task_sleep defined in <ti/sysbios/knl/Task.h>
static void Util_delay_ms(uint16_t t)
{  
  Task_sleep( ((t) * 1000) / Clock_tickPeriod );
}

static void HwUARTPrintf(const char *str)
{
 // HWUART_Printf(str);
}

static void DEV_Digital_Write(uint32_t pin, uint8_t value)
{    
    PIN_setOutputValue(GPIOHandle, pin, value);    
}

static int DEV_Digital_Read(uint32_t pin)
{
    int ret;
  ret = PIN_getInputValue(pin);
  return ret;
}

static void DEV_Delay_ms(uint16_t t)
{
    Util_delay_ms(t);
}

static void DEV_SPI_WriteByte(uint8_t byte)
{
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    
    txbuf[0] = byte;
    
  SPI_Transaction spiTransaction;
  spiTransaction.arg = NULL;
  spiTransaction.count = 1;
  spiTransaction.txBuf = txbuf;
  spiTransaction.rxBuf = rxbuf;
    
    

  bool ok =  SPI_transfer(SPIHandle, &spiTransaction);
 
  if (!ok) {
    HwUARTPrintf("spi transf fail\r\n");
  }
  
  
}

// should be only called once!
void epd_hw_init()
{

    HwUARTPrintf("setup epd gpio\r\n");
   GPIOHandle = PIN_open(&GPIOState, GPIOTable);      

     HwUARTPrintf("setup epd spi\r\n");
    
       SPI_init();
  SPI_Params_init(&SPIparams);
  SPIparams.bitRate  = 1000000;                    //1MHz
  SPIparams.dataSize = 8; 
  SPIparams.frameFormat = SPI_POL0_PHA0;           //
  SPIparams.mode = SPI_MASTER;                     //SPI master
  SPIparams.transferCallbackFxn = NULL;
  SPIparams.transferMode = SPI_MODE_BLOCKING;      // blocking
  SPIparams.transferTimeout = SPI_WAIT_FOREVER;
  
  SPIHandle = SPI_open(CC2640R2_LAUNCHXL_SPI0, &SPIparams);
  if (NULL == SPIHandle) {
    HwUARTPrintf("spi open fail\r\n");
  }
  
}



/******************************************************************************
function :	Software reset
parameter:
******************************************************************************/
static void EPD_2IN13_Reset(void)
{
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(2);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void EPD_2IN13_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void EPD_2IN13_SendData(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	Wait until the busy_pin goes LOW
parameter:
******************************************************************************/
void EPD_2IN13_ReadBusy(void)
{
    HwUARTPrintf("e-Paper busy\r\n");
    while(DEV_Digital_Read(EPD_BUSY_PIN) == 1) {      //LOW: idle, HIGH: busy
        DEV_Delay_ms(100);
    }
    HwUARTPrintf("e-Paper busy release\r\n");
}


/******************************************************************************
function :	Initialize the e-Paper register
parameter:
******************************************************************************/
void EPD_2IN13_Init()
{
    // power on
  DEV_Digital_Write(EPD_POWER_PIN, 0);
  DEV_Delay_ms(100);
  
    EPD_2IN13_Reset();

  
        EPD_2IN13_ReadBusy();
        EPD_2IN13_SendCommand(0x12); // soft reset
        EPD_2IN13_ReadBusy();

        EPD_2IN13_SendCommand(0x74); //set analog block control
        EPD_2IN13_SendData(0x54);
        EPD_2IN13_SendCommand(0x7E); //set digital block control
        EPD_2IN13_SendData(0x3B);

        EPD_2IN13_SendCommand(0x01); //Driver output control
        EPD_2IN13_SendData(0xD3);
        EPD_2IN13_SendData(0x00);
        EPD_2IN13_SendData(0x00);

        EPD_2IN13_SendCommand(0x11); //data entry mode
        EPD_2IN13_SendData(0x01);

        EPD_2IN13_SendCommand(0x44); //set Ram-X address start/end position
        EPD_2IN13_SendData(0x00);
        EPD_2IN13_SendData(0x0C);    //0x0C-->(15+1)*8=128

        EPD_2IN13_SendCommand(0x45); //set Ram-Y address start/end position
        EPD_2IN13_SendData(0xD3);   //0xF9-->(249+1)=250
        EPD_2IN13_SendData(0x00);
        EPD_2IN13_SendData(0x00);
        EPD_2IN13_SendData(0x00);

        EPD_2IN13_SendCommand(0x3C); //BorderWavefrom
        EPD_2IN13_SendData(0x01);
        
        // load lut
        
        EPD_2IN13_SendCommand(0x18); // set built in temperature sensor
        EPD_2IN13_SendData(0x80); //
        
        EPD_2IN13_SendCommand(0x22); // 
        EPD_2IN13_SendData(0xB1); //
        
        EPD_2IN13_SendCommand(0x20); // load LUT from OTP
                
        EPD_2IN13_ReadBusy();
        DEV_Delay_ms(100);
   
        HwUARTPrintf("epd initialized\r\n");
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void EPD_2IN13_Clear(void)
{
  HwUARTPrintf("epd clear\r\n");
    uint16_t Width, Height;
    Width = (EPD_2IN13_WIDTH % 8 == 0)? (EPD_2IN13_WIDTH / 8 ): (EPD_2IN13_WIDTH / 8 + 1);
    Height = EPD_2IN13_HEIGHT;

    EPD_2IN13_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN13_SendData(0XFF);
        }
    }

    EPD_2IN13_UpdateDisplay();
}

/******************************************************************************
function :	Sends the image buffer in RAM to e-Paper and displays
parameter:
******************************************************************************/
void EPD_2IN13_Display(const uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD_2IN13_WIDTH % 8 == 0)? (EPD_2IN13_WIDTH / 8 ): (EPD_2IN13_WIDTH / 8 + 1);
    Height = EPD_2IN13_HEIGHT;

    EPD_2IN13_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN13_SendData(Image[i + j * Width]);
        }
    }
    EPD_2IN13_UpdateDisplay();
}

void EPD_2IN13_PrepareBlkRAM(void)
{
  EPD_2IN13_SendCommand(0x4E);  // set RAM x address counter
  EPD_2IN13_SendData(0x00);
  
  EPD_2IN13_SendCommand(0x4F);  // set RAM Y address counter
  EPD_2IN13_SendData(0xD3);
  EPD_2IN13_SendData(0x00);
  
    // black white image
    EPD_2IN13_SendCommand(0x24);
    
}

void EPD_2IN13_PrepareRedRAM(void)
{
   EPD_2IN13_SendCommand(0x4E);  // set RAM x address counter
  EPD_2IN13_SendData(0x00);
  
  EPD_2IN13_SendCommand(0x4F);  // set RAM Y address counter
  EPD_2IN13_SendData(0xD3);
  EPD_2IN13_SendData(0x00);
  
    // red white image
    EPD_2IN13_SendCommand(0x26);
}

void EPD_2IN13_WriteRAM(const uint8_t *buf, const int len)
{
    for (int i = 0; i < len; i++) {
        EPD_2IN13_SendData(buf[i]);
    }
}

void EPD_2IN13_UpdateDisplay(void)
{
     HwUARTPrintf("turn on display\r\n");
  
    EPD_2IN13_SendCommand(0x22);
    EPD_2IN13_SendData(0xC7);
    EPD_2IN13_SendCommand(0x20);
    EPD_2IN13_ReadBusy();
    DEV_Delay_ms(200);
}

/******************************************************************************
function :	Enter sleep mode
parameter:
******************************************************************************/
void EPD_2IN13_Sleep(void)
{
    EPD_2IN13_SendCommand(0x22); //POWER OFF
    EPD_2IN13_SendData(0xC3);
    EPD_2IN13_SendCommand(0x20);

    EPD_2IN13_SendCommand(0x10); //enter deep sleep
    EPD_2IN13_SendData(0x01);
    DEV_Delay_ms(100);
    
    // power off
  DEV_Digital_Write(EPD_POWER_PIN, 1);
  DEV_Delay_ms(100);
}
