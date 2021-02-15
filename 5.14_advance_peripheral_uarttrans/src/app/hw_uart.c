#include "hw_uart.h"

#include "board.h"
#include <ti/drivers/uart/UARTCC26XX.h>

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static UART_Handle UARTHandle;
static UART_Params UARTparams;

static void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size)
{ 
}

static void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size)
{
  
}

void HWUART_Init()
{
  
  UART_init();                                      //初始化模块的串口功能
  UART_Params_init(&UARTparams);                    //初始化串口参数
  UARTparams.baudRate = 115200;                     //串口波特率115200
  UARTparams.dataLength = UART_LEN_8;               //串口数据位8
  UARTparams.stopBits = UART_STOP_ONE;              //串口停止位1
  UARTparams.readDataMode = UART_DATA_BINARY;       //串口接收数据不做处理
  UARTparams.writeDataMode = UART_DATA_TEXT;        //串口发送数据不做处理
  UARTparams.readMode = UART_MODE_CALLBACK;         //串口异步读
  UARTparams.writeMode = UART_MODE_BLOCKING;        //   
  UARTparams.readEcho = UART_ECHO_OFF;              //串口不回显
  UARTparams.readReturnMode = UART_RETURN_NEWLINE;  //当接收到换行符时，回调
  UARTparams.readCallback = Uart_ReadCallback;      //
  UARTparams.writeCallback = Uart_WriteCallback;    //
  
  UARTHandle = UART_open(Board_UART0, &UARTparams); //打开串口通道
 // UART_control(UARTHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE,  NULL);   //允许接收部分回调
  
}

void HWUART_Printf(const char* format, ...)
{
      va_list arg;
  va_start(arg,format);
  uint8_t buf[108];
  uint16_t len;
  len = vsprintf((char*)buf, format, arg);
  UART_write(UARTHandle, buf, len);
}