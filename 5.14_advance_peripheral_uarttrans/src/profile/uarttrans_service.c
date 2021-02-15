/*
 * Filename: serial_port_service.c
 *
 * Description: This is the simple_peripheral example modified to send
 * data over BLE at a high throughput.
 *
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"

#include "ti/drivers/uart/UARTCC26XX.h"
#include "board.h"
#include "peripheral_uarttrans.h"

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "uarttrans_service.h"
//#include "inc\sdi_task.h"
#include "task_epd.h"
/*********************************************************************
 * MACROS
 */
#define AUTO_NOTIFICATION FALSE
/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        5

gattAttribute_t SerialPortServiceAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED];
/*********************************************************************
 * TYPEDEFS
 */

typedef enum UART_STATUS_CHAR {
  FRAMING_ERR_BYTE,
  PARITY_ERR_BYTE,
  RF_OVERRUN_BYTE
} UART_STATUS_BYTE_DEF;

/*!
 *  @brief    UART config service settings
 *
 *  This enumeration defines the UART config bits in Byte 2 of config. char.
 */
typedef enum UART_CONFIG_CHAR {
    UART_CONFIG_START = 0,        /*!<Start bit level */
    UART_CONFIG_STOP = 1,         /*!< Stop bit level (must be different from start bit level)*/
    UART_CONFIG_STOP_BITS  = 2,   /*!< Number of stop bits */
    UART_CONFIG_PARITY = 3,       /*!< Parity enable  */
    UART_CONFIG_EVEN  = 4,        /*!< Parity level  */
    UART_CONFIG_FLOW = 5          /*!< Flow control enabled  */
} UART_CONFIG_BIT_DEF;
/*********************************************************************

 * GLOBAL VARIABLES
 */
// Serial Port Service Profile Service UUID: 0xC0E0
CONST uint8 SerialPortServUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SERIALPORTSERVICE_SERV_UUID)
};

// Characteristic Data UUID: 0xC0E1
CONST uint8 SerialPortServiceDataUUID[ATT_UUID_SIZE] =
{
  TI_BASE_UUID_128(SERIALPORTSERVICE_DATA_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static SerialPortServiceCBs_t *SerialPortService_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Serial Port Profile Service attribute
static CONST gattAttrType_t SerialPortService = { ATT_UUID_SIZE, SerialPortServUUID };

// Serial Port Profile Characteristic Data Properties
static uint8 SerialPortServiceDataProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_NOTIFY;

// Serial Port Profile Characteristic Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *SerialPortServiceDataConfig;

// Characteristic Data Value
uint8 SerialPortServiceData[SERIALPORTSERVICE_DATA_LEN] = {0,};

// Serial Port Profile Characteristic Data User Description
static uint8 SerialPortServiceDataUserDesp[21] = "Data Characteristic \0";

//Keep track of length
static uint8 charDataValueLen = SERIALPORTSERVICE_DATA_LEN;

/*********************************************************************
 * Profile Attributes - Table
 */

gattAttribute_t SerialPortServiceAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Serial Port Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&SerialPortService            /* pValue */
  },

    // Characteristic Data Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &SerialPortServiceDataProps
    },

      // Characteristic Data Value
      {
        { ATT_UUID_SIZE, SerialPortServiceDataUUID },
        GATT_PERMIT_WRITE,
        0,
        SerialPortServiceData
      },

       // Characteristic Data configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&SerialPortServiceDataConfig
      },

      // Characteristic Data User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        SerialPortServiceDataUserDesp
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t SerialPortService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method );
static bStatus_t SerialPortService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Serial Port Profile Service Callbacks
CONST gattServiceCBs_t SerialPortServiceCBs =
{
  SerialPortService_ReadAttrCB,  // Read callback function pointer
  SerialPortService_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SerialPortService_AddService
 *
 * @brief   Initializes the Serial Port Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SerialPortService_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  SerialPortServiceDataConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            linkDBNumConns );

  if ( SerialPortServiceDataConfig == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, SerialPortServiceDataConfig );

  if ( services & SERIALPORTSERVICE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( SerialPortServiceAttrTbl,
                                          GATT_NUM_ATTRS( SerialPortServiceAttrTbl ),
                                          16,
                                          &SerialPortServiceCBs );
  }
  else
  {
    status = SUCCESS;
  }

  return ( status );
}

/*********************************************************************
 * @fn      SerialPortService_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SerialPortService_RegisterAppCBs( SerialPortServiceCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    SerialPortService_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SerialPortService_SetParameter
 *
 * @brief   Set a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SERIALPORTSERVICE_CHAR_DATA:

      if( len <= SERIALPORTSERVICE_DATA_LEN )
      {
        //memset(SerialPortServiceData, 0, SERIALPORTSERVICE_DATA_LEN);
        VOID memcpy( SerialPortServiceData, value, len );
        charDataValueLen = len;

        // See if Notification has been enabled
        ret = GATTServApp_ProcessCharCfg( SerialPortServiceDataConfig, SerialPortServiceData, FALSE,
                                    SerialPortServiceAttrTbl, GATT_NUM_ATTRS( SerialPortServiceAttrTbl ),
                                    INVALID_TASK_ID, SerialPortService_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      SerialPortService_GetParameter
 *
 * @brief   Get a Serial Port Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SerialPortService_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SERIALPORTSERVICE_CHAR_DATA:
      VOID memcpy( value, SerialPortServiceData, charDataValueLen );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          SerialPortService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t SerialPortService_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint16 *pLen, uint16 offset,
                                           uint16 maxLen, uint8 method )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    // Get 16-bit UUID from 128-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);

    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics SERIALPORTSERVICE_STATUS_UUID and SERIALPORTSERVICE_CONFIG_UUID have read permissions
      // characteristic SERIALPORTSERVICE_DATA_UUID does not have read permissions, but because it
      //   can be sent as a notification, it is included here

      case SERIALPORTSERVICE_DATA_UUID:
        *pLen = charDataValueLen;
        VOID memcpy( pValue, pAttr->pValue, charDataValueLen );
        break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if( pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    //neither 16-bit UUID nor 128bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      SerialPortService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t SerialPortService_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint16 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  if ( pAttr->type.len == ATT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]);
    switch ( uuid )
    {
      case SERIALPORTSERVICE_DATA_UUID:
        if ( offset == 0 )
        {
          if ( len > SERIALPORTSERVICE_DATA_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;

          //Copy/Store data to the GATT table entry
          //memset(pCurValue, 0, SERIALPORTSERVICE_DATA_LEN);
          memcpy(pCurValue, pValue, len);
          
          //Send Data to UART
         // SDITask_sendToUART(pCurValue, len);
          EPDTask_parseCommand(pCurValue, len);
          //uncomment to notify application
          //notifyApp = SERIALPORTSERVICE_CHAR_DATA;
        }

        break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else if (pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch ( uuid )
    {
    case GATT_CLIENT_CHAR_CFG_UUID:
       status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
       break;
    default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
      //neither 16bit UUID nor 128bit UUID
      status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && SerialPortService_AppCBs && SerialPortService_AppCBs->pfnSerialPortServiceChange )
  {
    SerialPortService_AppCBs->pfnSerialPortServiceChange( notifyApp );
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
