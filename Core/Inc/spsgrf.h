/**
******************************************************************************
* @file    spsgrf.h
* @author  Matthew Mielke
* @version V1.0.0
* @date    07-Jul-2021
* @brief   An abstraction of the SPIRIT1 library for the SPSGRF module.
******************************************************************************
*/

#ifndef _SPSGRF_H_
#define _SPSGRF_H_

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Config.h"


/* Exported constants --------------------------------------------------------*/
#define XTAL_FREQUENCY              50000000

/*  Radio configuration parameters  */
#define XTAL_OFFSET_PPM             0
//#define INFINITE_TIMEOUT            0.0
#define BASE_FREQUENCY              915.0e6
#define CHANNEL_SPACE               100e3
#define CHANNEL_NUMBER              0
#define MODULATION_SELECT           FSK
#define DATARATE                    38400
#define FREQ_DEVIATION              20e3
#define BANDWIDTH                   100E3

#define POWER_INDEX                 7
#define POWER_DBM                   11.6

//#define RECEIVE_TIMEOUT             2000.0 // change the value for required timeout period
#define RSSI_THRESHOLD              -120  // Default RSSI at reception, more than noise floor
//#define CSMA_RSSI_THRESHOLD         -90   // Higher RSSI to Transmit. If it's lower, the Channel will be seen as busy.

///*  Packet configuration parameters  */
#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define SYNC_WORD                   0x88888888
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define LENGTH_WIDTH                7
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_ADDRESS                  S_ENABLE
#define EN_FEC                      S_DISABLE
#define EN_WHITENING                S_ENABLE

#define EN_FILT_MY_ADDRESS          S_ENABLE
#define EN_FILT_MULTICAST_ADDRESS   S_ENABLE
#define EN_FILT_BROADCAST_ADDRESS   S_ENABLE
#define MY_ADDRESS                  0x7F
#define MULTICAST_ADDRESS           0xEE
#define BROADCAST_ADDRESS           0xFF

#define MAX_BUFFER_LEN              96
#define MAX_PAYLOAD_LEN             126 // (2^7 - 1) - 1 - 0 = 126 (LENGTH_WID=7, 1 address byte, & 0 control bytes)


/* Exported functions --------------------------------------------------------*/
void SPSGRF_Init(void);
void SPSGRF_StartTx(uint8_t *txBuff, uint8_t txLen);
void SPSGRF_StartRx(void);
uint8_t SPSGRF_GetRxData(uint8_t *rxBuff);


#endif /* _SPSGRF_H_ */

/*** end of file ***/
