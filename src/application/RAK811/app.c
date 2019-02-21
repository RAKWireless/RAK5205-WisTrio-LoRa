/*
 / _____)             _              | |
 ( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
 (______/|_____)_|_|_| \__)_____)\____)_| |_|
 (C)2013 Semtech

 Description: LoRaMac classA device implementation

 License: Revised BSD License, see LICENSE.TXT file include in the project

 Maintainer: Miguel Luis and Gregory Cristian
 */

/*! \file classA/SensorNode/main.c */

#include <string.h>
#include <math.h>
#include "board.h"

#include "app.h"
#include "LoRaMac.h"
#include "Region.h"
#include "rw_sys.h"

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_5

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1


#include "LoRaMacTest.h"


#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        false

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }

#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }
/*
#define LC11               { 865300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC12               { 865500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC13               { 865700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC14               { 865900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC15               { 866100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC16               { 866300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC17               { 866500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC18               { 866700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
*/
#endif

#ifdef TRACKERBOARD

char* rw_Region2Str(LoRaMacRegion_t region);
LoRaMacRegion_t rw_Str2Region(char* region);
LoRaMacRegion_t region;
bool OTAA_JOIN_STATE = false;
/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize;


/*!
 * User application data
 */
extern uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
 bool AppLedStateOn = false;


 bool LoraSendWait = false;

 TimerEvent_t LoraSendWaitTimer;

 void LoraSendWaitTimerEvent(void){
	TimerStop(&LoraSendWaitTimer);
	LoraSendWait = false;
}
/*!
 * Timer to handle the state of LED1
 */
 TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
 TimerEvent_t Led2Timer;


 TimerEvent_t HIWDG_Timer;

 void HIWDG_Timer_Event(void){
	TimerStop(&HIWDG_Timer);
#ifdef TRACKERBOARD
	GpioWrite(&Led1, 0);
	TimerStart(&Led1Timer);
#endif
	TimerStart(&HIWDG_Timer);
}

T_eDeviceState DeviceState;

 bool at_wait = true;

 TimerEvent_t atWaitTimer;
 void atWaitTimerEvent(void) {
	at_wait = false;
}

/*!
 * Indicates if a new packet can be sent
 */
 bool NextTx = true;



extern bool SystemWakeupTimeCalibrated;

static uint32_t APP_PORT_FLAG = 0;

#define APP_PORT_GPS_FLAG	( 0x01 << 0 )
#define APP_PORT_TEMP_FLAG 	( 0x01 << 1 )
#define APP_PORT_ACC_FLAG 	( 0x01 << 2 )
#define APP_PORT_GAS_FLAG   ( 0x01 << 3 )

#define APP_PORT_GPS 2
#define APP_PORT_TEMP 3
#define APP_PORT_ACC 4
#define APP_PORT_GAS 5

static uint8_t PrepareTxFrame(uint8_t port, uint8_t * app_data, uint8_t * size ) {
	double latitude, longitude = 0;
	int16_t altitudeGps = 0xFFFF;
	//	int8_t tempr = 25;
	uint8_t ret;
	uint16_t bat;
	uint8_t f_ret;
	int16_t temp = 0;
	uint32_t press = 0;
	uint32_t humi = 0;
	uint32_t resis = 0;
	int index;
	  
	
	switch (port) 
	{
  	//https://mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload
//cayenne LPP GPS
		case APP_PORT_GPS: 
		{
			ret = GpsGetLatestGpsPositionDouble(&latitude, &longitude);
			
			altitudeGps = GpsGetLatestGpsAltitude(); // in m
				
			gps84_To_Gcj02(latitude, longitude, &latitude, &longitude);
			if( ret == SUCCESS )
			{
				app_data[0] = 0x01;
				app_data[1] = 0x88;
				app_data[2] = ((int32_t) (latitude * 10000) >> 16) & 0xFF;
				app_data[3] = ((int32_t) (latitude * 10000) >> 8) & 0xFF;
				app_data[4] = ((int32_t) (latitude * 10000)) & 0xFF;
				app_data[5] = ((int32_t) (longitude * 10000) >> 16) & 0xFF;
				app_data[6] = ((int32_t) (longitude * 10000) >> 8) & 0xFF;
				app_data[7] = ((int32_t) (longitude * 10000)) & 0xFF;
				app_data[8] = ((altitudeGps * 100) >> 16) & 0xFF;
				app_data[9] = ((altitudeGps * 100) >> 8) & 0xFF;
				app_data[10] = (altitudeGps * 100) & 0xFF;
				
				float speed;
				sscanf(NmeaGpsData.NmeaSpeed,"%f", &speed );
				speed*=1.852;  //unit:km/h
				app_data[11] = ((uint16_t)(speed*100)>>8)& 0xFF;; //high byte speed   
				app_data[12] = ((uint16_t)(speed*100))& 0xFF;; //low byte speed    
				*size = 13;	
				e_printf("latitude: %f, longitude: %f , altitudeGps: %d, speed: %f\r\n", latitude, longitude, altitudeGps,speed);
				f_ret = SUCCESS;
			}
			else 
			{
				*size = 0;
				f_ret = FAIL;
			}
			break;
		}
//cayenne LPP Temp
		case APP_PORT_TEMP: 
		{
			app_data[0] = 0x07;
			app_data[1] = 0x02; //Analog Output
			bat = BoardBatteryMeasureVolage();
			app_data[2] = ((bat / 10) >> 8) & 0xFF;
			app_data[3] = (bat / 10) & 0xFF;
			*size = 4;
			e_printf("Tx_Bat: %dmv\r\n",bat);
			f_ret = SUCCESS;
			break;
		}
//cayenne LPP Acceleration
		case APP_PORT_ACC: 
		{
			app_data[0] = 0x03;
			app_data[1] = 0x71;
			app_data[2] = ( acc_data.acc_x >> 8 ) && 0xFF;
			app_data[3] = ( acc_data.acc_x ) && 0xFF;
			app_data[4] = ( acc_data.acc_y >> 8 ) && 0xFF;
			app_data[5] = ( acc_data.acc_y ) && 0xFF;
			app_data[6] = ( acc_data.acc_z >> 8 ) && 0xFF;
			app_data[7] = ( acc_data.acc_z ) && 0xFF;
			*size = 8;
			e_printf("Tx_ACC X:%dmg Y:%dmg Z:%dmg\r\n",
				acc_data.acc_x, acc_data.acc_y,acc_data.acc_z);
			f_ret = SUCCESS;
			break;
		}
  		
		case APP_PORT_GAS : 
		{
			if(SUCCESS == BME680_read(&temp, &press, &humi, &resis))
			{
				app_data[0] = 0x02;
				app_data[1] = 0x67;
				app_data[2] = (( temp / 10 ) >> 8 ) & 0xFF;
				app_data[3] = (temp / 10 ) & 0xFF;

				app_data[4] = 0x05;
				app_data[5] = 0x68;
				app_data[6] = ( humi / 500 ) & 0xFF;

				app_data[7] = 0x06;
				app_data[8] = 0x73;
				app_data[9] = (((int32_t)( press / 10)) >> 8) & 0xFF;
				app_data[10] = ((int32_t)(press / 10 )) & 0xFF;

				*size = 11;
				f_ret = SUCCESS;
			}
			else
			{
				f_ret = FAIL;
			}
			break;
		}
		case 224:
		{
			if (ComplianceTest.LinkCheck == true) 
			{
				ComplianceTest.LinkCheck = false;
				*size = 3;
				app_data[0] = 5;
				app_data[1] = ComplianceTest.DemodMargin;
				app_data[2] = ComplianceTest.NbGateways;
				ComplianceTest.State = 1;
				} 
			else 
			{
				switch (ComplianceTest.State) 
				{
					case 4:
						ComplianceTest.State = 1;
						break;
					case 1:
						*size = 2;
						app_data[0] = ComplianceTest.DownLinkCounter >> 8;
						app_data[1] = ComplianceTest.DownLinkCounter;
						break;
				}
			}
			f_ret = SUCCESS;
			break;
		}
		default:
			f_ret = FAIL;
			break;
  	}
  	
	return f_ret;
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame(uint8_t app_port) {
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;

	AppPort = app_port;
	
	if (LoRaMacQueryTxPossible(AppDataSize, &txInfo) != LORAMAC_STATUS_OK) {
		// Send empty frame in order to flush MAC commands
		e_printf("Tx impossible!!\r\n");
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
	} else {
    		if (IsTxConfirmed == false) {
          mcpsReq.Type = MCPS_UNCONFIRMED;
          mcpsReq.Req.Unconfirmed.fPort = AppPort;
          mcpsReq.Req.Unconfirmed.fBuffer = AppData;
          mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
          mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    		} else {
    			mcpsReq.Type = MCPS_CONFIRMED;
          mcpsReq.Req.Confirmed.fPort = AppPort;
          mcpsReq.Req.Confirmed.fBuffer = AppData;
          mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
          mcpsReq.Req.Confirmed.NbTrials = 8;
          mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    		}
	}
	
	GpioWrite(&Led2, 0);
	TimerStart(&Led2Timer);
	
	LoraSendWait = true;
	TimerSetValue(&LoraSendWaitTimer, 5000 );
	TimerStart(&LoraSendWaitTimer);
	
	if (LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK) {
		return false;
	}
	return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent(void) 
{
	MibRequestConfirm_t mibReq;
	LoRaMacStatus_t status;

	TimerStop(&TxNextPacketTimer);

	mibReq.Type = MIB_NETWORK_JOINED;
	status = LoRaMacMibGetRequestConfirm(&mibReq);

	if (status == LORAMAC_STATUS_OK) 
	{
		if (mibReq.Param.IsNetworkJoined == true)
		{
			if( APP_PORT_FLAG == 0 )
			{
				APP_PORT_FLAG |= APP_PORT_GPS_FLAG;
				APP_PORT_FLAG |= APP_PORT_ACC_FLAG;			
				APP_PORT_FLAG |= APP_PORT_TEMP_FLAG;		
				APP_PORT_FLAG |= APP_PORT_GAS_FLAG;
				DeviceState = DEVICE_STATE_SEND;
			}
			else if((APP_PORT_FLAG & APP_PORT_GPS_FLAG))DeviceState = DEVICE_STATE_SEND;
			else DeviceState = DEVICE_STATE_CYCLE;
			
			NextTx = true;
		} 
		else 
		{
			DeviceState = DEVICE_STATE_JOIN;
		}
	}
}


/*!
 * \brief Function executed on Led 1 Timeout event
 */
 void OnLed1TimerEvent(void) {
	TimerStop(&Led1Timer);
		GpioWrite(&Led1, 1);
	 
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
void OnLed2TimerEvent(void) {
	TimerStop(&Led2Timer);
	GpioWrite(&Led2, 1);

}


/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm_callback(McpsConfirm_t *mcpsConfirm)
{
	if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) 
	{
		switch (mcpsConfirm->McpsRequest) 
		{
			case MCPS_UNCONFIRMED: 
			{
				// Check Datarate
				// Check TxPower			
				break;
			}
			case MCPS_CONFIRMED: 
			{
				// Check Datarate
				// Check TxPower
				// Check AckReceived
				// Check NbTrials
				break;
			}
			case MCPS_PROPRIETARY: 
			{
				break;
			}
			default:
				break;
		}
	}

	if( APP_PORT_FLAG & APP_PORT_GPS_FLAG )APP_PORT_FLAG &= ~APP_PORT_GPS_FLAG;
	else if( APP_PORT_FLAG & APP_PORT_TEMP_FLAG )APP_PORT_FLAG &= ~APP_PORT_TEMP_FLAG;
	else if( APP_PORT_FLAG & APP_PORT_ACC_FLAG )APP_PORT_FLAG &= ~APP_PORT_ACC_FLAG;
	else if( APP_PORT_FLAG & APP_PORT_GAS_FLAG )APP_PORT_FLAG &= ~APP_PORT_GAS_FLAG;		

	if( APP_PORT_FLAG != 0 )DeviceState = DEVICE_STATE_SEND;
	else 
	{
		if( APP_PORT_FLAG == 0 )
		{
			TxDutyCycleTime = g_lora_config.app_interval * 1000;
		}		
		DeviceState = DEVICE_STATE_CYCLE;	
	}
	NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication_callback(McpsIndication_t *mcpsIndication) {

	if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		return;
	}
	
	switch (mcpsIndication->McpsIndication) {
	case MCPS_UNCONFIRMED: {
		break;
	}
	case MCPS_CONFIRMED: {
		break;
	}
	case MCPS_PROPRIETARY: {
		break;
	}
	case MCPS_MULTICAST: {
		break;
	}
	default:
		break;
	}

	// Check Multicast
	// Check Port
	// Check Datarate
	// Check FramePending
	// Check Buffer
	// Check BufferSize
	// Check Rssi
	// Check Snr
	// Check RxSlot

	if (ComplianceTest.Running == true) {
		ComplianceTest.DownLinkCounter++;
	}

	if (mcpsIndication->RxData == true) {
		dump_hex2str(mcpsIndication->Buffer, mcpsIndication->BufferSize);
		switch (mcpsIndication->Port) {
		case 1: // The application LED can be controlled on port 1 or 2
		case 2:
			if (mcpsIndication->BufferSize == 1) {
				AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
				GpioWrite(&Led2, ((AppLedStateOn & 0x01) != 0) ? 0 : 1);
			}
			break;
		case 224:
			if (ComplianceTest.Running == false) {
				// Check compliance test enable command (i)
				if ((mcpsIndication->BufferSize == 4)
						&& (mcpsIndication->Buffer[0] == 0x01)
						&& (mcpsIndication->Buffer[1] == 0x01)
						&& (mcpsIndication->Buffer[2] == 0x01)
						&& (mcpsIndication->Buffer[3] == 0x01)) {
					IsTxConfirmed = false;
					AppPort = 224;
					AppDataSize = 2;
					ComplianceTest.DownLinkCounter = 0;
					ComplianceTest.LinkCheck = false;
					ComplianceTest.DemodMargin = 0;
					ComplianceTest.NbGateways = 0;
					ComplianceTest.Running = true;
					ComplianceTest.State = 1;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = true;
					LoRaMacMibSetRequestConfirm(&mibReq);

					if( LORAMAC_REGION_EU868 == region )
						LoRaMacTestSetDutyCycleOn( false );
						
				}
			} else {
				ComplianceTest.State = mcpsIndication->Buffer[0];
				switch (ComplianceTest.State) {
				case 0: // Check compliance test disable command (ii)
					IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
					AppPort = LORAWAN_APP_PORT;
					if( LORAMAC_REGION_CN470 == region ||
						LORAMAC_REGION_CN470 == region ||
						LORAMAC_REGION_CN779 == region ||
						LORAMAC_REGION_EU433 == region ||
						LORAMAC_REGION_EU868 == region ||
						LORAMAC_REGION_IN865 == region ||
						LORAMAC_REGION_KR920 == region ){
						AppDataSize = 16;
					}else{
						AppDataSize = 11;
					}
					
					ComplianceTest.DownLinkCounter = 0;
					ComplianceTest.Running = false;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
					LoRaMacMibSetRequestConfirm(&mibReq);
					if( LORAMAC_REGION_EU868 == region )
						LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
					break;
				case 1: // (iii, iv)
					AppDataSize = 2;
					break;
				case 2: // Enable confirmed messages (v)
					IsTxConfirmed = true;
					ComplianceTest.State = 1;
					break;
				case 3: // Disable confirmed messages (vi)
					IsTxConfirmed = false;
					ComplianceTest.State = 1;
					break;
				case 4: // (vii)
					AppDataSize = mcpsIndication->BufferSize;

					AppData[0] = 4;
					for (uint8_t i = 1;
							i < MIN(AppDataSize, LORAWAN_APP_DATA_MAX_SIZE);
							i++) {
						AppData[i] = mcpsIndication->Buffer[i] + 1;
					}
					break;
				case 5: // (viii)
				{
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_LINK_CHECK;
					LoRaMacMlmeRequest(&mlmeReq);
					LoraSendWait = true;
					TimerStart(&LoraSendWaitTimer);
				}
					break;
				case 6: // (ix)
				{
					MlmeReq_t mlmeReq;

					// Disable TestMode and revert back to normal operation
					IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
					AppPort = LORAWAN_APP_PORT;
					if( LORAMAC_REGION_CN470 == region ||
						LORAMAC_REGION_CN470 == region ||
						LORAMAC_REGION_CN779 == region ||
						LORAMAC_REGION_EU433 == region ||
						LORAMAC_REGION_EU868 == region ||
						LORAMAC_REGION_IN865 == region ||
						LORAMAC_REGION_KR920 == region ){
						AppDataSize = 16;
					}else{
						AppDataSize = 11;
					}
					
					ComplianceTest.DownLinkCounter = 0;
					ComplianceTest.Running = false;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
					LoRaMacMibSetRequestConfirm(&mibReq);
					if ( LORAMAC_REGION_EU868 == region )
					{
						LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
					}
					/*GpsStart();*/

					mlmeReq.Type = MLME_JOIN;

					mlmeReq.Req.Join.DevEui = g_lora_config.dev_eui;
					mlmeReq.Req.Join.AppEui = g_lora_config.app_eui;
					mlmeReq.Req.Join.AppKey = g_lora_config.app_key;
					mlmeReq.Req.Join.NbTrials = 3;

					LoRaMacMlmeRequest(&mlmeReq);
					LoraSendWait = true;
					TimerStart(&LoraSendWaitTimer);
					DeviceState = DEVICE_STATE_SLEEP;
				}
					break;
				case 7: // (x)
				{
					if (mcpsIndication->BufferSize == 3) {
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_TXCW;
						mlmeReq.Req.TxCw.Timeout = (uint16_t)(
								(mcpsIndication->Buffer[1] << 8)
										| mcpsIndication->Buffer[2]);
						LoRaMacMlmeRequest(&mlmeReq);
					} else if (mcpsIndication->BufferSize == 7) {
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_TXCW_1;
						mlmeReq.Req.TxCw.Timeout = (uint16_t)(
								(mcpsIndication->Buffer[1] << 8)
										| mcpsIndication->Buffer[2]);
						mlmeReq.Req.TxCw.Frequency =
								(uint32_t) ((mcpsIndication->Buffer[3] << 16)
										| (mcpsIndication->Buffer[4] << 8)
										| mcpsIndication->Buffer[5]) * 100;
						mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
						LoRaMacMlmeRequest(&mlmeReq);
						LoraSendWait = true;
						TimerStart(&LoraSendWaitTimer);
					}
					ComplianceTest.State = 1;
				}
					break;
				default:
					break;
				}
			}
			break;
		default:
			break;
		}
	}

	// Switch LED 2 ON for each received downlink
	GpioWrite(&Led2, 0);
	TimerStart(&Led2Timer);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm_callback(MlmeConfirm_t *mlmeConfirm) {
static uint8_t RetryCnt=0;
	switch (mlmeConfirm->MlmeRequest) {
	case MLME_JOIN: {
		if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
			// Status is OK, node has joined the network
			DeviceState = DEVICE_STATE_SEND;
			e_printf("OTAA Join Success! \r\n");
			RetryCnt=0;
			/* Switch LED 1 ON*/
			GpioWrite(&Led2, 0);
			TimerStart(&Led2Timer);
			
			APP_PORT_FLAG |= APP_PORT_GPS_FLAG;
			APP_PORT_FLAG |= APP_PORT_ACC_FLAG;
			APP_PORT_FLAG |= APP_PORT_TEMP_FLAG;
			APP_PORT_FLAG |= APP_PORT_GAS_FLAG;

			TxDutyCycleTime = 5000;
		} else {
			// Join was not successful. Try to join again
			e_printf("OTAA Join Failed! \r\n");
			RetryCnt++;
			if(RetryCnt <= 5)
			{
				DeviceState = DEVICE_STATE_JOIN;
				e_printf("Retry OTAA join.\r\n");
			}
		}
		break;
	}
	case MLME_LINK_CHECK: {
		if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK) {
			// Check DemodMargin
			// Check NbGateways
			if (ComplianceTest.Running == true) {
				ComplianceTest.LinkCheck = true;
				ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
				ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
			}
		}
		break;
	}
	default:
		break;
	}
	NextTx = true;
}


uint32_t gps_retry_cnt = 0;
uint8_t loop_in_task = false;
bool at_interval_flag=false;


static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static MibRequestConfirm_t mibReq;
static MlmeReq_t mlmeReq;
/**
 * Main application entry point.
 */
int LoRaWAN_loop(void) {
	
	uint8_t ret;
	uint32_t next_time;
	uint8_t app_port;
//e_printf("DeviceState=%d \r\n",DeviceState);    
	switch (DeviceState) 
	{
		case DEVICE_STATE_INIT: 
		{
			LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm_callback;
			LoRaMacPrimitives.MacMcpsIndication = McpsIndication_callback;
			LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm_callback;
			LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;

			region=rw_Str2Region(g_lora_config.region);

			LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, region );			


			TimerInit(&TxNextPacketTimer, OnTxNextPacketTimerEvent);

			TimerInit(&LoraSendWaitTimer, LoraSendWaitTimerEvent);
			TimerSetValue(&LoraSendWaitTimer, 3000);


			DeviceState  = DEVICE_STATE_JOIN;			
		}
		break;
		case DEVICE_STATE_JOIN:
		{
			if( g_lora_config.join_mode[0] == 0x0A && g_lora_config.join_mode[1] == 0x55)
			{
				e_printf("OTAA mode: \r\n");
				e_printf("DevEui: ");
				dump_hex2str(g_lora_config.dev_eui, 8);
				e_printf("AppEui: ");
				dump_hex2str(g_lora_config.app_eui, 8);
				e_printf("AppKey: ");
				dump_hex2str(g_lora_config.app_key, 16);

				mlmeReq.Type = MLME_JOIN;

				mlmeReq.Req.Join.DevEui = g_lora_config.dev_eui;
				mlmeReq.Req.Join.AppEui = g_lora_config.app_eui;
				mlmeReq.Req.Join.AppKey = g_lora_config.app_key;
				mlmeReq.Req.Join.NbTrials = 3;				
				if (NextTx == true) 
				{	
					e_printf("OTAA Join Start... \r\n");					
					LoRaMacMlmeRequest(&mlmeReq);
					LoraSendWait = true;
					TimerSetValue(&LoraSendWaitTimer, 10000);
					TimerStart(&LoraSendWaitTimer);					
				}
				
				DeviceState = DEVICE_STATE_SLEEP;
			}
			else if(g_lora_config.join_mode[0] == 0xAB && g_lora_config.join_mode[1] == 0xAA)
			{
				uint32_t *devaddr = (uint32_t *)&g_lora_config.dev_addr[0];
				e_printf("ABP: \r\n");
				e_printf("DevEui: ");
				dump_hex2str(g_lora_config.dev_eui, 8);
				e_printf("DevAddr: ");
				e_printf("%02X%02X%02X%02X\r\n",g_lora_config.dev_addr[3],g_lora_config.dev_addr[2],g_lora_config.dev_addr[1],g_lora_config.dev_addr[0]);
				
				e_printf("NwkSKey: ");
				dump_hex2str(g_lora_config.nwks_key , 16);
				e_printf("AppSKey: ");
				dump_hex2str(g_lora_config.apps_key , 16);

				mibReq.Type = MIB_NET_ID;
				mibReq.Param.NetID = 0;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_DEV_ADDR;
				mibReq.Param.DevAddr = *devaddr;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_NWK_SKEY;
				mibReq.Param.NwkSKey = g_lora_config.nwks_key;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_APP_SKEY;
				mibReq.Param.AppSKey = g_lora_config.apps_key;
				LoRaMacMibSetRequestConfirm( &mibReq );

				mibReq.Type = MIB_NETWORK_JOINED;
				mibReq.Param.IsNetworkJoined = true;
				LoRaMacMibSetRequestConfirm( &mibReq );

				// Switch LED 2 ON
				GpioWrite( &Led2, 0 );
				TimerStart( &Led2Timer );
				
				APP_PORT_FLAG |= APP_PORT_GPS_FLAG;
				APP_PORT_FLAG |= APP_PORT_ACC_FLAG;	
				APP_PORT_FLAG |= APP_PORT_TEMP_FLAG;
				APP_PORT_FLAG |= APP_PORT_GAS_FLAG;

				DeviceState = DEVICE_STATE_SEND;
			}			
		}
		break;
		case DEVICE_STATE_SEND: 
		{
			if (NextTx == true) 
			{
				if( AppPort == 224 )
				{
					ret = PrepareTxFrame(224, AppData, &AppDataSize);
					app_port = 224;
				}
				else
				{
					if( APP_PORT_FLAG & APP_PORT_GPS_FLAG )
					{			
						if( GpsIsEnable() == false )
						{
							GpsStart();
							next_time = 2000;
						}

						ret = PrepareTxFrame(APP_PORT_GPS, AppData, &AppDataSize);
						app_port = APP_PORT_GPS;
						if( ret == SUCCESS )
						{
//							APP_PORT_FLAG &= ~APP_PORT_GPS_FLAG;
							if( GetBoardPowerSource( ) == BATTERY_POWER)
							{
								GpsStop();
							}
							gps_retry_cnt = 0;
							next_time = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
						}
						else
						{
							gps_retry_cnt++;
							if( gps_retry_cnt >= g_lora_config.gps_stime)
							{
								/*ret = PrepareTxFrame(APP_PORT_TEMP, app_data, &app_data_size);*/
								gps_retry_cnt = 0;
								APP_PORT_FLAG &= ~APP_PORT_GPS_FLAG;
								if( GetBoardPowerSource( ) == BATTERY_POWER)
								{
									GpsStop();
								}
								e_printf("FAIL.The Satellite signal not found!\r\n");				
								next_time = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
								DelayMs(next_time);
								DeviceState = DEVICE_STATE_SEND;
								break;							
							}
							else
							{
								next_time = 1000;
							}
						}
					}
					else if( APP_PORT_FLAG & APP_PORT_TEMP_FLAG )
					{
						ret = PrepareTxFrame(APP_PORT_TEMP, AppData, &AppDataSize);
						if( ret == SUCCESS )
						{
							app_port = APP_PORT_TEMP;
//							APP_PORT_FLAG &= ~APP_PORT_TEMP_FLAG;
						}
						next_time = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
					}
					else if( APP_PORT_FLAG & APP_PORT_ACC_FLAG )
					{
						ret = PrepareTxFrame(APP_PORT_ACC, AppData, &AppDataSize);
						if( ret == SUCCESS )
						{
							app_port = APP_PORT_ACC;
//							APP_PORT_FLAG &= ~APP_PORT_ACC_FLAG;
						}
						next_time = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
					}
					else if( APP_PORT_FLAG & APP_PORT_GAS_FLAG )
					{								
						ret = PrepareTxFrame( APP_PORT_GAS, AppData, &AppDataSize);
						if( ret == SUCCESS )
						{
							app_port = APP_PORT_GAS;
//							APP_PORT_FLAG &= ~APP_PORT_GAS_FLAG;
						}
						next_time = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
					}    				
				}
				
				if( ret == SUCCESS )
				{
					ret = SendFrame(app_port);
					NextTx = ret;
				}				
			}

			if (ComplianceTest.Running == true) 
			{
				// Schedule next packet transmission
				TxDutyCycleTime = 5000; // 5000 ms
			} else {
				// Schedule next packet transmission
				TxDutyCycleTime = next_time;
			}
			DeviceState = DEVICE_STATE_CYCLE;			
		}
		break;
		case DEVICE_STATE_CYCLE: 
		{
			// Schedule next packet transmission
				TimerSetValue(&TxNextPacketTimer, TxDutyCycleTime);
				TimerStart(&TxNextPacketTimer);	
				DeviceState = DEVICE_STATE_SLEEP;							
		}
		break;
		case DEVICE_STATE_SLEEP: 
		{			
			// Wake up through events
			if( GpsIsEnable() == false && LoraSendWait == false ){				
				TimerLowPowerHandler( );
			}			
		}
		break;
		default:
		{
			DeviceState = DEVICE_STATE_INIT;
		}
		break;
	}
		
	if( Lis3dhGetIntState() == true)
	{
		DelayMs(100);
		Lis3dh_IntEventClear();
	}
#ifdef GPS_PPS
	if (GpsGetPpsDetectedState() == true) 
	{
		// Switch LED 2 ON
		GpioWrite(&Led2, 0);
		TimerStart(&Led2Timer);
	}
#endif
	
	return 0;
}


void read_falsh_data(uint32_t addr, void *buffer, uint16_t len)
{
	memcpy(buffer, (void *)addr, len);
}

void write_flash_data(uint32_t addr, void *buffer,uint16_t len)
{
	uint32_t *wr_data = buffer;
	int i = 0;
	uint32_t PAGEError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;

	HAL_FLASH_Unlock();

	for (i = 0; i < len/4; i++) {
	    if (addr % FLASH_PAGE_SIZE == 0) {
	        EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	        EraseInitStruct.PageAddress = addr;
	        EraseInitStruct.NbPages     = 1;
	        HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	    }

	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *wr_data);
        wr_data++;
        addr += 4;
	}
	//DelayMs(1000);
	HAL_FLASH_Lock();
}

int parse_args(char* str, char* argv[])
{
    int i = 0;
    char* ch = str;

    while(*ch != '\0') {
        i++;
        /*Check if length exceeds*/
        if (i > 10) {
            return 0;
        }

        argv[i-1] = ch;
        		
        while(*ch != '=' && *ch != '\0' && *ch != '\r') {
                ch++;
        }
        if (*ch == '\r')
            break;
				
        if (*ch != '\0') {
            *ch = '\0';
			//printf("parm: i=%d, %s \r\n", i-1, argv[i-1]);
            ch++;
            while(*ch == '=') {
                ch++;
            }
        }
    }
    return i;
}

int chack_data(char* str)
{
	int argc;
	char hex_num[3] = {0};
	char *argv[2]={NULL};
	argc = parse_args(str, argv);
	if (argc == 2){		
		char *buffer = argv[1];
//	read_partition(PARTITION_0, (char *)&g_lora_config, sizeof(g_lora_config));
		
		if (0 == strcmp(argv[0],"at+dev_eui")){
			if (strlen(buffer) == 16) {
				for (int i = 0; i < 8; i++)
				{
					memcpy(hex_num, &buffer[i*2], 2);
					g_lora_config.dev_eui[i] = strtoul(hex_num, NULL, 16);
				}				
			} else {
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+app_eui")) {
			if (strlen(buffer) == 16) {
				for (int i = 0; i < 8; i++)
				{
					memcpy(hex_num, &buffer[i*2], 2);
					g_lora_config.app_eui[i] = strtoul(hex_num, NULL, 16);
				}
			} else {
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+app_key")) {
			if (strlen(buffer) == 32) {
				for (int i = 0; i < 16; i++)
				{
					memcpy(hex_num, &buffer[i*2], 2);
					g_lora_config.app_key[i] = strtoul(hex_num, NULL, 16);
				}
			} else {
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+region")) {
			if ( 0 == strcmp(buffer,"EU868") || 0 == strcmp(buffer,"US915") || 0 == strcmp(buffer,"AU915") ||
				   0 == strcmp(buffer,"AS923") || 0 == strcmp(buffer,"IN865") || 0 == strcmp(buffer,"KR920") || 
				   0 == strcmp(buffer,"CN470") || 0 == strcmp(buffer,"EU433") || 0 == strcmp(buffer,"CN779") )
		  {
				memcpy(g_lora_config.region,buffer,sizeof(g_lora_config.region));				
			}else{
				return 1;
			}		
		} else if (0 == strcmp(argv[0],"at+join_mode")) {
			if (0 == strcmp(buffer,"otaa")){
				g_lora_config.join_mode[0] = 0x0A;  // OTAA
				g_lora_config.join_mode[1] = 0x55;
			} else if (0 == strcmp(buffer,"abp")){
				g_lora_config.join_mode[0] = 0xAB;  // ABP
				g_lora_config.join_mode[1] = 0xAA;
			}else{
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+dev_addr")) {
			if (strlen(buffer) == 8) {
				uint32_t dev_addr;
				dev_addr = strtoul(buffer, NULL, 16);
				memcpy(g_lora_config.dev_addr,&dev_addr,sizeof(g_lora_config.dev_addr));
			} else {
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+apps_key")) {
			if (strlen(buffer) == 32) {
				for (int i = 0; i < 16; i++)
				{
					memcpy(hex_num, &buffer[i*2], 2);
					g_lora_config.apps_key[i] = strtoul(hex_num, NULL, 16);
				}
			} else {
				return 1;
			}
		} else if (0 == strcmp(argv[0],"at+nwks_key")) {
			if (strlen(buffer) == 32) {
				for (int i = 0; i < 16; i++)
				{
					memcpy(hex_num, &buffer[i*2], 2);
					g_lora_config.nwks_key[i] = strtoul(hex_num, NULL, 16);
				}
			} else {
				return 1;
			}
		} 
		else if( 0 == strcmp(argv[0], "at+app_interval")){
			g_lora_config.app_interval = strtoul(buffer, NULL, 10);
			return 0;
		}
		else if( 0 == strcmp(argv[0], "at+gps_stime")){
			g_lora_config.gps_stime = strtoul(buffer, NULL, 10);
			return 0;
		}
		else if( 0 == strcmp(argv[0], "at+ps")){
            g_lora_config.power_save = ( buffer[0] & 0x0F ) ? 1 : 0;
            return 0;
		}
		else if( 0 == strcmp( argv[0], "at+msg_confirm")){
            g_lora_config.msg_confirm = ( buffer[0] & 0x0F ) ? 1 : 0;
            return 0;
		}
		else if( 0 == strcmp(argv[0],"at+run")){
			return -1;
		}else{
			return 1;
		}
		return 0;
	}
	return 1;
}


char g_buffer[64];
int data_i = 0;
int getchar_loop(void)
{
	uint8_t data = '\0';
	int ret = 0;
	data = e_getchar();
	
	if ( data != '\0' && data != 0xFF)
	{
		g_buffer[data_i] = data;
		
		if (data == '\r') {
		   g_buffer[data_i] = '\0';
		}
		data_i++;
	}
	if (data == '\n' || data_i == 63) {
		TimerStop(&atWaitTimer);
		TimerStart(&atWaitTimer);
//		e_printf("%s\r\n",g_buffer);
		data_i = 0;
		ret = chack_data(g_buffer);
		if ( ret == 0){
			write_partition(PARTITION_0,&g_lora_config,sizeof(g_lora_config));
			rw_restore_LoRaWAN_config(rw_Str2Region(g_lora_config.region),0);			
			e_printf("OK\r\n");
		} else if( ret == -1 ){
			return -1;
		}else{
			e_printf("ERROR %d\r\n",ret);
		}
   }

   return 0;
}

extern uint8_t g_power_source;

void load_default_config(lora_config_t* a)
{
    if((*( uint32_t *)&(a->dev_eui[0]) == 0 && *(uint32_t *)&(a->dev_eui[4]) == 0 ) ||
      ((*( uint32_t *)&(a->dev_eui[0]) == 0xFFFFFFFF && *(uint32_t *)&(a->dev_eui[4]) == 0xFFFFFFFF ))
    ){
        *( uint32_t *)&(a->dev_eui[0]) = 0xD896E0FF;
        *( uint32_t *)&(a->dev_eui[4]) = 0xFF0100ED;
    }
    if((*( uint32_t *)&(a->app_eui[0]) == 0 && *(uint32_t *)&(a->app_eui[4]) == 0 ) ||
      ((*( uint32_t *)&(a->app_eui[0]) == 0xFFFFFFFF && *(uint32_t *)&(a->app_eui[4]) == 0xFFFFFFFF ))
    ){
        *( uint32_t *)&(a->app_eui[0]) = 0x70B3D57E;
        *( uint32_t *)&(a->app_eui[4]) = 0xD0014CEE;
    }

    if( (*( uint32_t *)&(a->app_key[0] ) == 0 && 
         *( uint32_t *)&(a->app_key[4] ) == 0 && 
         *( uint32_t *)&(a->app_key[8] ) == 0 &&
         *( uint32_t *)&(a->app_key[12]) == 0 ) ||
         (*( uint32_t *)&(a->app_key[0]) == 0xFFFFFFFF && 
         *( uint32_t *)&(a->app_key[4] ) == 0xFFFFFFFF && 
         *( uint32_t *)&(a->app_key[8] ) == 0xFFFFFFFF &&
         *( uint32_t *)&(a->app_key[12]) == 0xFFFFFFFF ))
     {
          *( uint32_t *)&(a->app_key[0]) = 0x5F8A98D9;
          *( uint32_t *)&(a->app_key[4]) = 0xAB0FD802;
          *( uint32_t *)&(a->app_key[8]) = 0x53F4A58B;
          *( uint32_t *)&(a->app_key[12]) = 0x2BCDA2C4;
     }

         
    if( a->app_interval == 0 ){
        a->app_interval = 10;
    }
    
    if( a->gps_stime == 0 ){
        a->gps_stime = 60;
    }

    if( !(a->join_mode[0] == 0x0A && a->join_mode[1] == 0x55) && 
        !(a->join_mode[0] == 0xAB && a->join_mode[1] == 0xAA) ){
      a->join_mode[0] = 0x0A;
      a->join_mode[1] = 0x55;
    }
}

#endif