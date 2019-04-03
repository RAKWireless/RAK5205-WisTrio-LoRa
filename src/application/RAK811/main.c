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

/*
	*  MACRO DEFINED for different boards and on/off specific function 
*/
//#define  LORA_HF_BOARD
//#define  TRACKERBOARD

#include <time.h>
#include <string.h>
#include "board.h"

#include "app.h"
#include "rw_lora.h"
#include "rw_sys.h"

lora_config_t g_lora_config;

extern uint8_t IsTxConfirmed ;
extern bool at_interval_flag;
extern void lora_cli_loop(void);

extern char cli_buffer[];

int main( void )
{
    uart_config_t uart_config;
	
 		BoardInitMcu( );  
	
    if (read_partition(PARTITION_1, (char *)&uart_config, sizeof(uart_config)) < 0) {
        SET_UART_CONFIG_DEFAULT(uart_config);
    } 
    
    UartMcuInit(&Uart1, 1, UART_TX, UART_RX);
    UartMcuConfig(&Uart1, RX_TX, uart_config.baudrate, 
                                      uart_config.wordLength,
                                      uart_config.stopBits,
                                      uart_config.parity,
                                      uart_config.flowCtrl);
		

    e_printf("RAK5205_TrackerBoard software version:");
		
    rw_GetVersion(cli_buffer);
    e_printf("%s\r\n", cli_buffer);

#ifdef TRACKERBOARD
		BoardInitPeriph();

		TimerInit(&atWaitTimer, atWaitTimerEvent);
		TimerSetValue(&atWaitTimer, 1000 * 30);
		
		TimerInit(&Led1Timer, OnLed1TimerEvent);
		TimerSetValue(&Led1Timer, 100);

		TimerInit(&Led2Timer, OnLed2TimerEvent);
		TimerSetValue(&Led2Timer, 100);
		
		TimerInit(&HIWDG_Timer, HIWDG_Timer_Event);
		TimerSetValue(&HIWDG_Timer, 20*1000);
		

		TimerStart(&atWaitTimer);
		
		TimerStart(&Led1Timer);
		
		rw_ReadUsrConfig();

		e_printf("Please Configurate parameters...\r\n");
		while(1){
			if(at_wait == false )
				break;
			if( 0 != getchar_loop()){
				break;
			}
		}
		write_partition(PARTITION_0,&g_lora_config,sizeof(g_lora_config));
		e_printf("Configuration OK!\r\n");
		
		e_printf("app_interval = %u\r\n", g_lora_config.app_interval);
		e_printf("gps_stime = %u\r\n", g_lora_config.gps_stime);
		e_printf("msg_confirm = %u\r\n", g_lora_config.msg_confirm);
		e_printf("power_save = %u\r\n", g_lora_config.power_save);

		if( g_lora_config.power_save == 0){
			g_power_source = USB_POWER;
		}
		else{
			g_power_source = BATTERY_POWER;
		}

		IsTxConfirmed =  g_lora_config.msg_confirm ? true : false;
#endif	
	
#ifdef TRACKERBOARD
	DeviceState = DEVICE_STATE_INIT;		
	LoRaWAN_loop();	
#else 
	rw_InitLoRaWAN();
#endif	
	
	GPIOIRQ_Enable();	
	
	BoardHiwdogInit();	
	TimerStart(&HIWDG_Timer);
	
	e_printf("Board Initialization OK!\r\n\r\n");
		
	while(1) 
	{

		lora_cli_loop();				
		BoardHIWDGRefresh();
#ifdef TRACKERBOARD
		LoRaWAN_loop();	
#endif			
	}
}

