# RAK5205-WisTrio-LoRa
RAK5205-WisTrio-LoRa :GPS ,BME680,LIS3DH,LoRaWAN1.0.2,ARM Cortex-M3 STM32L1<br><br>

**RAK5205-WisTrio-LoRa firmware version 2.x.0.3**<br>
1.fix the "LIS3DH no ack" problem.. <br>
2.fix a bug :when set the parameter "app_interval" too big,MCU will restart.<br><br>
**RAK5205-WisTrio-LoRa firmware version 2.x.0.2**<br>
1.Fix "at+msg_confirm:x"was invalid bug. <br>
2.Fix "at+set_config=pwr_level:x" was invalid bug.<br><br>
**RAK5205-WisTrio-LoRa firmware version 2.x.0.0**<br>
1.This Firmware is based on LoRaWAN 1.0.2 protocol ,support Class A and Class C mode.User could switch the mode by such as 'at+set_config=class:2' command,0:class A,1:class B(unsupported),2:class C.<br>
>      It supports almost all frequency bands:(HF)->EU868, US915, AU915, KR920, AS923，IN865. 
>                                             (LF)->EU433，CN470.
                                                  
2.[The Firmware folder](https://github.com/RAKWireless/RAK5205-WisTrio-LoRa/tree/master/doc/Firmware) contains two firmware: "RAK5205_LF_trackerboard.bin"and"RAK5205_LF_trackerboard.bin". <br> 
>      "RAK5205_HF_trackerboard.bin" surpport region:EU868, US915, AU915, KR920, AS923，IN865.　　 　
>      "RAK5205_LF_trackerboard.bin" surpport region:EU433，CN470.

  Tips：  Region switch by such as"at+band=EU868"command.
  AT command refer to [RAK811 Lora AT Command V1.4.pdf](https://github.com/RAKWireless/RAK5205-WisTrio-LoRa/blob/master/doc/RAK811%C2%A0Lora%C2%A0AT%C2%A0Command%C2%A0V1.4.pdf).   <br>
3.Method of The Demo project generates different firmware refer to [ReleaseNotes.txt](https://github.com/RAKWireless/RAK5205-WisTrio-LoRa/blob/master/src/board/RAK811/ReleaseNotes.txt).<br>
4.the manual of RAK5205-WisTrio-LoRa refer to [RAK5205\_User\_Manual_V1.2.pdf](https://github.com/RAKWireless/RAK5205-WisTrio-LoRa/blob/master/doc/RAK5205_User_Manual_V1.2.pdf)<br>

**Overview**<br>
The main features are listed below： <br>
 Compatible with 96Boards IoT Edition Specification. <br>
 With SX1276 LoRa long range and L76-L GPS modems which allow to enable the
GPS low power mode. <br>
 Integrated the ultra-low power microcontroller ARM Cortex-M3 STM32L1. <br>
 Built-in environmental sensor BME680 (gas, pressure, humidity, temperature) and
3-axis MEMS sensor LIS3DH (accelerometer). <br>
 SMA/IPEX antenna optional for LoRa and GPS. <br>
 Supports latest LoRaWAN 1.0.2 protocol, activation by OTAA/ABP. <br>
 Supports programmable bit rate up to 300kbps. <br>
 Supports rechargeable battery through micro USB or 5V solar charging ports. <br>
 Supports sleep mode, the power consumption down to 16μA. <br>
 Supports global license-free ISM band (EU433, CN470, EU868, US915, AS923,
AU915, KR920 and IN865). <br>
 Supports I2C, GPIOs, UART and ADC interfaces
