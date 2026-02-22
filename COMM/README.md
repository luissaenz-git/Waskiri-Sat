STM32F401VE – ADDNICS COM Board (ADD1397B) Simulation

This firmware turns an STM32F401VE into a COM PIC–like controller to simulate the digital behavior of the ADDNICS COM subsystem used in BIRDS/CubeSat-style architectures.
It is designed for Proteus to validate logic, modes, GPIO control, UART interaction, and housekeeping (RSSI/TEMP)

Simulation includes:
	
COM PIC mode logic (RX / GMSK TX / CW beacon), digital control lines to the transceiver (ON/OFF, CW enable, CW key), configuration commands (RON/ROF, MON/MOF, PNF/PNN), a simplified “uplink frame” trigger via USART1 and housekeeping sampling (RSSI and TEMP) via ADC



UART interfaces
Interface	STM32 Peripheral	Purpose	Typical STM32F401 Pins*	Connects to
TRX UART	USART1	“Ground Station” terminal (uplink triggers & downlink text output)	PA9 (TX), PA10 (RX)	Virtual Terminal #1
CONFIG UART	USART2	Local config console (HELP, RON, DL, etc.)	PA2 (TX), PA3 (RX)	Virtual Terminal #2



CONFIG UART (USART2) commands

Type commands in Virtual Terminal #2 according to ICD Transceiver commands.

Command	Meaning	What you should observe
HELP	Print help menu	Help text on CONFIG terminal
RON	RF output ON (enable TX)	PB8 goes HIGH; status reports RON
ROF	RF output OFF (block TX)	PB8 goes LOW; TX attempts are blocked
MON	Modulation ON (normal TX)	TX allowed (when RON)
MOF	Modulation OFF (carrier-only simulation)	DL prints “carrier only” message
PNF	Payload source = external (UART)	DL prints AX25-like label
PNN	Payload source = internal PN9	DL prints PN9 label
DL	Downlink burst (5 packets)	TX mode briefly, 5 “DL[…]” lines on USART1
CW	CW beacon (~2 seconds)	PB9 HIGH, PB13 toggles, CW messages on USART1
ST	Print status	Status line on USART1
