# STM32F401VE – ADDNICS COM Board (ADD1397B) Simulation

This firmware turns an STM32F401VE into a COM PIC–like controller to simulate the digital behavior of the ADDNICS COM subsystem used in BIRDS/CubeSat-style architectures.
It is designed for Proteus to validate logic, modes, GPIO control, UART interaction, and housekeeping (RSSI/TEMP).

---

## Simulation includes:

* COM PIC mode logic (RX / GMSK TX / CW beacon)
* Digital control lines to the transceiver (ON/OFF, CW enable, CW key)
* Configuration commands (RON/ROF, MON/MOF, PNF/PNN)
* A simplified “uplink frame” trigger via USART1
* Housekeeping sampling (RSSI and TEMP) via ADC

---

## UART interfaces

| Interface | STM32 Peripheral | Purpose | Typical STM32F401 Pins* | Connects to |
| :--- | :--- | :--- | :--- | :--- |
| **TRX UART** | USART1 | “Ground Station” terminal (uplink triggers & downlink text output) | PA9 (TX), PA10 (RX) | Virtual Terminal #1 |
| **CONFIG UART** | USART2 | Local config console (HELP, RON, DL, etc.) | PA2 (TX), PA3 (RX) | Virtual Terminal #2 |

---
<img width="1920" height="1080" alt="Screenshot (181)" src="https://github.com/user-attachments/assets/6b37b1e6-129b-400b-ad3c-4994a559375b" />


## CONFIG UART (USART2) commands

Type commands in Virtual Terminal #2 according to ICD Transceiver commands.

| Command | Meaning | It shows |
| :--- | :--- | :--- |
| **HELP** | Print help menu | Help text on CONFIG terminal |
| **RON** | RF output ON (enable TX) | PB8 goes HIGH; status reports RON |
| **ROF** | RF output OFF (block TX) | PB8 goes LOW; TX attempts are blocked |
| **MON** | Modulation ON (normal TX) | TX allowed (when RON) |
| **MOF** | Modulation OFF (carrier-only simulation) | DL prints “carrier only” message |
| **PNF** | Payload source = external (UART) | DL prints AX25-like label |
| **PNN** | Payload source = internal PN9 | DL prints PN9 label |
| **DL** | Downlink burst (5 packets) | TX mode briefly, 5 “DL[…]” lines on USART1 |
| **CW** | CW beacon (~2 seconds) | PB9 HIGH, PB13 toggles, CW messages on USART1 |
| **ST** | Print status | Status line on USART1 |



A simplified binary frame parser is implemented.

## Frame Format (Hex)

```
0x7E 0x7E 0x42 <CMD> 0x7E
```

Where:

- `0x7E` = Flag delimiter
- `0x42` = Frame identifier
- `<CMD>` = Command byte

## Supported Commands

| CMD (Hex) | Action |
|------------|--------|
| 0x77 | Automatic downlink burst |
| 0x20 | CW beacon |
| 0x30 | Send housekeeping |
| 0xAA | Sync/Keepalive |

Example frame:

```
7E 7E 42 77 7E
```


# Procedures to begin with simulation

### Boot
- Run simulation
- Verify both terminals show READY message
- LCD displays RX mode

### TX Blocking Test
```
ROF
DL
```
Expected: TX blocked

Then:
```
RON
DL
```
Expected: Downlink burst

---

### Modulation Test
```
MOF
DL
```
Expected: Carrier-only message

Then:
```
MON
```

### PN Source Test
```
PNN
DL
```
Expected: PN9 labeled packets

### CW Beacon Test
```
CW
```
Expected:
- PB9 HIGH
- PB13 toggling
- TX LED active
- Return to RX after ~2 seconds

```
ST
```


# Communication Protocol Overview

The simulation uses two UART interfaces with different purposes and data formats.

---

## UART Configuration

- Baudrate: **115200 bps** //must to be in this baudrate, but to begin, we set to 9600 
- Data format: **8N1** (8 data bits, no parity, 1 stop bit)
- Byte-based communication (1 byte = 8 bits)

---

## 1) CONFIG UART (USART2) – ASCII Commands

Used for local configuration and debugging.

Type: **ASCII text commands** terminated by CR (`0x0D`) or LF (`0x0A`).

Example:

```
RON
```

Transmitted bytes:

```
52 4F 4E 0D
```

Maximum command length:
```
32 bytes (CFG_LINE_MAX)
```

Data type used:
```
char cfg_line[32];
```

Purpose:
- Change COM state (RON, ROF, MON, MOF, PNF, PNN)
- Trigger actions (DL, CW, ST)

---

## 2) TRX UART (USART1) – Binary Framed Protocol

Used to simulate space communication (uplink/downlink).

Type: **Binary framed protocol**, not ASCII.

Frame structure:

```
0x7E 0x7E 0x42 <PAYLOAD> 0x7E
```

Where:

- `0x7E` = frame delimiter
- `0x42` = frame identifier
- `<PAYLOAD>` = 1–128 bytes
- Final `0x7E` = end delimiter

Payload format:

```
PAYLOAD[0] = Command byte
PAYLOAD[1..N] = Optional parameters
```

Maximum payload size:
```
128 bytes (TRX_MAX_PAYLOAD)
```

Data types:
```
uint8_t trx_payload[128];
uint16_t trx_pl_len;
```

---

## Supported Binary Commands

| Command (Hex) | Description |
|---------------|------------|
| 0x77 | Downlink burst |
| 0x20 | CW beacon |
| 0x30 | Housekeeping request |
| 0xAA | Sync/keepalive |

Example frame:

```
7E 7E 42 77 7E
```
