/*
RF69.h - Library for handling the HopeRF RFM69 Transceivers
Created by Franz Stoiber 2016/2017
The commercial use of this code is not allowed.
*/

#include "Arduino.h"
#include <Stdint.h>

//#include <heltec.h>

#define MSGBLK1 60	//3o6 length of the first rx block
#define FixPktSize 64 //max len of the encoded wMBus packet = FIFO Size of RFM69

#define PACKET_OK 0
#define PACKET_CODING_ERROR 1
#define PACKET_CRC_ERROR 2

#ifndef RFM69_h
#define RFM69_h

// Register names
#define RFM69_REG_00_FIFO 0x00
#define RFM69_REG_01_OPMODE 0x01
#define RFM69_REG_02_DATAMODUL 0x02
#define RFM69_REG_03_BITRATEMSB 0x03
#define RFM69_REG_04_BITRATELSB 0x04
#define RFM69_REG_05_FDEVMSB 0x05
#define RFM69_REG_06_FDEVLSB 0x06
#define RFM69_REG_07_FRFMSB 0x07
#define RFM69_REG_08_FRFMID 0x08
#define RFM69_REG_09_FRFLSB 0x09
#define RFM69_REG_0A_OSC1 0x0a
#define RFM69_REG_0B_AFCCTRL 0x0b
#define RFM69_REG_0C_RESERVED 0x0c
#define RFM69_REG_0D_LISTEN1 0x0d
#define RFM69_REG_0E_LISTEN2 0x0e
#define RFM69_REG_0F_LISTEN3 0x0f
#define RFM69_REG_10_VERSION 0x10
#define RFM69_REG_11_PALEVEL 0x11
#define RFM69_REG_12_PARAMP 0x12
#define RFM69_REG_13_OCP 0x13
#define RFM69_REG_14_RESERVED 0x14
#define RFM69_REG_15_RESERVED 0x15
#define RFM69_REG_16_RESERVED 0x16
#define RFM69_REG_17_RESERVED 0x17
#define RFM69_REG_18_LNA 0x18
#define RFM69_REG_19_RXBW 0x19
#define RFM69_REG_1A_AFCBW 0x1a
#define RFM69_REG_1B_OOKPEAK 0x1b
#define RFM69_REG_1C_OOKAVG 0x1c
#define RFM69_REG_1D_OOKFIX 0x1d
#define RFM69_REG_1E_AFCFEI 0x1e
#define RFM69_REG_1F_AFCMSB 0x1f
#define RFM69_REG_20_AFCLSB 0x20
#define RFM69_REG_21_FEIMSB 0x21
#define RFM69_REG_22_FEILSB 0x22
#define RFM69_REG_23_RSSICONFIG 0x23
#define RFM69_REG_24_RSSIVALUE 0x24
#define RFM69_REG_25_DIOMAPPING1 0x25
#define RFM69_REG_26_DIOMAPPING2 0x26
#define RFM69_REG_27_IRQFLAGS1 0x27
#define RFM69_REG_28_IRQFLAGS2 0x28
#define RFM69_REG_29_RSSITHRESH 0x29
#define RFM69_REG_2A_RXTIMEOUT1 0x2a
#define RFM69_REG_2B_RXTIMEOUT2 0x2b
#define RFM69_REG_2C_PREAMBLEMSB 0x2c
#define RFM69_REG_2D_PREAMBLELSB 0x2d
#define RFM69_REG_2E_SYNCCONFIG 0x2e
#define RFM69_REG_2F_SYNCVALUE1 0x2f
#define RFM69_REG_37_PACKETCONFIG1 0x37
#define RFM69_REG_38_PAYLOADLENGTH 0x38
#define RFM69_REG_39_NODEADRS 0x39
#define RFM69_REG_3A_BROADCASTADRS 0x3a
#define RFM69_REG_3B_AUTOMODES 0x3b
#define RFM69_REG_3C_FIFOTHRESH 0x3c
#define RFM69_REG_3D_PACKETCONFIG2 0x3d
#define RFM69_REG_3E_AESKEY1 0x3e
#define RFM69_REG_4E_TEMP1 0x4e
#define RFM69_REG_4F_TEMP2 0x4f
#define RFM69_REG_58_TESTLNA 0x58
#define RFM69_REG_5A_TESTPA1 0x5a
#define RFM69_REG_5C_TESTPA2 0x5c
#define RFM69_REG_6F_TESTDAGC 0x6f
#define RFM69_REG_71_TESTAFC 0x71

// RFM69_REG_01_OPMODE
#define RFM69_OPMODE_SEQUENCEROFF 0x80
#define RFM69_OPMODE_LISTENON 0x40
#define RFM69_OPMODE_LISTENABORT 0x20
#define RFM69_OPMODE_MODE 0x1c
#define RFM69_OPMODE_MODE_SLEEP 0x00
#define RFM69_OPMODE_MODE_STDBY 0x04
#define RFM69_OPMODE_MODE_FS 0x08
#define RFM69_OPMODE_MODE_TX 0x0c
#define RFM69_OPMODE_MODE_RX 0x10

// RFM69_REG_02_DATAMODUL
#define RFM69_DATAMODUL_DATAMODE 0x60
#define RFM69_DATAMODUL_DATAMODE_PACKET 0x00
#define RFM69_DATAMODUL_DATAMODE_CONT_WITH_SYNC 0x40
#define RFM69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC 0x60
#define RFM69_DATAMODUL_MODULATIONTYPE 0x18
#define RFM69_DATAMODUL_MODULATIONTYPE_FSK 0x00
#define RFM69_DATAMODUL_MODULATIONTYPE_OOK 0x08
#define RFM69_DATAMODUL_MODULATIONSHAPING 0x03
#define RFM69_DATAMODUL_MODULATIONSHAPING_FSK_NONE 0x00
#define RFM69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0 0x01
#define RFM69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_5 0x02
#define RFM69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_3 0x03
#define RFM69_DATAMODUL_MODULATIONSHAPING_OOK_NONE 0x00
#define RFM69_DATAMODUL_MODULATIONSHAPING_OOK_BR 0x01
#define RFM69_DATAMODUL_MODULATIONSHAPING_OOK_2BR 0x02

// RFM69_REG_11_PALEVEL
#define RFM69_PALEVEL_PA0ON 0x80
#define RFM69_PALEVEL_PA1ON 0x40
#define RFM69_PALEVEL_PA2ON 0x20
#define RFM69_PALEVEL_OUTPUTPOWER 0x1f

// RFM69_REG_13_OCP
#define RFM69_OCP_ON 0x1a
#define RFM69_OCP_OFF 0x0f

// RFM69_REG_23_RSSICONFIG
#define RFM69_RSSICONFIG_RSSIDONE 0x02
#define RFM69_RSSICONFIG_RSSISTART 0x01

// RFM69_REG_25_DIOMAPPING1
#define RFM69_DIOMAPPING1_DIO0MAPPING 0xc0
#define RFM69_DIOMAPPING1_DIO0MAPPING_00 0x00
#define RFM69_DIOMAPPING1_DIO0MAPPING_01 0x40
#define RFM69_DIOMAPPING1_DIO0MAPPING_10 0x80
#define RFM69_DIOMAPPING1_DIO0MAPPING_11 0xc0

#define RFM69_DIOMAPPING1_DIO1MAPPING 0x30
#define RFM69_DIOMAPPING1_DIO1MAPPING_00 0x00
#define RFM69_DIOMAPPING1_DIO1MAPPING_01 0x10
#define RFM69_DIOMAPPING1_DIO1MAPPING_10 0x20
#define RFM69_DIOMAPPING1_DIO1MAPPING_11 0x30

#define RFM69_DIOMAPPING1_DIO2MAPPING 0x0c
#define RFM69_DIOMAPPING1_DIO2MAPPING_00 0x00
#define RFM69_DIOMAPPING1_DIO2MAPPING_01 0x04
#define RFM69_DIOMAPPING1_DIO2MAPPING_10 0x08
#define RFM69_DIOMAPPING1_DIO2MAPPING_11 0x0c

#define RFM69_DIOMAPPING1_DIO3MAPPING 0x03
#define RFM69_DIOMAPPING1_DIO3MAPPING_00 0x00
#define RFM69_DIOMAPPING1_DIO3MAPPING_01 0x01
#define RFM69_DIOMAPPING1_DIO3MAPPING_10 0x02
#define RFM69_DIOMAPPING1_DIO3MAPPING_11 0x03

// RFM69_REG_26_DIOMAPPING2
#define RFM69_DIOMAPPING2_DIO4MAPPING 0xc0
#define RFM69_DIOMAPPING2_DIO4MAPPING_00 0x00
#define RFM69_DIOMAPPING2_DIO4MAPPING_01 0x40
#define RFM69_DIOMAPPING2_DIO4MAPPING_10 0x80
#define RFM69_DIOMAPPING2_DIO4MAPPING_11 0xc0

#define RFM69_DIOMAPPING2_DIO5MAPPING 0x30
#define RFM69_DIOMAPPING2_DIO5MAPPING_00 0x00
#define RFM69_DIOMAPPING2_DIO5MAPPING_01 0x10
#define RFM69_DIOMAPPING2_DIO5MAPPING_10 0x20
#define RFM69_DIOMAPPING2_DIO5MAPPING_11 0x30

#define RFM69_DIOMAPPING2_CLKOUT 0x07
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_ 0x00
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_2 0x01
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_4 0x02
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_8 0x03
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_16 0x04
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_32 0x05
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_RC 0x06
#define RFM69_DIOMAPPING2_CLKOUT_FXOSC_OFF 0x07

// RFM69_REG_27_IRQFLAGS1
#define RFM69_IRQFLAGS1_MODEREADY 0x80
#define RFM69_IRQFLAGS1_RXREADY 0x40
#define RFM69_IRQFLAGS1_TXREADY 0x20
#define RFM69_IRQFLAGS1_PLLLOCK 0x10
#define RFM69_IRQFLAGS1_RSSI 0x08
#define RFM69_IRQFLAGS1_TIMEOUT 0x04
#define RFM69_IRQFLAGS1_AUTOMODE 0x02
#define RFM69_IRQFLAGS1_SYNADDRESSMATCH 0x01

// RFM69_REG_28_IRQFLAGS2
#define RFM69_IRQFLAGS2_FIFOFULL 0x80
#define RFM69_IRQFLAGS2_FIFONOTEMPTY 0x40
#define RFM69_IRQFLAGS2_FIFOLEVEL 0x20
#define RFM69_IRQFLAGS2_FIFOOVERRUN 0x10
#define RFM69_IRQFLAGS2_PACKETSENT 0x08
#define RFM69_IRQFLAGS2_PAYLOADREADY 0x04
#define RFM69_IRQFLAGS2_CRCOK 0x02

// RFM69_REG_2E_SYNCCONFIG
#define RFM69_SYNCCONFIG_SYNCON 0x80
#define RFM69_SYNCCONFIG_FIFOFILLCONDITION_MANUAL 0x40
#define RFM69_SYNCCONFIG_SYNCSIZE 0x38
#define RFM69_SYNCCONFIG_SYNCSIZE_1 0x00
#define RFM69_SYNCCONFIG_SYNCSIZE_2 0x08
#define RFM69_SYNCCONFIG_SYNCSIZE_3 0x10
#define RFM69_SYNCCONFIG_SYNCSIZE_4 0x18
#define RFM69_SYNCCONFIG_SYNCSIZE_5 0x20
#define RFM69_SYNCCONFIG_SYNCSIZE_6 0x28
#define RFM69_SYNCCONFIG_SYNCSIZE_7 0x30
#define RFM69_SYNCCONFIG_SYNCSIZE_8 0x38
#define RFM69_SYNCCONFIG_SYNCSIZE_SYNCTOL 0x07

// RFM69_REG_37_PACKETCONFIG1
#define RFM69_PACKETCONFIG1_PACKETFORMAT_VARIABLE 0x80
#define RFM69_PACKETCONFIG1_DCFREE 0x60
#define RFM69_PACKETCONFIG1_DCFREE_NONE 0x00
#define RFM69_PACKETCONFIG1_DCFREE_MANCHESTER 0x20
#define RFM69_PACKETCONFIG1_DCFREE_WHITENING 0x40
#define RFM69_PACKETCONFIG1_DCFREE_RESERVED 0x60
#define RFM69_PACKETCONFIG1_CRC_ON 0x10
#define RFM69_PACKETCONFIG1_CRCAUTOCLEAROFF 0x08
#define RFM69_PACKETCONFIG1_ADDRESSFILTERING 0x06
#define RFM69_PACKETCONFIG1_ADDRESSFILTERING_NONE 0x00
#define RFM69_PACKETCONFIG1_ADDRESSFILTERING_NODE 0x02
#define RFM69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC 0x04
#define RFM69_PACKETCONFIG1_ADDRESSFILTERING_RESERVED 0x06

// RFM69_REG_3C_FIFOTHRESH
#define RFM69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY 0x80
#define RFM69_FIFOTHRESH_FIFOTHRESHOLD 0x7f

// RFM69_REG_3D_PACKETCONFIG2
#define RFM69_PACKETCONFIG2_INTERPACKETRXDELAY 0xf0
#define RFM69_PACKETCONFIG2_RESTARTRX 0x04
#define RFM69_PACKETCONFIG2_AUTORXRESTARTON 0x02
#define RFM69_PACKETCONFIG2_AESON 0x01

// RFM69_REG_4E_TEMP1
#define RFM69_TEMP1_TEMPMEASSTART 0x08
#define RFM69_TEMP1_TEMPMEASRUNNING 0x04

// RFM69_REG_5A_TESTPA1
#define RFM69_TESTPA1_NORMAL 0x55
#define RFM69_TESTPA1_BOOST 0x5d

// RFM69_REG_5C_TESTPA2
#define RFM69_TESTPA2_NORMAL 0x70
#define RFM69_TESTPA2_BOOST 0x7c

// RFM69_REG_6F_TESTDAGC
#define RFM69_TESTDAGC_CONTINUOUSDAGC_NORMAL 0x00
#define RFM69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAON 0x20
#define RFM69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF 0x30

#define RFM69_MAX_ENCRYPTABLE_PAYLOAD_LEN 64
#define RFM69_HEADER_LEN 4
#define RFM69_BROADCAST_ADDRESS 0xff
#define RFM69_MAX_MESSAGE_LEN (RFM69_MAX_ENCRYPTABLE_PAYLOAD_LEN - RFM69_HEADER_LEN)

// The crystal oscillator frequency of the RF69 module
#define RFM69_FXOSC 32000000.

// The Frequency Synthesizer step = RH_RF69_FXOSC / 2^^19
#define RFM69_FSTEP (RFM69_FXOSC / 524288.)

#define SPI_READ 0x7F
#define SPI_WRITE 0x80

enum rfm69type
{
	W = 0,
	CW,
	HW,
	HCW
};

enum modulation
{
	FSK = 0,
	GFSK,
	OOK
};

enum mode
{
	INITIALIZING = 0,
	SLEEP,
	STDBY,
	rfm69TX,
	rfm69RX
};

static const unsigned char LenOk = 0x1;
static const unsigned char LenErr = 0x2;
static const unsigned char DecErr = 0x4;
static const unsigned char CRCErr = 0x8;

class RFM69
{
public:
	boolean initDevice(unsigned char PinNSS, unsigned char PinDIO0, rfm69type DeviceType, float Frequency, modulation Modulation, unsigned long BitRate, unsigned long fdev, int PreambleLength, int8_t TxPower);
	boolean awaitSizedFrame(unsigned char Size, unsigned long Timeout);
	boolean receiveSizedFrame(unsigned char Size);
	unsigned char rxMBusMsg(); //returns status
	boolean waitForChannelFree(float RSSI_Threshold, unsigned long TimeoutFreeChannel);

	void setModulation(modulation Modulation);
	void setBitRateAndFdev(unsigned long BitRate);
	void setFrequency(float Frequency);
	void setSyncWords(unsigned char *SyncWords, unsigned char Len);
	void useSyncWords(boolean Using);

	unsigned char getLastRSSI();
	float convertRSSIToRSSIdBm(unsigned char RSSI);
	int readTemperature();
	void setCalibrationTemp(unsigned char Val);
	void setModeSleep();
	void setModeStdby();
	void setModeRx();
	void printRegisters();
	void printRegister(unsigned char Reg);
	void writeReg(uint8_t addr, uint8_t value);
	//unsigned char _RxBuffer[RFM69_MAX_MESSAGE_LEN];
	unsigned char _RxBuffer[255];
	unsigned char _RxBufferLen;
	boolean _HighPowerDevice;
	unsigned char _mbusmsg[180];
	unsigned char msgerr; // 0 nothing received, 1 len ok, 2 len error, 4 3o6 error
	unsigned char _msgLen;
	unsigned char writeSPI(unsigned char Reg, unsigned char Val);
	unsigned char readSPI(unsigned char Val);
	void readFifo(unsigned char *buffer, unsigned char length);
	void setBitRate(unsigned long BitRate);
	void setFDEV(unsigned long fdev);
	unsigned char decode3o6(unsigned char *encodedData, unsigned char *decodedData, unsigned char lastByte);
	unsigned char decode3o6Block(unsigned char *encoded, unsigned char *decoded, unsigned char encodedSize);
	uint16_t crcCalc(uint16_t crcReg, uint8_t crcData);

private:
	void calcRegValsRateAndFdev(unsigned long BitRate, unsigned char *RegVal);
	void setPreambleLength(int Bytes);
	void setOverCurrentProtection(boolean On);
	void setOpMode(unsigned char Mode);
	boolean checkIsChannelFree(float RSSI_Threshold);

	unsigned char writeSPIBurst(unsigned char Reg, const unsigned char *Src, unsigned char Len);
	
	unsigned char _PinNSS;
	unsigned char _PinDIO0;
	modulation _Modulation;
	mode _Mode;
	int8_t _Power;
	unsigned char _RSSILast;
	unsigned char _CalibrationTempVal;
};

void encode3outof6(uint8_t *uncodedData, uint8_t *encodedData, uint8_t lastByte);

// Table for encoding for a 4-bit data into 6-bit
// "3 out of 6" coded data.
static uint8_t encodeTab[16] = {0x16,  // 0x0 "3 out of 6" encoded
								0x0D,  // 0x1 "3 out of 6" encoded
								0x0E,  // 0x2 "3 out of 6" encoded
								0x0B,  // 0x3 "3 out of 6" encoded
								0x1C,  // 0x4 "3 out of 6" encoded
								0x19,  // 0x5 "3 out of 6" encoded
								0x1A,  // 0x6 "3 out of 6" encoded
								0x13,  // 0x7 "3 out of 6" encoded
								0x2C,  // 0x8 "3 out of 6" encoded
								0x25,  // 0x9 "3 out of 6" encoded
								0x26,  // 0xA "3 out of 6" encoded
								0x23,  // 0xB "3 out of 6" encoded
								0x34,  // 0xC "3 out of 6" encoded
								0x31,  // 0xD "3 out of 6" encoded
								0x32,  // 0xE "3 out of 6" encoded
								0x29}; // 0xF "3 out of 6" encoded

static unsigned char decodeTab[64] = {0xFF,  //  "3 out of 6" encoded 0x00 decoded
									  0xFF,  //  "3 out of 6" encoded 0x01 decoded
									  0xFF,  //  "3 out of 6" encoded 0x02 decoded
									  0xFF,  //  "3 out of 6" encoded 0x03 decoded
									  0xFF,  //  "3 out of 6" encoded 0x04 decoded
									  0xFF,  //  "3 out of 6" encoded 0x05 decoded
									  0xFF,  //  "3 out of 6" encoded 0x06 decoded
									  0xFF,  //  "3 out of 6" encoded 0x07 decoded
									  0xFF,  //  "3 out of 6" encoded 0x08 decoded
									  0xFF,  //  "3 out of 6" encoded 0x09 decoded
									  0xFF,  //  "3 out of 6" encoded 0x0A decoded
									  0x03,  //  "3 out of 6" encoded 0x0B decoded
									  0xFF,  //  "3 out of 6" encoded 0x0C decoded
									  0x01,  //  "3 out of 6" encoded 0x0D decoded
									  0x02,  //  "3 out of 6" encoded 0x0E decoded
									  0xFF,  //  "3 out of 6" encoded 0x0F decoded
									  0xFF,  //  "3 out of 6" encoded 0x10 decoded
									  0xFF,  //  "3 out of 6" encoded 0x11 decoded
									  0xFF,  //  "3 out of 6" encoded 0x12 decoded
									  0x07,  //  "3 out of 6" encoded 0x13 decoded
									  0xFF,  //  "3 out of 6" encoded 0x14 decoded
									  0xFF,  //  "3 out of 6" encoded 0x15 decoded
									  0x00,  //  "3 out of 6" encoded 0x16 decoded
									  0xFF,  //  "3 out of 6" encoded 0x17 decoded
									  0xFF,  //  "3 out of 6" encoded 0x18 decoded
									  0x05,  //  "3 out of 6" encoded 0x19 decoded
									  0x06,  //  "3 out of 6" encoded 0x1A decoded
									  0xFF,  //  "3 out of 6" encoded 0x1B decoded
									  0x04,  //  "3 out of 6" encoded 0x1C decoded
									  0xFF,  //  "3 out of 6" encoded 0x1D decoded
									  0xFF,  //  "3 out of 6" encoded 0x1E decoded
									  0xFF,  //  "3 out of 6" encoded 0x1F decoded
									  0xFF,  //  "3 out of 6" encoded 0x20 decoded
									  0xFF,  //  "3 out of 6" encoded 0x21 decoded
									  0xFF,  //  "3 out of 6" encoded 0x22 decoded
									  0x0B,  //  "3 out of 6" encoded 0x23 decoded
									  0xFF,  //  "3 out of 6" encoded 0x24 decoded
									  0x09,  //  "3 out of 6" encoded 0x25 decoded
									  0x0A,  //  "3 out of 6" encoded 0x26 decoded
									  0xFF,  //  "3 out of 6" encoded 0x27 decoded
									  0xFF,  //  "3 out of 6" encoded 0x28 decoded
									  0x0F,  //  "3 out of 6" encoded 0x29 decoded
									  0xFF,  //  "3 out of 6" encoded 0x2A decoded
									  0xFF,  //  "3 out of 6" encoded 0x2B decoded
									  0x08,  //  "3 out of 6" encoded 0x2C decoded
									  0xFF,  //  "3 out of 6" encoded 0x2D decoded
									  0xFF,  //  "3 out of 6" encoded 0x2E decoded
									  0xFF,  //  "3 out of 6" encoded 0x2F decoded
									  0xFF,  //  "3 out of 6" encoded 0x30 decoded
									  0x0D,  //  "3 out of 6" encoded 0x31 decoded
									  0x0E,  //  "3 out of 6" encoded 0x32 decoded
									  0xFF,  //  "3 out of 6" encoded 0x33 decoded
									  0x0C,  //  "3 out of 6" encoded 0x34 decoded
									  0xFF,  //  "3 out of 6" encoded 0x35 decoded
									  0xFF,  //  "3 out of 6" encoded 0x36 decoded
									  0xFF,  //  "3 out of 6" encoded 0x37 decoded
									  0xFF,  //  "3 out of 6" encoded 0x38 decoded
									  0xFF,  //  "3 out of 6" encoded 0x39 decoded
									  0xFF,  //  "3 out of 6" encoded 0x3A decoded
									  0xFF,  //  "3 out of 6" encoded 0x3B decoded
									  0xFF,  //  "3 out of 6" encoded 0x3C decoded
									  0xFF,  //  "3 out of 6" encoded 0x3D decoded
									  0xFF,  //  "3 out of 6" encoded 0x3E decoded
									  0xFF}; // "3 out of 6" encoded 0x3F decoded

#endif
