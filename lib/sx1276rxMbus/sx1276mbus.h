#ifndef sx1276mbus_h
#define sx1276mbus_h

#include "Arduino.h"
#include <Stdint.h>

#include "sx1276Regs-Fsk.h"

#define FixPktSize 48

// The crystal oscillator frequency of the RF69 module
#define sx1276_FXOSC 32000000.

// The Frequency Synthesizer step = FXOSC / 2^^19
#define sx1276_FSTEP (sx1276_FXOSC / 524288.)

#define SPI_READ 0x7F
#define SPI_WRITE 0x80



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
	sx1276TX,
	sx1276RX
};

class SX1276MBUS
{
public:
	boolean initDevice(unsigned char PinNSS, unsigned char PinDIO0);
	boolean receiveSizedFrame(unsigned char Size, unsigned char minRSSI);
	
	void setFDEV( uint32_t fdev );
	void setFrequency(float Frequency);
	void setBitRate( uint32_t bitrate );
	void setSyncWords(unsigned char *SyncWords, unsigned char Len);
	void useSyncWords(boolean Using);

	unsigned char getLastRSSI();
	float convertRSSIToRSSIdBm(unsigned char RSSI);
	int readTemperature();
	void setCalibrationTemp(unsigned char Val);
	void setModeStdby();
	void setModeRx();
	//unsigned char _RxBuffer[RFM69_MAX_MESSAGE_LEN];
	unsigned char _RxBuffer[FixPktSize];
	unsigned char _RxBufferLen;
	boolean _HighPowerDevice;
	unsigned char _mbusmsg[FixPktSize*2/3];
	unsigned char msgerr; // 0 nothing received, 1 len ok, 2 len error, 4 3o6 error
	unsigned char _msgLen;
	unsigned char writeSPI(unsigned char Reg, unsigned char Val);
	unsigned char readSPI(unsigned char Val);
	void readFifo(unsigned char *buffer, unsigned char length);
	
	void RxChainCalibration(void);
	uint16_t crcCalc(uint16_t crcReg, uint8_t crcData);

private:
	void calcRegValsRateAndFdev(unsigned long BitRate, unsigned char *RegVal);
	void setPreambleLength(int Bytes);
	void setOverCurrentProtection(boolean On);
	void setOpMode(unsigned char Mode);

	unsigned char writeSPIBurst(unsigned char Reg, const unsigned char *Src, unsigned char Len);
	
	unsigned char _PinNSS;
	unsigned char _PinDIO0;
	modulation _Modulation;
	mode _Mode;
	int8_t _Power;
	unsigned char _RSSILast;
	unsigned char _CalibrationTempVal;
};

#endif //RFM69_h