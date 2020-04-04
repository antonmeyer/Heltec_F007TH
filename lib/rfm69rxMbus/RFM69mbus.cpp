/*
RFM69.cpp - Library for handling the HopeRF RFM69 transceivers
Created by Franz Stoiber 2016/2017
The commercial use of this code is not allowed.

Hints -------------------------------------------------------
Use Arduino with VCC 3.3V or level shifter
This code uses no interrupts
-------------------------------------------------------------

History -----------------------------------------------------
2016-06-27 begin programming
2016-07-09 send and receive is working
2016-07-22 initial version
2016-07-31 bugfix in convertRSSIToRSSIdBm
2016-08-07 bugfix setModeIdle renamed to setModeStdby
new function setModeSleep
2017-02-01 support for HW and HCW moduls
minor bugfixes
2017-02-08 bugfix in setting frequency deviation
2017-02-10 improved performance
2017-02-25 new function waitForChannelFree
-------------------------------------------------------------
*/

#include "RFM69mbus.h"
#include <SPI.h>

boolean RFM69::initDevice(unsigned char PinNSS, unsigned char PinDIO0, rfm69type DeviceType, float Frequency, modulation Modulation, unsigned long BitRate, unsigned long fdev, int PreambleLength, int8_t TxPower)
{
	//defaults to packet mode, variable length, whitening, CRC on, no address filtering
	//no syncword, no encryption, RSSI Threshold -114dBm, power 13, mode stdby
	//----------------------------------------------------------------------------------------------
	unsigned char Type;
	unsigned char SyncConfig;
	//init SPI
	_PinNSS = PinNSS;
	pinMode(_PinNSS, OUTPUT);
	digitalWrite(_PinNSS, HIGH);
	SPI.begin();
	//SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
	//init digital i/o DIO0
	SPI.setFrequency(100000);
	_PinDIO0 = PinDIO0;
	pinMode(_PinDIO0, INPUT);
	//set device type
	if (DeviceType == W || DeviceType == CW)
		_HighPowerDevice = false;
	else if (DeviceType == HW || DeviceType == HCW)
		_HighPowerDevice = true;
	else
		return (false);
	//check device
	delay(10);
	Type = readSPI(RFM69_REG_10_VERSION);
	if (Type == 00 || Type == 0xFF)
		return (false);
	//init device mode
	_Mode = INITIALIZING;
	setModeStdby();
	//set DAGC continuous automatic gain control)
	writeSPI(RFM69_REG_6F_TESTDAGC, RFM69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);

	//set LNA by AGC and 200 Ohm 0x8 / 0x00 50 Ohm
	writeSPI(RFM69_REG_18_LNA, 0x00);
	//set DCC 4% and 333 kHz bandwidth
	writeSPI(RFM69_REG_19_RXBW, 0xF0);  //010    10 000
	writeSPI(RFM69_REG_1A_AFCBW, 0xF0); //was E0 changed to F0 = 333 kHz E8
	//writeSPI(RFM69_REG_1E_AFCFEI, 0x4); //AfcAutoOn
	//writeSPI(RFM69_REG_0B_AFCCTRL,0x20 );

	//set frequency
	setFrequency(Frequency);
	
	//set bit rate
	setBitRate(BitRate);
	setFDEV(fdev);
	//set packet mode in data mode
	//writeSPI(RFM69_REG_02_DATAMODUL, readSPI(RFM69_REG_02_DATAMODUL) & 0x9F);
	writeSPI(RFM69_REG_02_DATAMODUL, RFM69_DATAMODUL_DATAMODE_PACKET);
	//set modulation
	setModulation(Modulation);
	//set PreambleLength
	setPreambleLength(PreambleLength);
	//set no sync word
	SyncConfig = readSPI(RFM69_REG_2E_SYNCCONFIG);
	writeSPI(RFM69_REG_2E_SYNCCONFIG, SyncConfig & 0x7F);
	//set no encryption

	//set sync words
	unsigned char SyncBytes[] = {0x54, 0x3D}; // lets start relaxed
	setSyncWords(SyncBytes, sizeof(SyncBytes));
	useSyncWords(true);

	//rfm69.writeSPI(RFM69_REG_38_PAYLOADLENGTH, 0);	// unlimited
	writeSPI(RFM69_REG_38_PAYLOADLENGTH, FixPktSize); //max FIFO lenght
	writeSPI(RFM69_REG_37_PACKETCONFIG1, 0x00);		  // fixed length, noCRC, no address filter 0x4 to set interrupt without CRC???
	writeSPI(RFM69_REG_3C_FIFOTHRESH, FixPktSize); //first mbus block
	
	return (true);
}

boolean RFM69::waitForChannelFree(float RSSI_Threshold, unsigned long TimeoutFreeChannel)
{
	unsigned long WaitUntilTime;
	//wait for a free channel, if CSMA/CA algorithm is enabled
	//this takes around 1,4 ms to finish if channel is free
	WaitUntilTime = millis() + TimeoutFreeChannel;
	while (!checkIsChannelFree(RSSI_Threshold))
	{
		//channel not free
		if ((long)(millis() - WaitUntilTime) > 0)
			return (false);
		delay(5);
	}
	return (true);
}

boolean RFM69::checkIsChannelFree(float RSSI_Threshold)
{
	unsigned long TimeoutTime;
	float RSSI;
	//restart RX
	writeSPI(RFM69_REG_3D_PACKETCONFIG2, (readSPI(RFM69_REG_3D_PACKETCONFIG2) & 0xFB) | 0x20);
	setModeRx();
	//wait until RSSI sampling is done otherwise 0xFF (-127 dBm) is read
	//RSSI sampling phase takes ~960 ?s after switch from standby to Rx
	TimeoutTime = millis() + 5;
	while ((readSPI(RFM69_REG_23_RSSICONFIG) & RFM69_RSSICONFIG_RSSIDONE) == 0)
	{
		if ((long)(millis() - TimeoutTime) > 0)
			break;
	}
	RSSI = -(float)(readSPI(RFM69_REG_24_RSSIVALUE)) / 2.;
	if (RSSI > RSSI_Threshold)
		return (false);
	else
		return (true);
}

boolean RFM69::awaitSizedFrame(unsigned char Size, unsigned long Timeout)
{
	unsigned long TimeoutTime;
	TimeoutTime = millis() + Timeout;
	while ((long)(millis() - TimeoutTime) < 0)
	{
		if (receiveSizedFrame(Size))
			return (true);
	}
	setModeStdby();
	return (false);
}

boolean RFM69::receiveSizedFrame(unsigned char Size)
{
	setModeRx();
	if (readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_PAYLOADREADY) //RFM69_IRQFLAGS2_FIFOLEVEL) //
	//if (digitalRead(this->_PinDIO0))
	{	setModeStdby();

		//Serial.println("PayloadReady");
		//FIFO level was reached
		_RSSILast = readSPI(RFM69_REG_24_RSSIVALUE);

		readFifo(_RxBuffer, Size); 
		//this->msgerr = decode3o6(_RxBuffer,_mbusmsg,0);
		_RxBufferLen = Size;
		//_RxBufferLen = ((_mbusmsg[0] + 0x0001)* 3) >> 1; //(L-field +1 ) *1.5 (3o6)
		
		return (true);
	};

	return (false);
}

void RFM69::setModulation(modulation Modulation)
{
	unsigned char RegVal;
	RegVal = readSPI(RFM69_REG_02_DATAMODUL);
	RegVal &= 0xE4;
	switch (Modulation)
	{
	case FSK:
		writeSPI(RFM69_REG_02_DATAMODUL, RegVal | 0x00);
		break;
	case GFSK:
		writeSPI(RFM69_REG_02_DATAMODUL, RegVal | 0x01);
		break;
	case OOK:
		writeSPI(RFM69_REG_02_DATAMODUL, RegVal | 0x08);
		break;
	}
	_Modulation = Modulation;
}

void RFM69::setBitRate(unsigned long BitRate)
{
	unsigned int RateVal;
	float Fosc = RFM69_FXOSC;
	if (BitRate < 1200)
		BitRate = 1200;
	if (BitRate > 300000)
		BitRate = 300000;
	if (_Modulation == OOK && BitRate > 19200)
		BitRate = 19200;
	RateVal = (unsigned int)(round(Fosc / BitRate));
	writeSPI(RFM69_REG_03_BITRATEMSB, highByte(RateVal));
	writeSPI(RFM69_REG_04_BITRATELSB, lowByte(RateVal));
}

void RFM69::setFDEV(unsigned long fdev)
{
	unsigned int Val;
	float Fstep = RFM69_FSTEP;
	Val = round(float(fdev) / Fstep);
	writeSPI(RFM69_REG_05_FDEVMSB, highByte(Val));
	writeSPI(RFM69_REG_06_FDEVLSB, lowByte(Val));
}

void RFM69::setFrequency(float Frequency)
{
	//if (Frequency < 424.0) Frequency = 424.0;
	//if (Frequency > 510.0) Frequency = 510.0;
	unsigned long Frf = (unsigned long)((Frequency * 1000000.0) / RFM69_FSTEP);
	writeSPI(RFM69_REG_07_FRFMSB, (Frf >> 16) & 0xff);
	writeSPI(RFM69_REG_08_FRFMID, (Frf >> 8) & 0xff);
	writeSPI(RFM69_REG_09_FRFLSB, Frf & 0xff);
}

void RFM69::setPreambleLength(int Len)
{
	writeSPI(RFM69_REG_2C_PREAMBLEMSB, Len >> 8);
	writeSPI(RFM69_REG_2D_PREAMBLELSB, Len & 0xff);
}

void RFM69::setSyncWords(unsigned char *SyncWords, unsigned char Len)
{
	unsigned char SyncConfig;
	if (Len > 8)
		Len = 8;
	if (Len > 0)
		writeSPIBurst(RFM69_REG_2F_SYNCVALUE1, SyncWords, Len);
	SyncConfig = readSPI(RFM69_REG_2E_SYNCCONFIG);
	SyncConfig &= 0x7F;
	SyncConfig &= ~RFM69_SYNCCONFIG_SYNCSIZE;
	SyncConfig |= (Len - 1) << 3;
	writeSPI(RFM69_REG_2E_SYNCCONFIG, SyncConfig);
}

void RFM69::useSyncWords(boolean Using)
{
	unsigned char SyncConfig;
	SyncConfig = readSPI(RFM69_REG_2E_SYNCCONFIG);
	if (Using)
		SyncConfig |= 0x80;
	else
		SyncConfig &= 0x7F;
	writeSPI(RFM69_REG_2E_SYNCCONFIG, SyncConfig);
}

void RFM69::setOverCurrentProtection(boolean On)
{
	if (On)
		writeSPI(RFM69_REG_13_OCP, RFM69_OCP_ON);
	else
		writeSPI(RFM69_REG_13_OCP, RFM69_OCP_OFF);
}

unsigned char RFM69::getLastRSSI()
{
	return _RSSILast;
}

float RFM69::convertRSSIToRSSIdBm(unsigned char RSSI)
{
	return -(float)(RSSI) / 2.;
}

int RFM69::readTemperature()
{
	int Temp;
	unsigned char LastMode;
	LastMode = _Mode;
	setModeStdby();
	writeSPI(RFM69_REG_4E_TEMP1, RFM69_TEMP1_TEMPMEASSTART);
	while (readSPI(RFM69_REG_4E_TEMP1) & RFM69_TEMP1_TEMPMEASRUNNING)
		;
	Temp = _CalibrationTempVal - readSPI(RFM69_REG_4F_TEMP2);
	switch (LastMode)
	{
	case rfm69RX:
		setModeRx();
		break;
	}
	return (Temp);
}

void RFM69::setCalibrationTemp(unsigned char Val)
{
	_CalibrationTempVal = Val;
}

void RFM69::setModeSleep()
{
	if (_Mode != SLEEP)
	{
		if (_HighPowerDevice)
		{
			if (_Power >= 14)
			{
				setOverCurrentProtection(true);
			}
		}
		setOpMode(RFM69_OPMODE_MODE_SLEEP);
		_Mode = SLEEP;
	}
}

void RFM69::setModeStdby()
{
	if (_Mode != STDBY)
	{
		if (_HighPowerDevice)
		{
			if (_Power >= 14)
			{
				setOverCurrentProtection(true);
			}
		}
		setOpMode(RFM69_OPMODE_MODE_STDBY);
		_Mode = STDBY;
	}
}

void RFM69::setModeRx()
{
	if (_Mode != rfm69RX)
	{
		if (_HighPowerDevice)
		{
			if (_Power >= 14)
			{
				setOverCurrentProtection(true);
			}
		}
		
		memset(_RxBuffer, 0, sizeof(_RxBuffer));
		memset(_mbusmsg, 0, sizeof(_mbusmsg));
		writeSPI(RFM69_REG_25_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
		setOpMode(RFM69_OPMODE_MODE_RX);									  // Clears FIFO
		_Mode = rfm69RX;
		
	}
	
}

void RFM69::setOpMode(unsigned char Mode)
{
	unsigned char OpMode = readSPI(RFM69_REG_01_OPMODE);
	OpMode &= ~RFM69_OPMODE_MODE;		  //clear bits 4-2 (mode)
	OpMode |= (Mode & RFM69_OPMODE_MODE); //set bits 4-2 (mode)
	writeSPI(RFM69_REG_01_OPMODE, OpMode);
	while (!(readSPI(RFM69_REG_27_IRQFLAGS1) & RFM69_IRQFLAGS1_MODEREADY))
		; //wWait for mode to change.
}

void RFM69::readFifo(unsigned char *buffer, unsigned char length)
{
	digitalWrite(_PinNSS, LOW);
	SPI.transfer(RFM69_REG_00_FIFO); //send the start address with the write mask off
	//for (_RxBufferLen = 0; _RxBufferLen < length; _RxBufferLen++) _RxBuffer[_RxBufferLen] = SPI.transfer(0);
	for (unsigned char i = 0; i < length; i++)
	{
		buffer[i] = SPI.transfer(0);
	}
	digitalWrite(_PinNSS, HIGH);
}

unsigned char RFM69::readSPI(unsigned char Reg)
{
	unsigned char Val;
	digitalWrite(_PinNSS, LOW);
	SPI.transfer(Reg & SPI_READ);
	Val = SPI.transfer(0);
	digitalWrite(_PinNSS, HIGH);
	return (Val);
}

unsigned char RFM69::writeSPI(unsigned char Reg, unsigned char Val)
{
	unsigned char Status = 0;
	digitalWrite(_PinNSS, LOW);
	Status = SPI.transfer(Reg | SPI_WRITE);
	SPI.transfer(Val);
	digitalWrite(_PinNSS, HIGH);
	return (Status);
}

unsigned char RFM69::writeSPIBurst(unsigned char Reg, const unsigned char *Src, unsigned char Len)
{
	unsigned char Status = 0;
	digitalWrite(_PinNSS, LOW);
	Status = SPI.transfer(Reg | SPI_WRITE);
	while (Len--)
		SPI.transfer(*Src++);
	digitalWrite(_PinNSS, HIGH);
	return (Status);
}

void RFM69::printRegisters()
{
	unsigned char i;
	for (i = 0; i < 0x50; i++)
		printRegister(i);
	// Non-contiguous registers
	printRegister(RFM69_REG_58_TESTLNA);
	printRegister(RFM69_REG_5A_TESTPA1);
	printRegister(RFM69_REG_5C_TESTPA2);
	printRegister(RFM69_REG_6F_TESTDAGC);
	printRegister(RFM69_REG_71_TESTAFC);
}

void RFM69::printRegister(unsigned char Reg)
{
	Serial.print(Reg, HEX);
	Serial.print(" ");
	Serial.println(readSPI(Reg), HEX);
}

#define DECODING_3OUTOF6_OK 0
#define DECODING_3OUTOF6_ERROR 1

unsigned char RFM69::decode3o6(unsigned char *encodedData, unsigned char *decodedData, unsigned char lastByte)
{
	unsigned char data[4];

	// - Perform decoding on the input data -
	if (!lastByte)
	{
		data[0] = decodeTab[(*(encodedData + 2) & 0x3F)];
		data[1] = decodeTab[((*(encodedData + 2) & 0xC0) >> 6) | ((*(encodedData + 1) & 0x0F) << 2)];
	}
	// If last byte, ignore postamble sequence
	else
	{
		data[0] = 0x00;
		data[1] = 0x00;
	}

	data[2] = decodeTab[((*(encodedData + 1) & 0xF0) >> 4) | ((*encodedData & 0x03) << 4)];
	data[3] = decodeTab[((*encodedData & 0xFC) >> 2)];

	// - Check for invalid data coding -
	if ((data[0] == 0xFF) | (data[1] == 0xFF) |
		(data[2] == 0xFF) | (data[3] == 0xFF))

		return (DecErr);
	// - Shift the encoded values into a unsigned char buffer -
	*decodedData = (data[3] << 4) | (data[2]);
	if (!lastByte)
		*(decodedData + 1) = (data[1] << 4) | (data[0]);

	return (0); // no error
}

unsigned char RFM69::decode3o6Block(unsigned char *encoded, unsigned char *decoded, unsigned char encodedSize)
{

	unsigned char bytesRemaining;
	unsigned char bytesEncoded;
	unsigned char decodingStatus =0;

	bytesRemaining = encodedSize;
	bytesEncoded = 0;

	while (bytesRemaining)
	{

		// If last byte
		if (bytesRemaining == 1)
		{
			decodingStatus = decode3o6(encoded, decoded, 1);
			if (decodingStatus == DecErr)
				return (DecErr);
			bytesRemaining -= 1;
			bytesEncoded += 1;
		}
		else
		{
			decodingStatus = decode3o6(encoded, decoded, 0);
			bytesRemaining -= 2;
			bytesEncoded += 2;

			encoded += 3;
			decoded += 2;
		}
	}
	return decodingStatus;
}

//----------------------------------------------------------------------------
// void encode3outof6 (uint8 *uncodedData, uint8 *encodedData, uint8 lastByte)
//
//  DESCRIPTION:
//    Performs the "3 out 6" encoding on a 16-bit data value into a
//    24-bit data value. When encoding on a 8 bit variable, a postamle
//    sequence is added.
//
//  ARGUMENTS:
//        uint8 *uncodedData      - Pointer to data
//        uint8 *encodedData      - Pointer to store the encoded data
//        uint8 lastByte          - Only one byte left in data buffer
//----------------------------------------------------------------------------

void encode3outof6(uint8_t *uncodedData, uint8_t *encodedData, uint8_t lastByte)
{

	uint8_t data[4];

	// - Perform encoding -

	// If last byte insert postamble sequence
	if (lastByte)
	{
		data[1] = 0x14;
	}
	else
	{
		data[0] = encodeTab[*(uncodedData + 1) & 0x0F];
		data[1] = encodeTab[(*(uncodedData + 1) >> 4) & 0x0F];
	}

	data[2] = encodeTab[(*uncodedData) & 0x0F];
	data[3] = encodeTab[((*uncodedData) >> 4) & 0x0F];

	// - Shift the encoded 6-bit values into a byte buffer -
	*(encodedData + 0) = (data[3] << 2) | (data[2] >> 4);
	*(encodedData + 1) = (data[2] << 4) | (data[1] >> 2);

	if (!lastByte)
	{
		*(encodedData + 2) = (data[1] << 6) | data[0];
	}
}

unsigned char RFM69::rxMBusMsg()
{
	/*this procedure reads the whole packet.
	the lenght is written in the first position
	as the 3outof6 encoding leads to a longer frame than the FIFO 
	can hold, we read in several chunks ..which seams to be a bit tricky
	threshhold should not be to high, to avoid FIFO overrun
	finally we kept this code for reference
	*/
	msgerr = 0;
	setModeRx();
	//if(digitalRead(_PinDIO0)) {
	//if (readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_PAYLOADREADY) {
	if (readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_FIFOLEVEL)
	{

		//payload ready, message has been received with good CRC
		_RSSILast = readSPI(RFM69_REG_24_RSSIVALUE);

		readFifo(&_RxBuffer[0], MSGBLK1); // read the first block
		this->msgerr = decode3o6(_RxBuffer, _mbusmsg, 0);

		_msgLen = _mbusmsg[0];
		//the tricky part: nobody nows if L-field is valid
		if (!(this->msgerr) && (_msgLen > 10) && (_msgLen < 100))
		{

			_RxBufferLen = _msgLen + (_msgLen >> 1) + 6; //(L-field +1 ) *1.5 (3o6) + CRC 2byte + Lenbyte

			// ToDo we want to have several chunks, 2/3rd of the fifo size could be good
			//unsigned char part2 = (_RxBufferLen -MSGBLK1) >> 1; //we want to have 3 parts
			//unsigned char  part3 = _RxBufferLen -MSGBLK1 - part2;

			unsigned char part2 = (_RxBufferLen - MSGBLK1);
			writeSPI(RFM69_REG_3C_FIFOTHRESH, (part2));

			unsigned char timeout = millis() + part2 * 8 / 100 + 5; // 1000000 bit/s -> 100 bit /ms plus safety margin

			while (!(readSPI(RFM69_REG_28_IRQFLAGS2) & (RFM69_IRQFLAGS2_FIFOLEVEL | RFM69_IRQFLAGS2_FIFOOVERRUN)) && (timeout > millis()))
			{
			}; //busy loop

			//check for timeout
			if (!(readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_FIFOLEVEL))
			{
				this->msgerr = this->msgerr | LenErr;
				//setModeStdby();
				//return (this->msgerr);
			}

			readFifo(&(_RxBuffer[MSGBLK1]), part2);

			/*
			writeSPI(RFM69_REG_3C_FIFOTHRESH, (part3));
			timeout = millis() + part3*8/100; // 1000000 bit/s -> 100 bit /ms plus safety margin
			while (!(readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_FIFOLEVEL) && (timeout > millis())) {}; //busy loop
			//check for timeout
			if (!(readSPI(RFM69_REG_28_IRQFLAGS2) & RFM69_IRQFLAGS2_FIFOLEVEL)) { // try RFM69_IRQFLAGS2_FIFONOTEMPTY
				this->msgerr = this->msgerr | 0x8;
				//setModeStdby();
				//return (this->msgerr);
			}
			
			readFifo(&_RxBuffer[18+part2],part3+1);
			*/
			setModeStdby();
			return (this->msgerr | LenOk);
		}
		else
		{

			setModeStdby();
			//setModeRx();
			return (0);
		}; //CRC was not ok
	}
	return (false);
}

uint16_t RFM69::crcCalc(uint16_t crcReg, uint8_t crcData)
{
#define CRC_POLYNOM 0x3D65
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		// If upper most bit is 1
		if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
			crcReg = (crcReg << 1) ^ CRC_POLYNOM;
		else
			crcReg = (crcReg << 1);

		crcData <<= 1;
	}

	return crcReg;
}
