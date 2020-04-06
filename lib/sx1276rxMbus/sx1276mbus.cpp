#include "sx1276mbus.h"
#include <SPI.h>
#include "decoder3o6.h"

boolean SX1276MBUS::initDevice(unsigned char PinNSS, unsigned char PinDIO0)
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
    //SPI.setFrequency(100000);
    _PinDIO0 = PinDIO0;
    pinMode(_PinDIO0, INPUT);
    //check device
    delay(10);
    Type = readSPI(REG_VERSION);
    if (Type == 00 || Type == 0xFF)
        return (false);
    //init device mode
    _Mode = INITIALIZING;
    setModeStdby();
    RxChainCalibration();

     //setFSK a bit tricky, but should fit in this case
     writeSPI(REG_OPMODE, RF_OPMODE_MODULATIONTYPE_FSK);
     //Gausian shaping and default 40us rampup
     writeSPI(REG_PARAMP, RF_PARAMP_MODULATIONSHAPING_01 |  RF_PARAMP_0040_US);
    //set DAGC continuous automatic gain control)
    //ToDo writeSPI(RFM69_REG_6F_TESTDAGC, RFM69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAOFF);

    //set LNA AGC
    writeSPI(REG_LNA, RF_LNA_GAIN_G1);
    writeSPI(REG_RXCONFIG, RF_RXCONFIG_AGCAUTO_ON | RF_RXCONFIG_AFCAUTO_ON);
    //set DCC 4% and 333 kHz bandwidth
    writeSPI(REG_RXBW, RF_RXBW_MANT_16 |  RF_RXBW_EXP_1);  // 250 kHz ToDo not 100% for sure

    // ToDo writeSPI(REG_AFCBW, 0xF0); //was E0 changed to F0 = 333 kHz E8
    //writeSPI(RFM69_REG_1E_AFCFEI, 0x4); //AfcAutoOn
    //writeSPI(RFM69_REG_0B_AFCCTRL,0x20 );

    //set frequency
    setFrequency(868.95); // wMbus T1

    //set bit rate
    setBitRate(100000);
    setFDEV(50000);

    //preamble settings
    writeSPI(REG_PREAMBLEDETECT, RF_PREAMBLEDETECT_DETECTOR_ON | RF_PREAMBLEDETECT_DETECTORSIZE_2 | RF_PREAMBLEDETECT_DETECTORTOL_10);

    //set sync words
    unsigned char SyncBytes[] = { 0x54, 0x3D}; // lets start relaxed
    setSyncWords(SyncBytes, sizeof(SyncBytes));
    useSyncWords(true);

    writeSPI(REG_PACKETCONFIG1, RF_PACKETCONFIG1_PACKETFORMAT_FIXED );
    writeSPI(REG_PACKETCONFIG2, RF_PACKETCONFIG2_DATAMODE_PACKET);       // fixed length, MSBs of length = 0
    writeSPI(REG_PAYLOADLENGTH, FixPktSize); //max FIFO lenght
   
    
    //writeSPI(RFM69_REG_3C_FIFOTHRESH, FixPktSize);    //first mbus block

    return (true);
}

boolean SX1276MBUS::receiveSizedFrame(unsigned char Size)
{
    setModeRx();
    if (readSPI(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) //RFM69_IRQFLAGS2_FIFOLEVEL) //
    //if (digitalRead(this->_PinDIO0))
    {
        setModeStdby();

        //Serial.println("PayloadReady");
        //FIFO level was reached
        _RSSILast = readSPI(REG_RSSIVALUE);

        readFifo(_RxBuffer, Size);
        //this->msgerr = decode3o6(_RxBuffer,_mbusmsg,0);
        _RxBufferLen = Size;
        //_RxBufferLen = ((_mbusmsg[0] + 0x0001)* 3) >> 1; //(L-field +1 ) *1.5 (3o6)

        return (true);
    };

    return (false);
}

void SX1276MBUS::setFrequency(float Frequency)
{
    unsigned long Frf = (unsigned long)((Frequency * 1000000.0) / sx1276_FSTEP);
    writeSPI(REG_FRFMSB, (Frf >> 16) & 0xff);
    writeSPI(REG_FRFMID, (Frf >> 8) & 0xff);
    writeSPI(REG_FRFLSB, Frf & 0xff);
}

void SX1276MBUS::setFDEV( uint16_t fdev )
{
    uint16_t fdevtemp = ( uint16_t )( ( double )fdev / ( double )sx1276_FSTEP );
     writeSPI( REG_FDEVMSB, ( uint8_t )( fdevtemp >> 8 ) );
     writeSPI( REG_FDEVLSB, ( uint8_t )( fdevtemp & 0xFF ) );

}

void SX1276MBUS::setBitRate( uint32_t bitrate ) {

    uint16_t bitrateval = ( uint16_t )( ( double )sx1276_FXOSC / ( double )bitrate );
    writeSPI( REG_BITRATEMSB, ( uint8_t )( bitrateval >> 8 ) );
    writeSPI( REG_BITRATELSB, ( uint8_t )( bitrateval & 0xFF ) );
}



           

void SX1276MBUS::setSyncWords(unsigned char *SyncWords, unsigned char Len)
{
    unsigned char SyncConfig;
    if (Len > 8)
        Len = 8;
    if (Len > 0)
        writeSPIBurst(REG_SYNCVALUE1, SyncWords, Len);
    SyncConfig = readSPI(REG_SYNCCONFIG);
    SyncConfig &= RF_SYNCCONFIG_SYNCSIZE_MASK;
    SyncConfig |= (Len - 1);
    writeSPI(REG_SYNCCONFIG, SyncConfig);
}

void SX1276MBUS::useSyncWords(boolean Using)
{
    unsigned char SyncConfig;
    SyncConfig = readSPI(REG_SYNCCONFIG) & RF_SYNCCONFIG_SYNC_MASK; //mask deletes the sync bit
    if (Using)
        SyncConfig |=RF_SYNCCONFIG_SYNC_ON;
    //else it is already off

    writeSPI(REG_SYNCCONFIG, SyncConfig);
}

unsigned char SX1276MBUS::getLastRSSI()
{
    return _RSSILast;
}

float SX1276MBUS::convertRSSIToRSSIdBm(unsigned char RSSI)
{
    return -(float)(RSSI) / 2.;
}

void SX1276MBUS::setModeStdby()
{
    if (_Mode != STDBY)
    {
        setOpMode(RF_OPMODE_STANDBY);
        _Mode = STDBY;
    }
}

void SX1276MBUS::setModeRx()
{
    if (_Mode != sx1276RX)
    {
        memset(_RxBuffer, 0, sizeof(_RxBuffer));
       //debug writeSPI(RFM69_REG_25_DIOMAPPING1, RFM69_DIOMAPPING1_DIO0MAPPING_01); // Set interrupt line 0 PayloadReady
        setOpMode(RF_OPMODE_RECEIVER);                                      // Clears FIFO
        _Mode = sx1276RX;
    }
}

void SX1276MBUS::setOpMode(unsigned char opMode)
{
    writeSPI( REG_OPMODE, ( readSPI( REG_OPMODE ) & RF_OPMODE_MASK ) | opMode );

    while (!(readSPI(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY))
        ; //wWait for mode to change.
}

void SX1276MBUS::readFifo(unsigned char *buffer, unsigned char length)
{
    digitalWrite(_PinNSS, LOW);
    SPI.transfer(REG_FIFO); //send the start address with the write mask off
    //for (_RxBufferLen = 0; _RxBufferLen < length; _RxBufferLen++) _RxBuffer[_RxBufferLen] = SPI.transfer(0);
    for (unsigned char i = 0; i < length; i++)
    {
        buffer[i] = SPI.transfer(0);
    }
    digitalWrite(_PinNSS, HIGH);
}

unsigned char SX1276MBUS::readSPI(unsigned char Reg)
{
    unsigned char Val;
    digitalWrite(_PinNSS, LOW);
    SPI.transfer(Reg & SPI_READ);
    Val = SPI.transfer(0);
    digitalWrite(_PinNSS, HIGH);
    return (Val);
}

unsigned char SX1276MBUS::writeSPI(unsigned char Reg, unsigned char Val)
{
    unsigned char Status = 0;
    digitalWrite(_PinNSS, LOW);
    Status = SPI.transfer(Reg | SPI_WRITE);
    SPI.transfer(Val);
    digitalWrite(_PinNSS, HIGH);
    return (Status);
}

unsigned char SX1276MBUS::writeSPIBurst(unsigned char Reg, const unsigned char *Src, unsigned char Len)
{
    unsigned char Status = 0;
    digitalWrite(_PinNSS, LOW);
    Status = SPI.transfer(Reg | SPI_WRITE);
    while (Len--)
        SPI.transfer(*Src++);
    digitalWrite(_PinNSS, HIGH);
    return (Status);
}

/*!

 * Performs the Rx chain calibration for LF and HF bands

 * \remark Must be called just after the reset so all registers are at their

 *         default values

 */

void SX1276MBUS::RxChainCalibration(void)

{
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;
    // Save context
    regPaConfigInitVal = readSPI(REG_PACONFIG);
    initialFreq = (double)(((uint32_t)readSPI(REG_FRFMSB) << 16) |
                  ((uint32_t)readSPI(REG_FRFMID) << 8) |
                  ((uint32_t)readSPI(REG_FRFLSB))) *
                  (double)sx1276_FSTEP;
    // Cut the PA just in case, RFO output, power = -1 dBm
    writeSPI(REG_PACONFIG, 0x00);
    // Launch Rx chain calibration for LF band
    writeSPI(REG_IMAGECAL, (readSPI(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);

    while ((readSPI(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }
    // Sets a Frequency in HF band
    setFrequency(868.95);
    // Launch Rx chain calibration for HF band
    writeSPI(REG_IMAGECAL, (readSPI(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK) | RF_IMAGECAL_IMAGECAL_START);
    while ((readSPI(REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING) == RF_IMAGECAL_IMAGECAL_RUNNING)
    {
    }
    // Restore context
    writeSPI(REG_PACONFIG, regPaConfigInitVal);
}

/*
https://github.com/alexanderwachter/loramac-node/blob/master/src/radio/sx1276/sx1276.c
datarate = ( uint16_t )( ( double )XTAL_FREQ / ( double )datarate );

            writeSPI( REG_BITRATEMSB, ( uint8_t )( datarate >> 8 ) );

            writeSPI( REG_BITRATELSB, ( uint8_t )( datarate & 0xFF ) );



            writeSPI( REG_RXBW, GetFskBandwidthRegValue( bandwidth ) );

            writeSPI( REG_AFCBW, GetFskBandwidthRegValue( bandwidthAfc ) );



            writeSPI( REG_PREAMBLEMSB, ( uint8_t )( ( preambleLen >> 8 ) & 0xFF ) );

            writeSPI( REG_PREAMBLELSB, ( uint8_t )( preambleLen & 0xFF ) );



            if( fixLen == 1 )

            {

                writeSPI( REG_PAYLOADLENGTH, payloadLen );

            }

            else

            {

                writeSPI( REG_PAYLOADLENGTH, 0xFF ); // Set payload length to the maximum

            }



            writeSPI( REG_PACKETCONFIG1,

                         ( readSPI( REG_PACKETCONFIG1 ) &

                           RF_PACKETCONFIG1_CRC_MASK &

                           RF_PACKETCONFIG1_PACKETFORMAT_MASK ) |

                           ( ( fixLen == 1 ) ? RF_PACKETCONFIG1_PACKETFORMAT_FIXED : RF_PACKETCONFIG1_PACKETFORMAT_VARIABLE ) |

                           ( crcOn << 4 ) );

            writeSPI( REG_PACKETCONFIG2, ( readSPI( REG_PACKETCONFIG2 ) | RF_PACKETCONFIG2_DATAMODE_PACKET ) );

            */