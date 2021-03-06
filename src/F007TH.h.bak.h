#include <Arduino.h>
#include <U8x8lib.h>
//#include <heltec.h>
#include <Wire.h>
#include "crc.h"
//ToDo clean that lib and object oriented conecpt

U8X8_SSD1306_128X64_NONAME_HW_I2C OLED(/*OLED_RST*/ 16, /*OLED_SCL*/ 15, /*OLED_SDA*/ 4);
//U8X8_SSD1306_128X64_NONAME_SW_I2C OLED(/* clock=*/15, /* data=*/4, /* reset=*/16);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 (U8G2_R0, 16, 15, 4)

// 1.3" OLED
//U8X8_SH1106_128X64_NONAME_HW_I2C OLED(/* reset=*/U8X8_PIN_NONE);

// Variables for Manchester Receiver Logic:
word sDelay = 242;         // Small Delay about 1/4 of bit duration
word lDelay = 484;         // Long Delay about 1/2 of bit duration, 1/4 + 1/2 = 3/4
byte polarity = 1;         // 0 for lo->hi==1 or 1 for hi->lo==1 for Polarity, sets tempBit at start
byte tempBit = 1;          // Reflects the required transition polarity
boolean firstZero = false; // flags when the first '0' is found.
// Variables for Header detection
byte headerBits = 11; // The number of ones expected to make a valid header
byte headerHits = 0;  // Counts the number of "1"s to determine a header
// Variables for Byte storage
byte dataByte = 0; // Accumulates the bit information
byte nosBits = 6;  // Counts to 8 bits within a dataByte
#define maxBytes 7 // Set the bytes collected after each header. NB if set too high, any end noise will cause an error
byte nosBytes = 0; // Counter stays within 0 -> maxBytes
// Variables for multiple packets

byte dataarray[7]; // Array to store 7 bytes of manchester pattern decoded on the fly

// Variables to prepare recorded values for Ambient

byte stnId = 0; // Identifies the channel number

int Newtemp = 0;
int Newhum = 0;
int chTemp[8];
int chHum[8];
unsigned long chLastRecv[8];

char displaystr[20];
float siTemp;

char tempstr[8][6]; //floating point string per Channel

volatile unsigned long EdgeTime; // zur Speicherung der Zeit

unsigned long Dstop = 0;

byte rxstate = 0;

// Interrupt Service Routine for a falling edge
void IRAM_ATTR RF_ISR()
{
  EdgeTime = micros();
} // end of isr

char *temp2str(byte idx)
{
  //helper function to convert temp into °C and string

  siTemp = 0.0556 * (chTemp[idx] - 720);
  dtostrf(siTemp, 2, 1, tempstr[idx]);
  return tempstr[idx];
}

void saveReading(int stnId, int newTemp, int newHum)
{

  if (stnId >= 0 && stnId <= 7)
  {
    chTemp[stnId] = newTemp;
    chHum[stnId] = newHum;

    unsigned long now = millis();
    uint32_t diff = now - chLastRecv[stnId];
    if (diff > 1000)
    {
      // checks above seems to avoid too many outpuut
      //digitalWrite(7,HIGH); //is for Oszi trigger
      //delay(1);

      chLastRecv[stnId] = now;

      int stnId1 = stnId + 1;
      Serial.print(stnId1);
      Serial.print(":");

      Serial.print(temp2str(stnId));
      Serial.print(":");
      Serial.println(newHum);

      //OLED.clearLine(stnId1);

      snprintf(displaystr, 17, "%d:%sC %2d%% %4d", stnId1, tempstr[stnId], newHum, diff / 1000);
      OLED.drawString(0, stnId1, displaystr);
      // Heltec.display->drawString(0, stnId1*8, displaystr);
      // Heltec.display->display();
    }
  }
}

void add(byte bitData)
{
  dataByte = (dataByte << 1) | bitData;
  nosBits++;
  if (nosBits == 8)
  {
    nosBits = 0;
    dataarray[nosBytes] = dataByte;
    nosBytes++;
  }
  if (nosBytes == maxBytes)
  {
    rxstate = 0; // we got all bytes lets start for the next packet
    // Subroutines to extract data from Manchester encoding and error checking

    // Identify channels 0 to 7 by looking at 3 bits in byte 3

    // Identify sensor by looking for sensorID in byte 1 (F007th Ambient Thermo-Hygrometer = 0x45)

    /*
    for (byte j = 1; j < maxBytes; j++)
    {
      Serial.print(manchester[j], HEX);
    }
    Serial.print(":");
    Serial.println(Checksum(5, &manchester[1]),HEX);
    //Serial.println((lfsr_digest8(&manchester[1], 5, 0x98, 0x3e) ^ 0x64), HEX);
   //Serial.println();
*/
    // Gets humidity data from byte 5

    // Checks sensor is a F007th with a valid humidity reading equal or less than 100
    if (dataarray[1] == 0x45 && (Checksum(5, &dataarray[1]) == dataarray[6]))
    {
      // Gets raw temperature from bytes 3 and 4 (note this is neither C or F but a value from the sensor)
      stnId = (dataarray[3] & B01110000) >> 4;
      Newtemp = ((dataarray[3] & B00000111) << 8) + dataarray[4];
      Newhum = dataarray[5];

      saveReading(stnId, Newtemp, Newhum);
    }
  }
}

void eraseManchester()
{
  for (byte j = 0; j < maxBytes; j++)
  {
    dataarray[j] = j;
  }
}

void RFinit(byte rxpin)
{
  rxstate = 0;
  //ToDo wrong place
  Wire.begin(4, 15); // remapping of SPI for OLED
  //Wire.setClock(700000);

  OLED.begin();
  OLED.setFont(u8x8_font_chroma48medium8_r);
  OLED.drawString(0, 0, "F0007TH");

  eraseManchester(); // clear the array to different nos cause if all zeroes it might think that is a valid 3 packets ie all equal
  chTemp[0] = chTemp[1] = chTemp[2] = chTemp[3] = chTemp[4] = chTemp[5] = chTemp[6] = chTemp[7] = 720;
  chHum[0] = chHum[1] = chHum[2] = chHum[3] = chHum[4] = chHum[5] = chHum[6] = chHum[7] = 0;
  chLastRecv[0] = chLastRecv[1] = chLastRecv[2] = chLastRecv[3] = chLastRecv[4] = chLastRecv[5] = chLastRecv[6] = chLastRecv[7] = 1;

  pinMode(rxpin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(rxpin), RF_ISR, CHANGE);
}
void check_RF_state(byte rxpin)
{

  //ToDo init values depends on the state: new packet
  //state 0 = init = new packet
  //state 1 = wait for first change
  //state 2 = sDelay
  //state 3 = lDelay
  //state 4 = processBit

  //digitalWrite(7,LOW); //is for Oszi trigger

  switch (rxstate)
  {

  case 0:               // new packet
    tempBit = polarity; // these begin the same for a packet
    firstZero = false;
    headerHits = 0;
    nosBits = 6; // starts with 2 Bits???
    nosBytes = 0;
    rxstate = 1; //next state
    EdgeTime = 0;
    attachInterrupt(digitalPinToInterrupt(rxpin), RF_ISR, CHANGE);
    //break;
  case 1: //waiting for edge
    if ((digitalRead(rxpin) == tempBit) && EdgeTime > 0)
    {
      detachInterrupt(digitalPinToInterrupt(rxpin));
      Dstop = EdgeTime + sDelay;
      //Dstop = micros() + sDelay;
      //isrcalled = false; //double check this state handling race condition, noise, does it fit for the timing in all cases?
      rxstate = 2;
      //Serial.println("1");
    }
    //else
    //break;
  case 2: // sDelay
    if (micros() > Dstop)
    { // delay passed
      // 3/4 the way through, if RxPin has changed it is definitely an error
      if (digitalRead(rxpin) != tempBit)
      {
        rxstate = 0;
        break;         // something has gone wrong, polarity has changed too early, ie always an error
      }                // exit and retry
      Dstop += lDelay; //next stop for state 3
      rxstate = 3;
      //Serial.println("2");
    };

    break;

  case 3:
    if (micros() > Dstop)
    {
      //delay passed
      // now 1 quarter into the next bit pattern,
      rxstate = 4;
      //Serial.println("3");
    }
    else
      break; //we have to wait in state 3
  case 4:
    rxstate = 1; // might change later to 0, is just the default
        EdgeTime = 0; //double check this state handling race condition, noise, does it fit for the timing in all cases?
    attachInterrupt(digitalPinToInterrupt(rxpin), RF_ISR, CHANGE);

    if (digitalRead(rxpin) == tempBit)
    { // if RxPin has not swapped, then bitWaveform is swapping
      // If the header is done, then it means data change is occuring ie 1->0, or 0->1
      // data transition detection must swap, so it loops for the opposite transitions
      tempBit = tempBit ^ 1;
    } // end of detecting no transition at end of bit waveform, ie end of previous bit waveform same as start of next bitwaveform

    //****************************//
    // Now process the tempBit state and make data definite 0 or 1's, allow possibility of Pos or Neg Polarity
    byte bitState = tempBit ^ polarity; // if polarity=1, invert the tempBit or if polarity=0, leave it alone.

    if (bitState == 1)
    { // 1 data could be header or packet
      if (!firstZero)
      {
        headerHits++;
        //Serial.print("H");
      }
      else
      {
        add(bitState); // already seen first zero so add bit in
        Serial.print("A");
      }
    } // end of dealing with ones
    else
    { // bitState==0 could first error, first zero or packet
      // if it is header there must be no "zeroes" or errors
      if (headerHits < headerBits)
      {
        // Still in header checking phase, more header hits required
        // landing here means header is corrupted, so it is probably an error
        rxstate = 0;
         Serial.print("B");
        break;
      } // end of detecting a "zero" inside a header
      else
      {
        firstZero = true;
        add(bitState);
        Serial.print("Z");
      } // end of dealing with a first zero
    }   // end of dealing with zero's (in header, first or later zeroes)
    //next bit
    /*
    if (rxstate == 1)
    {               //if it has not change to 0 by add bit
      attachInterrupt(digitalPinToInterrupt(rxpin), RF_ISR, CHANGE);
      EdgeTime = 0; //double check this state handling race condition, noise, does it fit for the timing in all cases?
    } */

  } // case statement we might disable interrupt, as it is ignored anyhow
}