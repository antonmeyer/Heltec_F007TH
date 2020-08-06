#include <Arduino.h>
//#include <heltec.h>
#include "F007TH.h"
#include "send2GS.h"
#include <U8x8lib.h>

#include "zeit.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp32-hal-cpu.h"

//#include "RFM69mbus.h"
#include "sx1276mbus.h"
SX1276MBUS sx12xxmbus;

#define RxPin 38 //ESP32 input only
//#define RxPin 19

#define PinNSS 18 //for sx1276 on Heltec v2
//#define PinNSS 2
#define PinDIO0 26 //for sx1276 on Heltec v2

/* heltec esp32 lora v2
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// so we use for the RFM69 same MISO MOSI SCK but different CS PinNSS
*/
//instance RFM69
//RFM69 rfm69;

int8_t PAind = 13;

U8X8_SSD1306_128X64_NONAME_HW_I2C OLED(/*OLED_RST*/ 16, /*OLED_SCL*/ 15, /*OLED_SDA*/ 4);
//U8X8_SSD1306_128X64_NONAME_SW_I2C OLED(/* clock=*/15, /* data=*/4, /* reset=*/16);
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 (U8G2_R0, 16, 15, 4)

// 1.3" OLED
//U8X8_SH1106_128X64_NONAME_HW_I2C OLED(/* reset=*/U8X8_PIN_NONE);
char displaystr[20];

#include "msgdecoder.h"

unsigned long nextsend;
const unsigned long period = 300000; //period to send to google spreadsheet

unsigned int nextdraw = 0;

const char *NODEID = "F007TH"; // names the spreadsheet
const char *NODEID2 = "WMZL14";

const uint16_t techemtype = 0x4322;
const uint32_t wmzL14[] = {0x30585388, 0x30586050, 0x30586062, 0x30586064, 0x30586121, 0x30586125};
long wmzValue[6];
char wmzcnt = 6;
char wmzvals = 0;
const unsigned long wmz_cycle = 24 * 60 * 60 * 1000; //  1 day; the tricky part here ...we do not know which time it is, but next value come at midnight
unsigned long next_wmz_run;
const unsigned long wmz_wait = 15 * 60 * 1000; // we wait 15 min to catch all values

char sendstr[100];

void setup()
{
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  //setCpuFrequencyMhz(80);

  //Heltec.begin(false /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);

  //Heltec.display->display();

  //esp_log_level_set("wifi", ESP_LOG_INFO);
  Serial.begin(115200);
  //Serial.println(getCpuFrequencyMhz());
  Serial.println(NODEID);
  //Serial.println(ESP.getFreeHeap());

  //ToDo wrong place
  Wire.begin(4, 15); // remapping of SPI for OLED
  //Wire.setClock(700000);

  OLED.begin();
  OLED.setFont(u8x8_font_chroma48medium8_r);
  OLED.drawString(0, 0, "F0007TH");

  //Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/, false /*PABOOST Enable*/, 868E6 /**/);
  //Heltec.display->flipScreenVertically();
  //Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  //Heltec.display->setFont(ArialMT_Plain_10);

  //Heltec.display->drawString(0, 0, "Hello world");
  //Heltec.display->display();

  // seems to be important to start that first
  // ISR seems to crash WLAN/IP stack
  // ToDo disable ISR during reconnect
  WiFiinit();

  RFinit(RxPin);
  nextsend = millis(); //update asap

  //if (!rfm69.initDevice(PinNSS, PinDIO0, CW, 868.95, GFSK, 100000, 40000, 5, PAind))
  if (!sx12xxmbus.initDevice(PinNSS, PinDIO0)) //minRSSI
  {
    Serial.println("error initializing sx1276");
  }
  else
  {
    Serial.println("sx1276 ready");
  };

  next_wmz_run = millis();

  for (short i = 0; i < wmzcnt; i++)
  {
    wmzValue[i] = -1;
  }
}

void checkcmd()
{
  String cmdstr;
  unsigned char reg, regval;
  char cmd;
  char *ptr;
  if (Serial.available() > 0)
  {
    cmdstr = Serial.readStringUntil('\n');
    //we assume "register, value" = 5 byte
    cmd = cmdstr.charAt(0);
    reg = strtoul(cmdstr.substring(1, 3).c_str(), &ptr, 16);

    if (cmd == 'w')
    {
      sx12xxmbus.setModeStdby();
      regval = strtoul(cmdstr.substring(4, 6).c_str(), &ptr, 16);
      sx12xxmbus.writeSPI(reg, regval);
      Serial.print("set ");
      Serial.print(reg, HEX);
      Serial.print(" to ");
      Serial.println(regval, HEX);
    }
    else if (cmd == 'r')
    {
      regval = sx12xxmbus.readSPI(reg);
      Serial.print(reg, HEX);
      Serial.print(" is: ");
      Serial.println(regval, HEX);
    } 
  }
}

void loop()
{

  byte idx = check_RF_state(RxPin);
  if (idx > 0)
  {
    // ToDo decouple it object driffen approach
    OLED.clearLine(idx);
    snprintf(displaystr, 17, "%d:%sC %2d%% %4d", idx, tempstr[idx - 1], chHum[idx - 1], diff / 1000);
    OLED.drawString(0, idx, displaystr);
  }

  if (millis() > next_wmz_run)
  {                                                    // we run only once per period
    if (sx12xxmbus.receiveSizedFrame(FixPktSize, 200)) //minRSSI to reduce noice load
    {
      byte RSSI = sx12xxmbus.getLastRSSI();
      if (RSSI < 250)
      {
        memset(mBusMsg, 0, sizeof(mBusMsg));
        if (decode3o6Block(sx12xxmbus._RxBuffer, mBusMsg, sx12xxmbus._RxBufferLen) != DecErr) {
        //decode3o6Block(sx1276mbus._RxBuffer, mBusMsg, sx1276mbus._RxBufferLen);
        printmsg(mBusMsg, sx12xxmbus._RxBufferLen, RSSI);
        
          if (techemtype == get_type(mBusMsg))
          {
            //printmsg(mBusMsg, sx1276mbus._RxBufferLen, RSSI);

            uint32_t heatmeterserial = get_serial(mBusMsg);

            //find the entry in the array
            for (short i = 0; i < wmzcnt; i++)
            {
              if ((wmzL14[i] == heatmeterserial) && (wmzValue[i] == -1))
              {
                wmzValue[i] = get_curWMZ(mBusMsg);
                wmzvals++;
              }
            }

            if (0x30585388 == heatmeterserial)
            {
              char str[12];
              sprintf(str, "%li", wmzValue[0]);
              OLED.drawString(12, 0, str);
            }

          } //techemtype c
        }   // decode ok
      }     //RSSI
    }       // packet received

    if ((wmzvals == wmzcnt ) || (millis() > (next_wmz_run + wmz_wait)))
    {                            //timeout or we got all
      next_wmz_run += wmz_cycle; // add 1 period before next wmz capture run
      snprintf(sendstr, 99, "nodeid=%s&values=%li;%li;%li;%li;%li;%li", NODEID2,
               wmzValue[0], wmzValue[1], wmzValue[2],
               wmzValue[3], wmzValue[4], wmzValue[5]);
      Serial.println(sendstr);
      send2google(sendstr); // resend ToDo
      //init for the next run
      wmzvals = 0;
      for (short i = 0; i < wmzcnt; i++)
      {
        wmzValue[i] = -1;
      }
    }
  } //next wmzrun

  checkcmd();

  if ((millis() > nextsend) && (rxstate == 0))
  //rxstate 0 is in the beginning, waitung for the first edge, ISR disabled
  {
    snprintf(sendstr, 99, "nodeid=%s&values=%s;%d;%s;%d;%s;%d;%s;%d;%s;%d;%s;%d;%s;%d;%li", NODEID,
             temp2str(0), chHum[0], temp2str(1), chHum[1], temp2str(2), chHum[2],
             temp2str(3), chHum[3], temp2str(4), chHum[4], temp2str(5), chHum[5],
             temp2str(6), chHum[6],
             wmzValue[0]);
    Serial.println(sendstr);
    if (send2google(sendstr))
    {
      nextsend += period; //send ok, next period
    }
    else
    {
      nextsend += 30000; // send nok retry 30s later
    }

    Serial.println(ESP.getMinFreeHeap());
  }

  if ((millis() > nextdraw)) //&& (rxstate == 0))
  {
    unsigned long now1 = millis() / 1000UL;
    snprintf(displaystr, 17, "%2luT%2lu:%02lu:%02lu", elapsedDays(now1), numberOfHours(now1), numberOfMinutes(now1), numberOfSeconds(now1));
    OLED.drawString(0, 0, displaystr);
    nextdraw += 3000;
  }

} // end of mainloop
