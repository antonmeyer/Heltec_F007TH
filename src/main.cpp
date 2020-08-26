#include <Arduino.h>
#include <Wire.h>
//#include <heltec.h>
//#include "F007TH.h"
#include "F007TH_RMT.h"
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
F007TH_RMT *f007th = new F007TH_RMT(RxPin, 7); // max 7 sensors

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

#include "mbusmeters.h"
wMBusMsg wmbMsg1;

unsigned long nextsend;
const unsigned long period = 300000; //period to send to google spreadsheet

unsigned int nextdraw = 0;

const char *NODEID = "F007TH"; // names the spreadsheet
const char *NODEID2 = "WMZL14";

#define tWMZ 0x43
#define tWWZ 0x62
#define techemvendor 0x6850

const uint32_t wmzL14[] = {0x30585388, 0x30586050, 0x30586062, 0x30586064, 0x30586121, 0x30586125};
const uint32_t wwzL14[] = {0x25063431, 0x25071898, 25071981, 0x25153680, 0x25153687, 0x25153718};
#define SIZEA(a) (sizeof a / sizeof *a)

mbMeterGrp wmzL14g(techemvendor, tWMZ, (uint32_t *)wmzL14, SIZEA(wmzL14));
mbMeterGrp wwzL14g(techemvendor, tWWZ, (uint32_t *)wwzL14, SIZEA(wwzL14));
#define MYWWZ 0x25153718

const unsigned long wmz_cycle = 24 * 60 * 60 * 1000; //  1 day; the tricky part here ...we do not know which time it is, but next value come at midnight
unsigned long next_wmz_run;
const unsigned long wmz_wait = 15 * 60 * 1000; // we wait 15 min to catch all values

char sendstr[100];

#include "prgbtn.h"

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

  // seems to be important to start that first
  // ISR seems to crash WLAN/IP stack
  // ToDo disable ISR during reconnect
  WiFiinit();
  //RFinit(RxPin);
  
  nextsend = millis() + period; //update asap
  Serial.println("RFinit");
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

  button_init();

  f007th->startRx();

} //setup

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
  //Serial.println(grpwmzL14.vendor);
  //byte idx = check_RF_state(RxPin);
  f007th->rxHandler2();

  //delay(100);
  //byte idx =0;
  /*
  if ((idx > 0) && !OLEDoff)
  {
    // ToDo decouple it object driffen approach
    OLED.clearLine(idx);
    snprintf(displaystr, 17, "%d:%sC %2d%% %4d", idx, tempstr[idx - 1], chHum[idx - 1], diff / 1000);
    OLED.drawString(0, idx, displaystr);
  } */

  if (millis() > next_wmz_run)
  //if (0)
  {                                                    // we run only once per period
    if (sx12xxmbus.receiveSizedFrame(FixPktSize, 200)) //minRSSI to reduce noice load
    {
      byte RSSI = sx12xxmbus.getLastRSSI();
      if ((RSSI < 250) && wmbMsg1.parseraw(sx12xxmbus._RxBuffer, sx12xxmbus._RxBufferLen))
      {
        //if (decode3o6Block(sx12xxmbus._RxBuffer, mBusMsg, sx12xxmbus._RxBufferLen) != DecErr) {
        wmbMsg1.printmsg();
        if (!wwzL14g.matchgrp(wmbMsg1)) //we assume 1 msg can belong only to 1 group
          wmzL14g.matchgrp(wmbMsg1);

        /*
            if (0x30585388 == heatmeterserial)
            {
              char str[12];
              sprintf(str, "%li", wmzValue[0]);
              OLED.drawString(12, 0, str);
            } */

      } //RSSI
    }   // packet received

    if ((wmzL14g.complete() && wwzL14g.complete()) || (millis() > (next_wmz_run + wmz_wait)))
    {                            //timeout or we got all
      sx12xxmbus.setModeSleep(); //sx12xx will sleep until next period
      next_wmz_run += wmz_cycle; // add 1 period before next wmz capture run NODEID2
      f007th->stopRx();          // we want avoid buffer overrrun

      wmzL14g.fillsendstr(NODEID2, sendstr, 100);
      Serial.println(sendstr);
      send2google(sendstr); // resend ToDo
      //init for the next run
      wmzL14g.clearvals();
      wwzL14g.fillsendstr("WWL14", sendstr, 100);
      Serial.println(sendstr);
      send2google(sendstr);
      wwzL14g.clearvals();

      f007th->startRx(); // we want avoid buffer overrrun
    }
  } //next wmzrun

  checkcmd();

  if ((millis() > nextsend)) //&& (rxstate == 0))
  //rxstate 0 is in the beginning, waitung for the first edge, ISR disabled
  {
    if ((f007th->stopRx()) != ESP_OK)
      Serial.println("rmt stop failed"); // we want to avoid buffer overrun of the RMT

    f007th->fillsendstr(NODEID, sendstr, 99);
    Serial.println(sendstr);
    
    if (send2google(sendstr))
    //if (1)
    {
      nextsend += period; //send ok, next period
    }
    else
    {
      nextsend += 30000; // send nok retry 30s later
    }

    //Serial.println(ESP.getMinFreeHeap());
    Serial.println("rmt start ");
    if ((f007th->startRx()) != ESP_OK) //we want to avoid buffer overrun of the RMT
    Serial.println("rmt start failed");
  }
  
        
  if ((millis() > nextdraw)) //&& (rxstate == 0))
  {                          //update the timer on the OLED

    if (!OLEDoff)
    {
      unsigned long now1 = millis() / 1000UL;
      snprintf(displaystr, 17, "%2luT%2lu:%02lu:%02lu", elapsedDays(now1), numberOfHours(now1), numberOfMinutes(now1), numberOfSeconds(now1));
      OLED.drawString(0, 0, displaystr);
    }
    nextdraw += 3000;
    OLED.setPowerSave(OLEDoff); //ToDo encapsulate, wenn off no write to OLED, and call only when changed
  }

} // end of mainloop
