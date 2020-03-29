#include <Arduino.h>
//#include <heltec.h>
#include "F007TH.h"
#include "send2GS.h"
//#include <U8x8lib.h>

#include "zeit.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "esp32-hal-cpu.h"

#include "RFM69mbus.h"

#define RxPin 13
//#define RxPin 19

#define PinNSS 5
//#define PinNSS 2
#define PinDIO0 34
//instance RFM69
RFM69 rfm69;
int8_t PAind = 13;

#include "msgdecoder.h"

unsigned long nextsend;
const unsigned long period = 300000;

unsigned int nextdraw = 0;

const char *NODEID = "F007TH";

char sendstr[100];

void setup()
{
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  //setCpuFrequencyMhz(80);

  //Heltec.begin(false /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);

  //Heltec.display->display();

  //esp_log_level_set("wifi", ESP_LOG_INFO);
  Serial.begin(115200);
  Serial.println(getCpuFrequencyMhz());
  Serial.println(NODEID);
  Serial.println(ESP.getFreeHeap());

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

  //RFinit(RxPin);
  nextsend = millis(); //update asap

  if (!rfm69.initDevice(PinNSS, PinDIO0, CW, 868.95, GFSK, 100000, 40000, 5, PAind))
  {
    Serial.println("error initializing RFM69");
  }
  else
  {
    Serial.println("RFM69 ready");
  };
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
      regval = strtoul(cmdstr.substring(4, 6).c_str(), &ptr, 16);
      rfm69.writeSPI(reg, regval);
      Serial.print("set ");
      Serial.print(reg, HEX);
      Serial.print(" to ");
    }
    if (cmd == 'r')
    {
      regval = rfm69.readSPI(reg);
      Serial.print(reg, HEX);
      Serial.print(" was set to: ");
    }

    Serial.println(regval, HEX);
  }
}

void loop()
{
  //check_RF_state(RxPin);

  if (rfm69.receiveSizedFrame(FixPktSize))
  {
    printmsg();
  }

  checkcmd();
  /*
  if ((millis() > nextsend) && (rxstate == 0))
  //rxstate 0 is in the beginning, waitung for the first edge, ISR disabled
  {
    snprintf(sendstr, 99, "nodeid=%s&values=%s;%d;%lu;%s;%d;%lu;%s;%d;%lu;%s;%d;%lu", NODEID,
             temp2str(0), chHum[0], chLastRecv[0],
             temp2str(1), chHum[1], chLastRecv[1],
             temp2str(2), chHum[2], chLastRecv[2],
             temp2str(3), chHum[3], chLastRecv[3]);
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
  */
  /*
  if ((millis() > nextdraw) ) //&& (rxstate == 0))
  {
    unsigned long now1 = millis() / 1000UL;
    snprintf(displaystr, 17, "%2luT %2lu:%02lu:%02lu", elapsedDays(now1), numberOfHours(now1), numberOfMinutes(now1), numberOfSeconds(now1));
    OLED.drawString(0, 7, displaystr);
    nextdraw += 3000;
  }
*/
} // end of mainloop
