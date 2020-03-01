#include <Arduino.h>
//#include <heltec.h>
#include "F007TH.h"
#include "send2GS.h"
#include <U8x8lib.h>

#define RxPin 13

unsigned long nextsend;
const unsigned int period = 120000;

const char *NODEID = "F007TH";

char sendstr[100];




void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Heltec F0007TH");

  Serial.println("before wire");
  Wire.begin(4,15); // remapping of SPI for OLED
  Serial.println("after wire");
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
}

void loop()
{
  check_RF_state(RxPin);

  if (millis() > nextsend)
  {
    snprintf(sendstr, 100, "nodeid=%s&values=%s;%d;%lu;%s;%d;%lu;%s;%d;%lu;%s;%d;%lu", NODEID, temp2str(0), chHum[0], chLastRecv[0],
             temp2str(1), chHum[1], chLastRecv[1],
             temp2str(2), chHum[2], chLastRecv[2],
             temp2str(3), chHum[3], chLastRecv[3]);
    Serial.println(sendstr);
    send2google(sendstr);
    nextsend += period;
  }

} // end of mainloop
