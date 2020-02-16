#include <Arduino.h>
#include "F007TH.h"
#include "send2GS.h"


#define RxPin 39

unsigned long nextsend;
const unsigned int period = 300000;

const char* NODEID = "F007TH";

char sendstr[100];

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Heltec F0007TH");

// seems to be important to start that first 
// ISR seems to crash WLAN/IP stack
// ToDo disable ISR during reconnect
  WiFiinit();

  RFinit(RxPin);
 
  //send2GS("nodeid=F007TH&values=16.7;67;21.15;66;0;0;13.4;74;16.5;51");
  nextsend = millis(); //update asap

}

void loop()
{
  check_RF_state(RxPin);


  if (millis() > nextsend ) {

    snprintf(sendstr, 100, "nodeid=%s&values=%s;%d;%s;%d;%s;%d;%s;%d", NODEID, temp2str(0), chHum[0], temp2str(1), chHum[1],temp2str(2), chHum[2],temp2str(3), chHum[3]);
    Serial.println(sendstr);
    //send2GS("nodeid=F007TH&values=16.7;67;21.15;66;0;0;13.4;74;16.5;51");
    send2GS(sendstr);
    nextsend += period;
  }
  
} // end of mainloop


