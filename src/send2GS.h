//#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "credentials.h"
#define debug true

const String URL = "https://script.google.com/macros/s/" + key + "/exec?"; // Server URL

HTTPClient httpsClient;

void WiFiinit()
{
  Serial.println("Warte auf Verbindung");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP Addresse: ");
  Serial.println(WiFi.localIP());

  httpsClient.setReuse(true);
}

void send2google(String datastr)
{
  Serial.print("[HTTPS] begin...\n");
  if (httpsClient.begin(URL  + datastr))
  { // HTTPS
    Serial.print("[HTTPS] GET...");
    // start connection and send HTTP header
    int httpCode = httpsClient.GET();
    // httpCode will be negative on error
    if (httpCode > 0)
    { /*
          // HTTP header has been send and Server response header has been handled
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
          {
            String payload = https.getString();
            Serial.println(payload);
          } */
          Serial.println(httpCode);
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", httpsClient.errorToString(httpCode).c_str());
    }
    //httpsClient.end();
  }
  else
  {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
}
