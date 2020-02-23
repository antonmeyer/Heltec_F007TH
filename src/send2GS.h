//#include <WiFiMulti.h>
#include <HTTPClient.h>
#include "credentials.h"
#define debug true

WiFiClientSecure wClient;

const char *server = "script.google.com"; // Server URL

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
}

void send2google(String datastr)
{

  HTTPClient https;
  Serial.print("[HTTPS] begin...\n");
  if (https.begin(/* *client, */ "https://" + String(server) + "/macros/s/" + key + "/exec?" + datastr))
  { // HTTPS
    Serial.print("[HTTPS] GET...");
    // start connection and send HTTP header
    int httpCode = https.GET();
    // httpCode will be negative on error
    if (httpCode > 0)
    { /*
          // HTTP header has been send and Server response header has been handled
          // file found at server
          if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
          {
            String payload = https.getString();
            Serial.println(payload);
          } */
          Serial.println(httpCode);
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  }
  else
  {
    Serial.printf("[HTTPS] Unable to connect\n");
  }
  // End extra scoping block
}
