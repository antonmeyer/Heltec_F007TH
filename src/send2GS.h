//#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "credentials.h"

#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>


//#define debug false


//WiFiClientSecure client;

const String URL = "https://script.google.com/macros/s/" + key + "/exec?"; // Server URL
//const String URI = "/macros/s/"+ key + "/exec?";

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

  //WiFi.setTxPower(WIFI_POWER_7dBm);
esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

  
}

boolean send2google(String datastr)
{
  HTTPClient httpsClient;
  httpsClient.setReuse(false);
  boolean result;
  if (WiFi.status() != WL_CONNECTED){
    Serial.println("Wifi needs erconnect");
    WiFiinit();
  }
   
  Serial.print("[HTTPS] begin...\n");
  if (httpsClient.begin(URL + datastr))
  //if (httpsClient.begin("script.google.com", 443, URI +datastr))
  { // HTTPS
    Serial.print("[HTTPS] GET...");
    // start connection and send HTTP header
    int httpCode = httpsClient.GET();
    // httpCode will be negative on error
    if (httpCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        //String payload = httpsClient.getString();
        // Serial.println(payload);
        result = true;
      };
      Serial.println(httpCode);
    }
    else
    {
      Serial.printf("[HTTPS] GET... failed, error: %s\n", httpsClient.errorToString(httpCode).c_str());
      result = false;
    };
    httpsClient.end(); //keep it open??? works for 2min but not for 5 min
  }
  else
  {
    Serial.printf("[HTTPS] Unable to connect\n");
    result = false;
  };
  return result;
}
