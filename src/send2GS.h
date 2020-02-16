#include <WiFiClientSecure.h>
#include "credentials.h"
#define debug true

WiFiClientSecure wClient;

const char* server = "script.google.com";  // Server URL

void WiFiinit() {
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

boolean send2GS(String datastr)
{
  String movedURL;
  String line;

  if (debug)Serial.println("Verbinde zum script.google.com");
  if (!wClient.connect(server, 443))
  {
    if (debug) Serial.println("Verbindung fehlgeschlagen!");
    return false;
  }

  if (debug) Serial.println("Verbunden!");
  // ESP32 Erzeugt HTTPS Anfrage an Google sheets
  String url = "https://" + String(server) + "/macros/s/" + key + "/exec?" + datastr;
  if (debug) Serial.println(url);
  wClient.println("GET " + url);
  wClient.println("Host: script.google.com" );
  wClient.println("Connection: close");
  wClient.println();


  // ESP32 empfängt antwort vom Google sheets
  while (wClient.connected())     // ESP32  empfängt Header
  {
    line = wClient.readStringUntil('\n');
    if (debug) Serial.println(line);
    if (line == "\r") break;      // Ende Des Headers empfangen
    if (line.indexOf ( "Location" ) >= 0)   // Weiterleitung im Header?
    { // Neue URL merken
      movedURL = line.substring ( line.indexOf ( ":" ) + 2 ) ;
    }
  }
wClient.stop();
return true;
  /*
  while (wClient.connected())    // Google Antwort HTML Zeilenweise Lesen
  {
    if (wClient.available())
    {
      line = wClient.readStringUntil('\r');
      if (debug) Serial.print(line);
    }
  }
  wClient.stop();



  movedURL.trim(); // leerzeichen, \n entfernen
  if (debug) Serial.println("Weiterleitungs URL: \"" + movedURL + "\"");

  if (movedURL.length() < 10) return false; // Weiterleitung nicht da

  if (debug) Serial.println("\n Starte Weiterleitung...");
  if (!wClient.connect(server, 443))
  {
    if (debug) Serial.println("Weiterleitung fehlgeschlagen!");
    return false;
  }

  Serial.println("Verbunden!");
  // // ESP32 Erzeugt HTTPS Anfrage an Google Tabellen
  wClient.println("GET " + movedURL);
  wClient.println("Host: script.google.com");
  wClient.println("Connection: close");
  wClient.println();

  while (wClient.connected()) // ESP32  empfängt Header
  {
    line = wClient.readStringUntil('\n');
    if (debug) Serial.println(line);
    if (line == "\r")break;
  }
  
  while (wClient.connected()) // Google Antwort HTML Zeilenweise Lesen
  {
    if (wClient.available())
    {
      line = wClient.readStringUntil('\r');
      if (debug) Serial.print(line);
    }
  }
  wClient.stop();
  if (line == "Ok") {
      return true;
  } else return false;

  */

}
