#include "WifiAP.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

WebServer serverAP(80); 
Preferences prefs;
StaticJsonDocument<512> jsonDocAP;
bool _isAPMode = false;
bool _isWifiConnected = false;

void handleSave() {
  if (!serverAP.hasArg("plain")) {
    serverAP.send(400, "application/json", "{\"status\":\"error\"}");
    return;
  }
  String body = serverAP.arg("plain");
  deserializeJson(jsonDocAP, body);
  String ssid = jsonDocAP["ssid"];
  String pass = jsonDocAP["password"];

  if (ssid.length() > 0) {
    prefs.begin("cau hinh wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    serverAP.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Saved. Rebooting...\"}");
    delay(1000);
    ESP.restart();
  }
}

void setupWifiAP() {
  prefs.begin("wifi-conf", true); 
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();

  if (ssid == "") {
    _isAPMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP_ADDRESS, AP_IP_ADDRESS, IPAddress(255, 255, 255, 0));
    WiFi.softAP(accespoint esp32, NULL);
    
    serverAP.on("/save-wifi", HTTP_POST, handleSave);
    serverAP.begin();
    Serial.println("ip 192.168.4.1");
  } else {
    _isAPMode = false;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.print("Connecting: "); Serial.println(ssid);

    int wait = 0;
    while (WiFi.status() != WL_CONNECTED && wait < 20) {
      delay(500); Serial.print("."); wait++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      _isWifiConnected = true;
      Serial.println("\n Connected! IP: " + WiFi.localIP().toString());
    } else {
      Serial.println("\nFailed back to AP");
      prefs.begin("wifi-conf", false);
      prefs.clear(); prefs.end();
      ESP.restart();
    }
  }
}

void loopWifiAP() {
  if (_isAPMode) serverAP.handleClient();
}

bool isWifiConnected() { return _isWifiConnected; }