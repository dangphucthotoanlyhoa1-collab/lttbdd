#include "WifiAP.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>

WebServer serverAP(80); 
Preferences prefs;
JsonDocument jsonDocAP;
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
    prefs.begin("wifi-conf", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    serverAP.send(200, "application/json", "{\"status\":\"ok\",\"msg\":\"Saved. Rebooting...\"}");
    delay(1000);
    ESP.restart();
  }
}
// Hàm hiển thị giao diện nhập WiFi
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Cai Dat WiFi Robot</title>";
  html += "<style>body{font-family:sans-serif;text-align:center;margin-top:50px;} input{padding:10px;width:80%;margin:10px 0;} button{padding:10px 20px;background:#28a745;color:white;border:none;}</style></head>";
  html += "<body><h1>Cau Hinh WiFi Robot</h1>";
  html += "<form action='/save-wifi' method='POST' enctype='text/plain'>"; // Gửi thẳng text raw
  html += "<input type='text' id='ssid' placeholder='Ten WiFi (SSID)'><br>";
  html += "<input type='text' id='password' placeholder='Mat khau'><br>";
  
  // Javascript nhỏ để gom dữ liệu thành JSON trước khi gửi cho khớp với code của bạn
  html += "<button type='button' onclick='submitForm()'>Luu & Khoi Dong</button>";
  html += "</form>";
  
  html += "<script>";
  html += "function submitForm() {";
  html += "  var ssid = document.getElementById('ssid').value;";
  html += "  var pass = document.getElementById('password').value;";
  html += "  var data = JSON.stringify({ssid: ssid, password: pass});"; // Đóng gói JSON
  html += "  var xhr = new XMLHttpRequest();";
  html += "  xhr.open('POST', '/save-wifi', true);";
  html += "  xhr.setRequestHeader('Content-Type', 'text/plain');";
  html += "  xhr.send(data);";
  html += "  xhr.onload = function() { alert(this.responseText); };";
  html += "}";
  html += "</script>";
  
  html += "</body></html>";
  
  serverAP.send(200, "text/html", html);
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
    WiFi.softAP("ROBOT_SETUP_WIFI", NULL);
    serverAP.on("/", HTTP_GET, handleRoot);
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
void resetWifiConfig() {
  Serial.println("Dang xoa WiFi va Reset...");
  
  // 1. Xóa trong bộ nhớ Preferences
  prefs.begin("wifi-conf", false);
  prefs.clear(); 
  prefs.end();
  
  // 2. Xóa trong bộ nhớ WiFi của ESP32
  WiFi.disconnect(true, true); 
  
  delay(500);
  ESP.restart(); // Tự động khởi động lại
}