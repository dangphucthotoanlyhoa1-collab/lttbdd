#ifndef WIFI_AP_H
#define WIFI_AP_H
#include "config.h"

void handleSave();
// Hàm kiểm tra và khởi động chế độ AP nếu chưa có WiFi
void setupWifiAP();
// Vòng lặp xử lý WebServer (chỉ chạy khi ở chế độ AP)
void loopWifiAP();
// Kiểm tra xem đã kết nối WiFi thành công chưa
bool isWifiConnected();

#endif