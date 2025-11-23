#ifndef WIFI_AP_H
#define WIFI_AP_H
#include "config.h"

void handleSave();
void setupWifiAP();
void loopWifiAP();
bool isWifiConnected();
void resetWifiConfig();
#endif