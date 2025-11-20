
#include "WifiAP.h"
#include "FirebaseLogic.h"
#include "config.h"
AccelStepper Axis_Base(AccelStepper::DRIVER, 0, 0);     
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 0, 0); 
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 0, 0);
String task = "";               // Fix lỗi undefined task
toado Coordinates_auto[100];    // Fix lỗi undefined Coordinates
int diem_hien_co = 0;
volatile int jogCommand = 0;
volatile int command_jog = 0; // Khai báo cả 2 tên để tránh lỗi code cũ
volatile int Mode = 0; 
volatile int autoChainMode = 0;
volatile int TotalSteps = 0;
volatile float Object_Size = 0.0;

// 3. Khai báo biến đếm vật phẩm
int Number_of_Objects_small = 0;
int Number_of_Objects_medium = 0;
int Number_of_Objects_large = 0;

// 4. Khai báo bộ nhớ cho mảng tọa độ
XYZT_Coordinates Moving_Coordinates[50];
XYZT_Coordinates Classify[50];
// ================= SETUP & LOOP CHÍNH =================
long readEncoderCount() {
  return millis() / 1000; 
}
void setup() {
  Serial.begin(115200);
  setupWifiAP();
  if (isWifiConnected()) {
    delay(2000);
    setupFirebase();
           Serial.println("connect");
  } else {
    Serial.println("disconect");
  }
}

void loop() {
  loopWifiAP();
  if (isWifiConnected()) {
    
     loopFirebase();
  }
}