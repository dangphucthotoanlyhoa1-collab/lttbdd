
#include "WifiAP.h"
#include "FirebaseLogic.h"
#include "config.h"
unsigned long buttonTimer = 0;
bool buttonActive = false;
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
float Size_of_objects[100]; 
int Counter_Size_of_objects = 0;
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
  pinMode(0, INPUT_PULLUP);
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
  if (digitalRead(0) == LOW) {
    // Nếu mới bắt đầu nhấn
    if (!buttonActive) {
      buttonActive = true;
      buttonTimer = millis(); // Bắt đầu tính giờ
      Serial.println(">> Dang giu nut... (Giu 3s de Reset WiFi)");
    }
    
    // Nếu đang giữ nút, kiểm tra xem đã đủ 3 giây chưa
    if ((millis() - buttonTimer > 3000)) {
       Serial.println(">> DA DU 3 GIAY! TIEN HANH RESET...");
       resetWifiConfig(); // Gọi hàm reset bên WifiAP.cpp
       buttonActive = false; // Reset trạng thái
    }
  } else {
    // Nếu nhả nút ra
    if (buttonActive) {
      buttonActive = false;
      Serial.println(">> Da nha nut (Huy Reset).");
    }
  }
}