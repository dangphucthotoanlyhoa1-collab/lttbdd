#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include "driver/pcnt.h"


//========== Khai báo cấu trúc tọa độ XYZT và Tốc độ chuyển động ======
typedef struct
{
  float X;
  float Y;
  float Z;
  int T;
  float Speed_X;
  float Speed_Y;
  float Speed_Z;
  int Speed_T;

} XYZT_Coordinates;

XYZT_Coordinates Moving_Coordinates[1000];

//========== Khai báo các đối tượng stepper motor ======
AccelStepper Axis_Base(AccelStepper::DRIVER, 26, 27); // Step, Dir
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 14, 12); // Step, Dir
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 25, 33); // Step, Dir
#define Axis_Gripper_forward 3 // Chân điều khiển gripper (nếu có)
#define Axis_Gripper_backward 4 // Chân điều khiển gripper (nếu có)
int Direction_Gripper = 0; // Hướng kẹp vật: 0= đóng càng, 1=mở càng
int speedGripperForward = 0; // Tốc độ gripper mở
int speedGripperBackward = 0; // Tốc độ gripper đóng

//========== Khai báo các biến toàn cục ======

// ====== Chân công tắc hành trình ======
const int LIM_BASE_PIN     = 15; // chỉnh theo phần cứng
const int LIM_SHOULDER_PIN = 13; // chỉnh theo phần cứng
const int LIM_ELBOW_PIN    = 23; // chỉnh theo phần cứng

// ====== Các biến điều khiển robot (State Machine) ======
volatile int robotMode = 0; // 0=AUTO (dùng run()), 1=JOG (dùng runSpeed())
volatile int jogCommand = 0; // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0; // Vận tốc cố định 600

//====== Các biến cho chạy chuỗi tự động
volatile int autoChainMode = 0; // 0=STOPPED, 1=RUNNING
volatile int NumberOfStepRunAuto = 0; // só bước chạy auto
volatile int autoChainStep = 0; // Bước hiện tại
int maxSteps = 1000; // Số bước tối đa

//================================================================================//
//================================================================================//
//===================== các tác vụ điều khiển robot ==============================//

// Hàm homing cho 1 trục (blocking)
void homeAxis(AccelStepper &axis, int limPin, const char *name) {

  const float fastSpeed = -800; // bước/s (hướng về công tắc) -> chỉnh theo cấu hình
  const float slowSpeed = -400; // bước/s cho approach chính xác
  const float retreatSpeed = 400.0; // Tốc độ lùi ra (hướng dương)
  const int retreatSteps = 400; // số bước rút lui sau lần chạm đầu -> chỉnh theo cơ chế

  // Approach nhanh tới công tắc
  axis.setSpeed(fastSpeed);
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
  }
  // dừng và rút lui một đoạn
  axis.setSpeed(0);
  axis.runSpeed();
  delay(20);

  axis.setCurrentPosition(0); // Tạm đặt 0 ở đây
  axis.setSpeed(retreatSpeed);
  while (axis.currentPosition() < retreatSteps) {
    axis.runSpeed();
  }
  axis.setSpeed(0);
  axis.runSpeed();
  delay(20);

  // Approach chậm lại cho vị trí chính xác
  axis.setSpeed(slowSpeed);
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
  }
  // dừng và đặt vị trí hiện tại làm home (0)
  axis.setSpeed(0); //dừng trục
  axis.runSpeed();

  axis.setCurrentPosition(0); //cài đặt home
  Serial.print(name); // in ra trạng thái trục đã home
  Serial.println(" homed!");
  delay(50);
}

// Hàm homing cho cả 3 trục (blocking)
void SetHomeAll() {
  Serial.println("Starting homing sequence...");
  homeAxis(Axis_Base, LIM_BASE_PIN, "Axis Base");
  homeAxis(Axis_Shoulder, LIM_SHOULDER_PIN, "Axis Shoulder");
  homeAxis(Axis_Elbow, LIM_ELBOW_PIN, "Axis Elbow");
  Serial.println("Homing completed!");
}

//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void ReturnToHome() {
  // 1. Đảm bảo robot ở chế độ AUTO
  robotMode = 0;
  jogCommand = 0;
  autoChainMode = 0; // Dừng chạy chuỗi nếu đang chạy
  
  // 2. Chỉ cần "đặt mục tiêu"
  Axis_Base.moveTo(0);
  Axis_Shoulder.moveTo(0);
  Axis_Elbow.moveTo(0);
  
  Serial.println("Da ra lenh di chuyen ve Home...");
}
//========Thuật toán phát hiện kẹp vật của gripper ========


// ====== Move Jog Task (Chế độ JOG) ======
void MoveJog(void *parameter) {
  while(1) { 

    if (robotMode == 1) { // Chỉ chạy khi ở chế độ JOG
    
      // Đặt tất cả về 0 trước
      Axis_Base.setSpeed(0);
      Axis_Shoulder.setSpeed(0);
      Axis_Elbow.setSpeed(0);

      // Đặt tốc độ cho trục được yêu cầu
      switch (jogCommand) {
        case 1: Axis_Base.setSpeed(JOG_SPEED); break;
        case 2: Axis_Base.setSpeed(-JOG_SPEED); break;
        case 3: Axis_Shoulder.setSpeed(JOG_SPEED); break;
        case 4: Axis_Shoulder.setSpeed(-JOG_SPEED); break;
        case 5: Axis_Elbow.setSpeed(JOG_SPEED); break;
        case 6: Axis_Elbow.setSpeed(-JOG_SPEED); break;
      }
      
      // Gọi runSpeed() cho TẤT CẢ (chỉ trục nào != 0 mới chạy)
      Axis_Base.runSpeed();
      Axis_Shoulder.runSpeed();
      Axis_Elbow.runSpeed();
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS); //cập nhật lệnh mỗi 1ms
  }
}

//====== Task chạy chế độ AUTO (Chạy chuỗi và Non-Blocking) ======
// Task này xử lý mọi lệnh 'run()'
void motor_task_AUTO(void *parameter) {

  while(1) { // Vòng lặp task vô tận
  
    if (robotMode == 0) { // Chỉ chạy khi ở chế độ AUTO
      
      // 1. Logic chạy chuỗi (Auto Chain)
      if (autoChainMode == 1) {
        
        // Kiểm tra xem 3 trục đã dừng lại (hoàn thành bước) chưa
        if (Axis_Base.distanceToGo() == 0 && 
            Axis_Shoulder.distanceToGo() == 0 && 
            Axis_Elbow.distanceToGo() == 0) {
              
          // Đã hoàn thành bước autoChainStep
          Serial.print("Hoan thanh buoc: "); Serial.println(autoChainStep);
          autoChainStep++; // Chuyển sang bước tiếp theo
          
          if (autoChainStep >= maxSteps) {
            
            autoChainStep = 0; // Dừng lại

            }
          } else {
            // Còn bước: Ra lệnh cho bước tiếp theo
            Serial.print("Bat dau buoc: "); Serial.println(autoChainStep);
            
            // Đặt tốc độ MỚI (quan trọng)
            Axis_Base.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_X);
            Axis_Shoulder.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Y);
            Axis_Elbow.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Z);
            
            // Đặt mục tiêu MỚI
            Axis_Base.moveTo(Moving_Coordinates[autoChainStep].X);
            Axis_Shoulder.moveTo(Moving_Coordinates[autoChainStep].Y);
            Axis_Elbow.moveTo(Moving_Coordinates[autoChainStep].Z);
          }
        }
      } // Kết thúc logic autoChainMode
      
      // 2. Luôn gọi run() (cho cả chạy chuỗi và goHome)
      Axis_Base.run();
      Axis_Shoulder.run();
      Axis_Elbow.run();
      ledcWrite(Axis_Gripper_forward, speedGripperForward); 
      ledcWrite(Axis_Gripper_backward,speedGripperBackward); 
      
    // Luôn delay để nhả CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
    } // Kết thúc (robotMode == 0)
}    


// ====== Khởi tạo các task ======
void setup_task() {
  // Khởi tạo 2 Task điều khiển motor
  // Ưu tiên cao (5) để đảm bảo chạy mượt
  xTaskCreate(MoveJog, "MoveJog", 2048, NULL, 5, NULL);
  xTaskCreate(motor_task_AUTO, "MotorRunAUTO", 4096, NULL, 5, NULL);
}

// ====== Hàm setup ======
void setup() {
  Serial.begin(115200);
  //=====Cấu hình chân công tắc hành trình=====
  pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);
  pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP);
  pinMode(LIM_BASE_PIN, INPUT_PULLUP);

  //=====Cấu hình cho trục càng chạy PWM ======
  ledcAttachChannel(Axis_Gripper_forward, 5000, 8, 0); // chân, freq, resolution, channel
  ledcAttachChannel(Axis_Gripper_backward, 5000, 8, 1); // chân, freq, resolution, channel


  //=====Cài đặt thông số động cơ bước=====
  Axis_Base.setMaxSpeed(1000);
  Axis_Base.setAcceleration(2000);
  Axis_Shoulder.setMaxSpeed(1000);
  Axis_Shoulder.setAcceleration(2000);
  Axis_Elbow.setMaxSpeed(1000);
  Axis_Elbow.setAcceleration(2000);

  // ===== Thực hiện homing trước khi khởi tạo task =====
  Serial.println("Robot chua san sang.");
  SetHomeAll();

  //=====Khởi tạo các task RTOS=====
  setup_task();
  
  Serial.println("Robot da san sang.");
  Serial.println("Chuyen sang che do AUTO (0).");
}

// ====== Hàm loop ======
void loop() {
  // Để trống hoặc cho task chính ngủ
  // Vì tất cả logic đã nằm trong các Task RTOS
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}