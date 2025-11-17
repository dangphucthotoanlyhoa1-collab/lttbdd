#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>

// Function prototypes (paste after #include ...)
void IRAM_ATTR onEncoderPulse();
long readEncoderCount();
void MoveAxisGripper(int direction, int speed);
int DetachObject();
int homeAxis(AccelStepper &axis, int limPin, const char *name);
void SetHomeAll();
void ReturnToHome(void *parameter);
void MoveJog(void *parameter);
void motor_task_AUTO(void *parameter);
void setup_task();

//========== Khai báo cấu trúc tọa độ XYZT và Tốc độ chuyển động ======
typedef struct
{
  float X;
  float Y;
  float Z;
  int T; // (SỬA) Đây là biến "Type": 0=Stepper, 1=Kẹp, 2=Nhả
  float Speed_X;
  float Speed_Y;
  float Speed_Z;
  int Speed_T; // Tốc độ kẹp/nhả

} XYZT_Coordinates;

XYZT_Coordinates Moving_Coordinates[1000];
XYZT_Coordinates Classify[1000];

//========== Khai báo các đối tượng stepper motor ======
AccelStepper Axis_Base(AccelStepper::DRIVER, 26, 27); // Step, Dir
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 14, 12); // Step, Dir
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 25, 33); // Step, Dir
#define Axis_Gripper_forward_PIN 3 // kẹp lại
#define Axis_Gripper_backward_PIN 4 // mở ra
volatile int Direction_Gripper = 0; // Hướng kẹp vật: 0= đóng càng, 1=mở càng
// (XÓA) 2 biến tốc độ này, chúng ta sẽ đọc từ mảng
// int speedGripperForward = 0; 
// int speedGripperBackward = 0; 

//========== Khai báo các biến toàn cục ======

// ====== Chân công tắc hành trình ======
const int LIM_BASE_PIN     = 15; // chỉnh theo phần cứng
const int LIM_SHOULDER_PIN = 13; // chỉnh theo phần cứng
const int LIM_ELBOW_PIN    = 23; // chỉnh theo phần cứng

// ====== Các biến điều khiển robot (State Machine) ======
volatile int Auto_Manual = 0; // 1=AUTO (dùng run()), 0=Manual (dùng runSpeed())
int classifyMode = 0; // 0=Không phân loại, 1=Phân loại
volatile int jogCommand = 0; // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0; // Vận tốc cố định 600
volatile int autoChainMode = 0; // 0=stop, 1=running
volatile int autoChainStep = 0; // bước hiện tại trong chuỗi
int TotalStep = 0; // Số bước chạy auto (sẽ được cập nhật khi nhận lệnh từ web)


//=======Biến điều khiển encoder gripper ======
const int ENCODER_PIN = 35; // Chân đọc encoder
volatile long g_encoderCount = 0;
volatile float Object_Size = 0.0; // Kích thước vật cầm nắm (cm)
const float GRIPPER_PULSES_PER_CM = 100.0; // Số xung encoder trên mỗi cm di chuyển của gripper


// (MỚI) Hằng số phát hiện kẹp (Stall)
const int GRIPPER_CHECK_INTERVAL = 50;  // ms - Kiểm tra 20 lần/giây
const int GRIPPER_STALL_THRESHOLD = 5;  // Xung - Nếu ít hơn 5 xung/interval -> coi là Kẹp
const int GRIPPER_MOVE_THRESHOLD = 15;  // Xung - Phải thấy nhiều hơn 10 xung/interval


//================================================================================//
//================================================================================//
//===================== các tác vụ điều khiển robot ==============================//

// Hàm ngắt đọc encoder gripper

void IRAM_ATTR onEncoderPulse() {
  if (Direction_Gripper)
    g_encoderCount++; //mở càng
  else
    g_encoderCount--; //đóng càng
}

long readEncoderCount() {
  long count_copy;
  noInterrupts(); // TẮT ngắt
  count_copy = g_encoderCount; // Sao chép giá trị
  interrupts(); // BẬT ngắt lại ngay
  return count_copy;
}

// (SỬA) Sửa hàm MoveAxisGripper để dùng KÊNH (Channel)
void MoveAxisGripper(int direction, int speed) {
  
  if (direction == 1) { // Mở càng
    Direction_Gripper = 1; 
    ledcWrite(Axis_Gripper_forward_PIN, speed); // (SỬA) Ghi vào Kênh 0
    ledcWrite(Axis_Gripper_backward_PIN, 0);     // (SỬA) Ghi vào Kênh 1
    
  } else if (direction == 0) { // Đóng càng
    Direction_Gripper = 0; 
    ledcWrite(Axis_Gripper_forward_PIN, 0);     // (SỬA) Ghi vào Kênh 0
    ledcWrite(Axis_Gripper_backward_PIN, speed); // (SỬA) Ghi vào Kênh 1
    
  } else { // Dừng
    ledcWrite(Axis_Gripper_forward_PIN, 0); // (SỬA) Ghi vào Kênh 0
    ledcWrite(Axis_Gripper_backward_PIN, 0); // (SỬA) Ghi vào Kênh 1
  }
}

  long time_now = 0;
  long value_now = 0;
  long time_last = 0;
  long value_last = 0;
  int HasConsitancy = 0;

int DetachObject(){
  time_now = millis();
  value_now = readEncoderCount();
  if (time_now - time_last >= GRIPPER_CHECK_INTERVAL) {
    long delta = abs(value_now - value_last);
    time_last = time_now;
    value_last = value_now;
    if (delta > GRIPPER_MOVE_THRESHOLD){
      HasConsitancy = 1; // đã ổn định chuyển động
    }
    if (delta < GRIPPER_STALL_THRESHOLD && HasConsitancy == 1){
      Serial.println("Da phat hien vat bi kep!");
      Object_Size = (float)(value_now) / GRIPPER_PULSES_PER_CM; // tính kích thước vật
        time_now = 0;
        value_now = 0;
        time_last = 0;
        value_last = 0;
        HasConsitancy = 0;
        return 1; // đã kẹp vật
    }
  }
  return 0; // chưa kẹp vật
}

// Hàm homing cho 1 trục (blocking)
int homeAxis(AccelStepper &axis, int limPin, const char *name) {
  if (Auto_Manual == 0){
  const float fastSpeed = -800; 
  const float slowSpeed = -400; 
  const float retreatSpeed = 400.0; 
  const int retreatSteps = 400; 

  axis.setSpeed(fastSpeed);
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
  }
  axis.setSpeed(0);
  axis.runSpeed();
  delay(20);

  axis.setCurrentPosition(0); 
  axis.setSpeed(retreatSpeed);
  while (axis.currentPosition() < retreatSteps) {
    axis.runSpeed();
  }
  axis.setSpeed(0);
  axis.runSpeed();
  delay(20);

  axis.setSpeed(slowSpeed);
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
  }
  axis.setSpeed(0); 
  axis.runSpeed();
  axis.setCurrentPosition(0); 
  Serial.print(name); 
  Serial.println(" homed!");
  delay(50);
  return 1; // Homing thành công
}
}

// Hàm homing cho cả 3 trục (blocking)
void SetHomeAll() {
  Auto_Manual = -1;
  jogCommand = 0;
  autoChainMode = 0;

  int HomingBase = 0;
  int HomingShoulder = 0;
  int HomingElbow = 0;
  int HomingGripper = 0;
  int time_out = 0;
  while (time_out < 20000){ // timeout 20s

      time_out = millis();
      Serial.println("Starting homing sequence...");
      HomingBase = homeAxis(Axis_Base, LIM_BASE_PIN, "Axis Base");
      HomingShoulder =  homeAxis(Axis_Shoulder, LIM_SHOULDER_PIN, "Axis Shoulder");
      HomingElbow = homeAxis(Axis_Elbow, LIM_ELBOW_PIN, "Axis Elbow");
      MoveAxisGripper(1, 150); // đóng càng với tốc độ 150
      HomingGripper = DetachObject();

      if (HomingGripper == 1){
        MoveAxisGripper(-1, 0); // dừng càng
        g_encoderCount = 0; // reset encoder count
      }
    if (HomingBase && HomingShoulder && HomingElbow && HomingGripper){
      Serial.println("Homing completed!");
      Auto_Manual = 0;
      return;
      
    }
  }
  return;
}

//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void ReturnToHome(void *parameter) {
  Auto_Manual = -1;
  jogCommand = 0;
  autoChainMode = 0; 
  
  while(Axis_Base.distanceToGo() == 0 && 
        Axis_Shoulder.distanceToGo() == 0 && 
        Axis_Elbow.distanceToGo() == 0 &&
        g_encoderCount == 0) // reset encoder count
        {
  Axis_Base.setMaxSpeed(600);
  Axis_Shoulder.setMaxSpeed(600);
  Axis_Elbow.setMaxSpeed(600);

  Axis_Base.moveTo(0);
  Axis_Shoulder.moveTo(0);
  Axis_Elbow.moveTo(0);
  MoveAxisGripper(1, 150); // đóng càng với tốc độ 150

  Axis_Base.run();
  Axis_Shoulder.run();
  Axis_Elbow.run();

  vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// ====== Move Jog Task (Chế độ JOG) ======
void MoveJog(void *parameter) {

  while(1) { 
    if (Auto_Manual == 0) { // Chỉ chạy khi ở chế độ MANUAL (JOG)
      Axis_Base.setSpeed(0);
      Axis_Shoulder.setSpeed(0);
      Axis_Elbow.setSpeed(0); 
      switch (jogCommand) {
        case 1: Axis_Base.setSpeed(JOG_SPEED); break;
        case 2: Axis_Base.setSpeed(-JOG_SPEED); break;
        case 3: Axis_Shoulder.setSpeed(JOG_SPEED); break;
        case 4: Axis_Shoulder.setSpeed(-JOG_SPEED); break;
        case 5: Axis_Elbow.setSpeed(JOG_SPEED); break;
        case 6: Axis_Elbow.setSpeed(-JOG_SPEED); break;
      }
      
      Axis_Base.runSpeed();
      Axis_Shoulder.runSpeed();
      Axis_Elbow.runSpeed();
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}


//====== (SỬA LẠI HOÀN TOÀN) Task chạy chế độ AUTO (Đồng bộ Stepper + Gripper) ======
void motor_task_AUTO(void *parameter) {
  int Direction_Gripper = 0;
  int Axis_Gripper_Speed = 0;
  int Axis_Gripper_Next_Position = 0;
  while(1) {
   
    if (Auto_Manual == 1) { // Chỉ chạy khi ở chế độ AUTO
      if (autoChainMode == 1) { // Chạy chuỗi tự động
        // Kiểm tra xem 3 trục đã dừng lại (hoàn thành bước) chưa
        if (Axis_Base.distanceToGo() == 0 && 
            Axis_Shoulder.distanceToGo() == 0 && 
            Axis_Elbow.distanceToGo() == 0 &&
            g_encoderCount == Axis_Gripper_Next_Position) {
             // và kẹp đã dừng{
              
          // Đã hoàn thành bước autoChainStep
          Serial.print("Hoan thanh buoc: "); Serial.println(autoChainStep);
          autoChainStep++; // Chuyển sang bước tiếp theo
          
          if (autoChainStep > TotalStep) {
            autoChainStep = 0; 
            }
          } 
            // Đặt tốc độ MỚI 
            if (classifyMode == 1){ 
            Axis_Base.setMaxSpeed(Classify[autoChainStep].Speed_X);
            Axis_Shoulder.setMaxSpeed(Classify[autoChainStep].Speed_Y);
            Axis_Elbow.setMaxSpeed(Classify[autoChainStep].Speed_Z);
            Axis_Gripper_Speed = Classify[autoChainStep].Speed_T;

            Axis_Base.moveTo(Classify[autoChainStep].X);
            Axis_Shoulder.moveTo(Classify[autoChainStep].Y);
            Axis_Elbow.moveTo(Classify[autoChainStep].Z);
            Axis_Gripper_Next_Position = Classify[autoChainStep].T;

            }
            else {
            Axis_Base.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_X);
            Axis_Shoulder.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Y);
            Axis_Elbow.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Z);
            Axis_Gripper_Speed = Moving_Coordinates[autoChainStep].Speed_T;

            Axis_Base.moveTo(Moving_Coordinates[autoChainStep].X);
            Axis_Shoulder.moveTo(Moving_Coordinates[autoChainStep].Y);
            Axis_Elbow.moveTo(Moving_Coordinates[autoChainStep].Z);
            Axis_Gripper_Next_Position = Moving_Coordinates[autoChainStep].T;
            }
            // Đặt mục tiêu MỚI

          }
        }
      if (Axis_Gripper_Next_Position > g_encoderCount){
        Direction_Gripper = 1; // mở càng
      } else if (Axis_Gripper_Next_Position < g_encoderCount){
        Direction_Gripper = 0; // đóng càng
      } else {
        Direction_Gripper = -1; // dừng
      }
      
      Axis_Base.run();
      Axis_Shoulder.run();
      Axis_Elbow.run();
      
      if (g_encoderCount != Axis_Gripper_Next_Position){
        MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
      } else {
        MoveAxisGripper(-1, 0); // dừng càng
      }
      
    // Luôn delay để nhả CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);

      }   

  }


// ====== Khởi tạo các task ======
void setup_task() {
  xTaskCreate(MoveJog, "MoveJog", 2048, NULL, 1, NULL);
  xTaskCreate(motor_task_AUTO, "MotorRunAUTO", 4096, NULL, 1, NULL);
  xTaskCreate(ReturnToHome, "ReturnToHome", 4096, NULL, 1, NULL);
}

// ====== Hàm setup ======
void setup() {
  Serial.begin(115200);
  pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);
  pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP);
  pinMode(LIM_BASE_PIN, INPUT_PULLUP);

  //===== (SỬA) Cấu hình PWM cho Gripper ======
  ledcAttachChannel(Axis_Gripper_forward_PIN, 5000, 8, 0); // Kênh 0 cho mở càng
  ledcAttachChannel(Axis_Gripper_backward_PIN, 5000, 8, 1); // Kênh 1 cho đóng càng
  // cấu hình encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), onEncoderPulse, FALLING);
  Serial.println("Encoder interrupt ready!");

  //=====Cài đặt thông số động cơ bước=====
  Axis_Base.setMaxSpeed(1000);
  Axis_Base.setAcceleration(2000);
  Axis_Shoulder.setMaxSpeed(1000);
  Axis_Shoulder.setAcceleration(2000);
  Axis_Elbow.setMaxSpeed(1000);
  Axis_Elbow.setAcceleration(2000);

  Serial.println("Robot chua san sang.");
  SetHomeAll();

  setup_task();

  Serial.println("Robot da san sang.");
  Serial.println("Chuyen sang che do AUTO (0).");
}

// ====== Hàm loop ======
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}