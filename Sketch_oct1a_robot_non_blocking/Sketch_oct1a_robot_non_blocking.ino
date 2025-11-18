#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>

/*
===========Dạng Chuỗi JSON tổng quát cho tất cả tác vụ========================
{
  "task": "jog|autochain|homing|status",
  
  // === JOG (Manual mode) ===
  "command": 1,
  "action": "start|stop",
  
  // === AUTOCHAIN (Auto mode) ===
  "mode": 1,
  "classify": 0,
  "totalSteps": 2,
  "coordinates": [
    {
      "X": 0,
      "Y": 0,
      "Z": 0,
      "T": 0,
      "speed_X": 800,
      "speed_Y": 800,
      "speed_Z": 800,
      "speed_T": 100
    },
    {
      "X": 1000,
      "Y": 500,
      "Z": 300,
      "T": 100,
      "speed_X": 600,
      "speed_Y": 600,
      "speed_Z": 600,
      "speed_T": 150
    }
  ],
  
  // === HOMING ===
  "action": "home|return"
}

Không phải tất cả các trường trong JSON đều bắt buộc trong mỗi tác vụ. Nó chỉ là 1 JSON tổng quát để cho việc quản lý
trở nên dễ dàng và đồng bộ.

*/

// Function prototypes 
void IRAM_ATTR onEncoderPulse(); //intererupt handler for encoder
long readEncoderCount();           //read encoder count safely
void MoveAxisGripper(int direction, int speed); // direction: 0=close, 1=open, -1=stop
int DetachObject(); // detect if object is gripped and measure size
int homeAxis(AccelStepper &axis, int limPin, const char *name); // homing for one axis
void SetHomeAll(); // homing for all axes
void ReturnToHome(void *parameter); // non-blocking return to home task
void MoveJog(void *parameter); // JOG movement task
void motor_task_AUTO(void *parameter); // AUTO and classify movement task
void setup_task(); //initialize RTOS tasks

// HTTP handlers
void handleUnifiedCommand(); // main handler for all tasks
void handleJog_Unified(); // JOG handler
void handleAutoChain_Unified(); // AUTOCHAIN handler
void handleHoming_Unified(); // HOMING handler
void handleStatus_Unified(); // STATUS handler
void setup_webserver(); // setup web server and routing

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

//========== Khai báo Web Server và JSON Document ======
WebServer server(80);
StaticJsonDocument<4096> jsonDocument;
// ====== Wi‑Fi credentials (edit to your network) ======
const char *SSID = "Thanh Gia 1";
const char *PWD = "ThanhgiA1931";

//========== Khai báo các đối tượng stepper motor ======
AccelStepper Axis_Base(AccelStepper::DRIVER, 26, 27);     // Step, Dir
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 14, 12); // Step, Dir
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 25, 33);    // Step, Dir
#define Axis_Gripper_forward_PIN 3                        // kẹp lại
#define Axis_Gripper_backward_PIN 4                       // mở ra
volatile int Direction_Gripper = 0;                       // Hướng kẹp vật: 0= đóng càng, 1= mở càng
// (XÓA) 2 biến tốc độ này, chúng ta sẽ đọc từ mảng
// int speedGripperForward = 0;
// int speedGripperBackward = 0;

//========== Khai báo các biến toàn cục ======

// ====== Chân công tắc hành trình ======
const int LIM_BASE_PIN = 15;     // chỉnh theo phần cứng
const int LIM_SHOULDER_PIN = 13; // chỉnh theo phần cứng
const int LIM_ELBOW_PIN = 23;    // chỉnh theo phần cứng

// ====== Các biến điều khiển robot (State Machine) ======
volatile int Auto_Manual = 0;   // 1=AUTO (dùng run()), 0=Manual (dùng runSpeed())
int classifyMode = 0;           // 0=Không phân loại, 1=Phân loại
volatile int jogCommand = 0;    // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0;  // Vận tốc cố định 600
volatile int autoChainMode = 0; // 0=stop, 1=running
volatile int autoChainStep = 0; // bước hiện tại trong chuỗi
int TotalStep = 0;              // Số bước chạy auto (sẽ được cập nhật khi nhận lệnh từ web)

//=======Biến điều khiển encoder gripper ======
const int ENCODER_PIN = 35; // Chân đọc encoder
volatile long g_encoderCount = 0;
volatile float Object_Size = 0.0;        // Kích thước vật cầm nắm (cm)
const float GRIPPER_PULSES_PER_CM = 0.1; // Số xung encoder trên mỗi mm di chuyển của gripper

// (MỚI) Hằng số phát hiện kẹp (Stall)
const int GRIPPER_CHECK_INTERVAL = 50; // ms - Kiểm tra 20 lần/giây
const int GRIPPER_STALL_THRESHOLD = 5; // Xung - Nếu ít hơn 5 xung/interval -> coi là Kẹp
const int GRIPPER_MOVE_THRESHOLD = 15; // Xung - Phải thấy nhiều hơn 10 xung/interval
int Object_Small_size = 0;
int Object_Large_size = 2;
int Object_Medium_size = 4;
int Number_of_Objects_small = 0;
int Number_of_Objects_medium = 0;
int Number_of_Objects_large = 0;

//================================================================================//
//================================================================================//
//===================== các tác vụ điều khiển robot ==============================//

// Hàm ngắt đọc encoder gripper

void IRAM_ATTR onEncoderPulse()
{
  if (Direction_Gripper)
    g_encoderCount++; // mở càng
  else
    g_encoderCount--; // đóng càng
}

long readEncoderCount()
{
  long count_copy;
  noInterrupts();              // TẮT ngắt
  count_copy = g_encoderCount; // Sao chép giá trị
  interrupts();                // BẬT ngắt lại ngay
  return count_copy;
}

// (SỬA) Sửa hàm MoveAxisGripper để dùng KÊNH (Channel)
void MoveAxisGripper(int direction, int speed)
{

  if (direction == 0)
  { // đóng càng
    Direction_Gripper = 0;
    ledcWrite(Axis_Gripper_forward_PIN, speed); 
    ledcWrite(Axis_Gripper_backward_PIN, 0);    
  }
  else if (direction == 1)
  { // Đóng càng
    Direction_Gripper = 1;
    ledcWrite(Axis_Gripper_forward_PIN, 0);      
    ledcWrite(Axis_Gripper_backward_PIN, speed); 
  }
  else
  {                                          // Dừng
    ledcWrite(Axis_Gripper_forward_PIN, 0);  // (SỬA) Ghi vào Kênh 0
    ledcWrite(Axis_Gripper_backward_PIN, 0); // (SỬA) Ghi vào Kênh 1
  }
}

long time_now = 0;
long value_now = 0;
long time_last = 0;
long value_last = 0;
int HasConsitancy = 0;

int DetachObject()
{
  time_now = millis();
  value_now = readEncoderCount();
  if (time_now - time_last >= GRIPPER_CHECK_INTERVAL)
  {
    long delta = abs(value_now - value_last);
    time_last = time_now;
    value_last = value_now;
    if (delta > GRIPPER_MOVE_THRESHOLD)
    {
      HasConsitancy = 1; // đã ổn định chuyển động
    }
    if (delta < GRIPPER_STALL_THRESHOLD && HasConsitancy == 1)
    {
      Serial.println("Da phat hien vat bi kep!");
      Object_Size = (float)(value_now) / GRIPPER_PULSES_PER_CM; // tính kích thước vật
      if (Object_Size < Object_Small_size)
      {
        Number_of_Objects_small++;
      }
      else if (Object_Size < Object_Medium_size)
      {
        Number_of_Objects_medium++;
      }
      else
      {
        Number_of_Objects_large++;
      }
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
int homeAxis(AccelStepper &axis, int limPin, const char *name)
{
  if (Auto_Manual == 0)
  {
    const float fastSpeed = -800;
    const float slowSpeed = -400;
    const float retreatSpeed = 400.0;
    const int retreatSteps = 400;

    axis.setSpeed(fastSpeed);
    while (digitalRead(limPin) == HIGH)
    {
      axis.runSpeed();
    }
    axis.setSpeed(0);
    axis.runSpeed();
    delay(20);

    axis.setCurrentPosition(0);
    axis.setSpeed(retreatSpeed);
    while (axis.currentPosition() < retreatSteps)
    {
      axis.runSpeed();
    }
    axis.setSpeed(0);
    axis.runSpeed();
    delay(20);

    axis.setSpeed(slowSpeed);
    while (digitalRead(limPin) == HIGH)
    {
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
void SetHomeAll()
{
  Auto_Manual = -1;
  jogCommand = 0;
  autoChainMode = 0;

  int HomingBase = 0;
  int HomingShoulder = 0;
  int HomingElbow = 0;
  int HomingGripper = 0;
  int time_out = 0;
  while (time_out < 20000)
  { // timeout 20s

    time_out = millis();
    Serial.println("Starting homing sequence...");
    HomingBase = homeAxis(Axis_Base, LIM_BASE_PIN, "Axis Base");
    HomingShoulder = homeAxis(Axis_Shoulder, LIM_SHOULDER_PIN, "Axis Shoulder");
    HomingElbow = homeAxis(Axis_Elbow, LIM_ELBOW_PIN, "Axis Elbow");
    MoveAxisGripper(0, 220); // đóng càng với tốc độ 220
    HomingGripper = DetachObject();

    if (HomingGripper == 1)
    {
      MoveAxisGripper(-1, 0); // dừng càng
      g_encoderCount = 0;     // reset encoder count
    }
    if (HomingBase && HomingShoulder && HomingElbow && HomingGripper)
    {
      Serial.println("Homing completed!");
      Auto_Manual = 0;
      return;
    }
  }
  return;
}

//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void ReturnToHome(void *parameter)
{
  Auto_Manual = -1;
  autoChainMode = 0;
  jogCommand = 0;
  
  while (Axis_Base.distanceToGo() == 0 &&
         Axis_Shoulder.distanceToGo() == 0 &&
         Axis_Elbow.distanceToGo() == 0 &&
         g_encoderCount == 0 ) // reset encoder count
  {
    Axis_Base.setMaxSpeed(600);
    Axis_Shoulder.setMaxSpeed(600);
    Axis_Elbow.setMaxSpeed(600);

    Axis_Base.moveTo(0);
    Axis_Shoulder.moveTo(0);
    Axis_Elbow.moveTo(0);
    MoveAxisGripper(0, 220); // đóng càng với tốc độ 220

    Axis_Base.run();
    Axis_Shoulder.run();
    Axis_Elbow.run();

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ====== Move Jog Task (Chế độ JOG) ======
void MoveJog(void *parameter)
{
  int Axis_Gripper_setSpeed = 0;
  int direction_Gripper_Jog = -1;
  while (1)
  {
    if (Auto_Manual == 0)
    { // Chỉ chạy khi ở chế độ MANUAL (JOG)
      Axis_Base.setSpeed(0);
      Axis_Shoulder.setSpeed(0);
      Axis_Elbow.setSpeed(0);
      Axis_Gripper_setSpeed = 0;
      direction_Gripper_Jog = -1;
      switch (jogCommand)
      {
      case 1:
        Axis_Base.setSpeed(JOG_SPEED);
        break; // nâng lên
      case 2:
        Axis_Base.setSpeed(-JOG_SPEED);
        break; // hạ xuống
      case 3:
        Axis_Shoulder.setSpeed(JOG_SPEED);
        break; // quay phải
      case 4:
        Axis_Shoulder.setSpeed(-JOG_SPEED);
        break; // quay trái
      case 5:
        Axis_Elbow.setSpeed(JOG_SPEED);
        break; // vươn ra
      case 6:
        Axis_Elbow.setSpeed(-JOG_SPEED);
        break; // thu lại
      case 7:
        direction_Gripper_Jog = 1;
        Axis_Gripper_setSpeed = 220;
        break; // mở càng
      case 8:
        direction_Gripper_Jog = 0;
        Axis_Gripper_setSpeed = 220;
        break; // đóng càng
      }

      Axis_Base.runSpeed();
      Axis_Shoulder.runSpeed();
      Axis_Elbow.runSpeed();
      MoveAxisGripper(direction_Gripper_Jog, Axis_Gripper_setSpeed);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

//====== (SỬA LẠI HOÀN TOÀN) Task chạy chế độ AUTO (Đồng bộ Stepper + Gripper) ======
void motor_task_AUTO(void *parameter)
{
  int Direction_Gripper = 0;
  int Axis_Gripper_Speed = 0;
  int Axis_Gripper_Next_Position = 0;
  while (1)
  {

    if (Auto_Manual == 1)
    { // Chỉ chạy khi ở chế độ AUTO
      if (autoChainMode == 1)
      { // Chạy chuỗi tự động
        // Kiểm tra xem 3 trục đã dừng lại (hoàn thành bước) chưa
        if (Axis_Base.distanceToGo() == 0 &&
            Axis_Shoulder.distanceToGo() == 0 &&
            Axis_Elbow.distanceToGo() == 0 &&
            g_encoderCount == Axis_Gripper_Next_Position)
        {
          // và kẹp đã dừng{

          // Đã hoàn thành bước autoChainStep
          Serial.print("Hoan thanh buoc: ");
          Serial.println(autoChainStep);
          autoChainStep++; // Chuyển sang bước tiếp theo

          if (autoChainStep > TotalStep)
          {
            autoChainStep = 0;
          }
        }
        // Đặt tốc độ MỚI
        if (classifyMode == 1)
        {
          Axis_Base.setMaxSpeed(Classify[autoChainStep].Speed_X);
          Axis_Shoulder.setMaxSpeed(Classify[autoChainStep].Speed_Y);
          Axis_Elbow.setMaxSpeed(Classify[autoChainStep].Speed_Z);
          Axis_Gripper_Speed = Classify[autoChainStep].Speed_T;

          Axis_Base.moveTo(Classify[autoChainStep].X);
          Axis_Shoulder.moveTo(Classify[autoChainStep].Y);
          Axis_Elbow.moveTo(Classify[autoChainStep].Z);
          Axis_Gripper_Next_Position = Classify[autoChainStep].T;
        }
        else
        {
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
    if (Axis_Gripper_Next_Position > g_encoderCount)
    {
      Direction_Gripper = 1; // mở càng
    }
    else if (Axis_Gripper_Next_Position < g_encoderCount)
    {
      Direction_Gripper = 0; // đóng càng
    }
    else
    {
      Direction_Gripper = -1; // dừng
    }

    Axis_Base.run();
    Axis_Shoulder.run();
    Axis_Elbow.run();

    if (g_encoderCount != Axis_Gripper_Next_Position)
    {
      MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
    }
    else
    {
      MoveAxisGripper(-1, 0); // dừng càng
    }

    // Luôn delay để nhả CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ====== Khởi tạo các task ======
void setup_task()
{
  xTaskCreate(MoveJog, "MoveJog", 2048, NULL, 2, NULL);
  xTaskCreate(motor_task_AUTO, "MotorRunAUTO", 4096, NULL, 3, NULL);
  xTaskCreate(ReturnToHome, "ReturnToHome", 4096, NULL, 1, NULL);
}

//================================================================================//
//===================== HTTP HANDLERS - Nhận dữ liệu từ Server ===================//
//================================================================================//

// Main unified handler
void handleUnifiedCommand() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
    return;
  }

  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  // Kiểm tra task có tồn tại không
  if (!jsonDocument.containsKey("task")) {
    server.send(400, "application/json", "{\"error\":\"task required\"}");
    return;
  }

  const char* task = jsonDocument["task"];

  // Phân phối theo task
  if (strcmp(task, "jog") == 0) {
    handleJog_Unified();
  }
  else if (strcmp(task, "autochain") == 0) {
    handleAutoChain_Unified();
  }
  else if (strcmp(task, "homing") == 0) {
    handleHoming_Unified();
  }
  else if (strcmp(task, "status") == 0) {
    handleStatus_Unified();
  }
  else {
    server.send(400, "application/json",
      "{\"error\":\"Invalid task. Valid: jog, autochain, homing, status\"}");
  }
}

// JOG Handler
void handleJog_Unified() {
  // Kiểm tra tham số bắt buộc
  if (!jsonDocument.containsKey("command")) {
    server.send(400, "application/json", "{\"error\":\"command required (1-8)\"}");
    return;
  }
  if (!jsonDocument.containsKey("action")) {
    server.send(400, "application/json", "{\"error\":\"action required (start/stop)\"}");
    return;
  }

  int command = jsonDocument["command"];
  const char* action = jsonDocument["action"];

  // Kiểm tra giá trị hợp lệ
  if (command < 0 || command > 8) {
    server.send(400, "application/json", "{\"error\":\"command must be 0-8\"}");
    return;
  }

  if (strcmp(action, "start") != 0 && strcmp(action, "stop") != 0) {
    server.send(400, "application/json", "{\"error\":\"action must be start or stop\"}");
    return;
  }

  // Xử lý lệnh
  if (strcmp(action, "start") == 0) {
    if (command == 0) {
      server.send(400, "application/json", "{\"error\":\"command cannot be 0 when action is start\"}");
      return;
    }
    Auto_Manual = 0; // Chế độ Manual/JOG
    autoChainMode = 0;
    jogCommand = command;
    server.send(200, "application/json", "{\"status\":\"jog_started\",\"command\":" + String(command) + "}");
  }
  else if (strcmp(action, "stop") == 0) {
    jogCommand = 0;
    server.send(200, "application/json", "{\"status\":\"jog_stopped\"}");
  }

  Serial.print("JOG Command: ");
  Serial.print(command);
  Serial.print(", Action: ");
  Serial.println(action);
}

// AUTOCHAIN Handler
void handleAutoChain_Unified() {
  // Kiểm tra tham số bắt buộc
  if (!jsonDocument.containsKey("mode")) {
    server.send(400, "application/json", "{\"error\":\"mode required (0=stop, 1=run)\"}");
    return;
  }
  if (!jsonDocument.containsKey("totalSteps")) {
    server.send(400, "application/json", "{\"error\":\"totalSteps required\"}");
    return;
  }
  if (!jsonDocument.containsKey("coordinates")) {
    server.send(400, "application/json", "{\"error\":\"coordinates array required\"}");
    return;
  }

  int mode = jsonDocument["mode"];
  int totalSteps = jsonDocument["totalSteps"];
  JsonArray coords = jsonDocument["coordinates"];
  int classify = jsonDocument["classify"] | 0;

  // Kiểm tra giá trị
  if (mode < 0 || mode > 1) {
    server.send(400, "application/json", "{\"error\":\"mode must be 0 or 1\"}");
    return;
  }
  if (totalSteps <= 0 || totalSteps > 1000) {
    server.send(400, "application/json", "{\"error\":\"totalSteps must be 1-1000\"}");
    return;
  }

  // Kiểm tra số phần tử trong mảng
  if (coords.size() != totalSteps) {
    server.send(400, "application/json",
      "{\"error\":\"coordinates size mismatch. Got " + String(coords.size()) +
      ", expected " + String(totalSteps) + "\"}");
    return;
  }

  // Kiểm tra mỗi coordinate có đủ tham số
  for (int i = 0; i < totalSteps; i++) {
    if (!coords[i].containsKey("X") || !coords[i].containsKey("Y") ||
        !coords[i].containsKey("Z") || !coords[i].containsKey("T") ||
        !coords[i].containsKey("speed_X") || !coords[i].containsKey("speed_Y") ||
        !coords[i].containsKey("speed_Z") || !coords[i].containsKey("speed_T")) {
      server.send(400, "application/json",
        "{\"error\":\"coordinate at index " + String(i) + " missing required fields\"}");
      return;
    }
  }

  // Lưu dữ liệu vào mảng
  for (int i = 0; i < totalSteps; i++) {
    if (classify == 1) {
      Classify[i].X = coords[i]["X"];
      Classify[i].Y = coords[i]["Y"];
      Classify[i].Z = coords[i]["Z"];
      Classify[i].T = coords[i]["T"];
      Classify[i].Speed_X = coords[i].containsKey("speed_X") ? (float)coords[i]["speed_X"] : 600.0;
      Classify[i].Speed_Y = coords[i].containsKey("speed_Y") ? (float)coords[i]["speed_Y"] : 600.0;
      Classify[i].Speed_Z = coords[i].containsKey("speed_Z") ? (float)coords[i]["speed_Z"] : 600.0;
      Classify[i].Speed_T = coords[i].containsKey("speed_T") ? (int)coords[i]["speed_T"] : 200;
    } else {
      Moving_Coordinates[i].X = coords[i]["X"];
      Moving_Coordinates[i].Y = coords[i]["Y"];
      Moving_Coordinates[i].Z = coords[i]["Z"];
      Moving_Coordinates[i].T = coords[i]["T"];
      Moving_Coordinates[i].Speed_X = coords[i].containsKey("speed_X") ? (float)coords[i]["speed_X"] : 600.0;
      Moving_Coordinates[i].Speed_Y = coords[i].containsKey("speed_Y") ? (float)coords[i]["speed_Y"] : 600.0;
      Moving_Coordinates[i].Speed_Z = coords[i].containsKey("speed_Z") ? (float)coords[i]["speed_Z"] : 600.0;
      Moving_Coordinates[i].Speed_T = coords[i].containsKey("speed_T") ? (int)coords[i]["speed_T"] : 200;
    }
  }

  // Cập nhật trạng thái
  Auto_Manual = 1; // Chế độ AUTO
  classifyMode = classify;
  autoChainStep = 0;
  autoChainMode = mode;
  TotalStep = totalSteps;

  if (mode == 1) {
    server.send(200, "application/json",
      "{\"status\":\"autochain_started\",\"totalSteps\":" + String(totalSteps) + "}");
    Serial.println("AUTOCHAIN: Started");
  } else {
    autoChainMode = 0;
    server.send(200, "application/json", "{\"status\":\"autochain_stopped\"}");
    Serial.println("AUTOCHAIN: Stopped");
  }
}

// HOMING Handler
void handleHoming_Unified() {
  if (!jsonDocument.containsKey("action")) {
    server.send(400, "application/json", "{\"error\":\"action required (home/return)\"}");
    return;
  }

  const char* action = jsonDocument["action"];

  if (strcmp(action, "home") == 0) {
    jogCommand = 0;
    autoChainMode = 0;
    SetHomeAll();
    server.send(200, "application/json", "{\"status\":\"homing_completed\"}");
    Serial.println("HOMING: Completed");
  }
  else if (strcmp(action, "return") == 0) {
    Auto_Manual = -1;
    autoChainMode = 0;
    jogCommand = 0;

    server.send(200, "application/json", "{\"status\":\"return_to_home_started\"}");
    Serial.println("HOMING: Return to home started");
  }
  else {
    server.send(400, "application/json", "{\"error\":\"action must be home or return\"}");
  }
}

// STATUS Handler
void handleStatus_Unified() {
  // JSON Document là biến toàn cục, có thể bị ghi đè khi có nhiều client cùng gọi.
  // Do đó dùng jsonDocument.clear() để tránh bị ghi đè, nhưng nếu dùng jsonDocument.clear()
  // nếu giả sử handleAutoChain_Unified đang phân tích chuỗi,
  // thì khi vừa gọi handleStatus_Unified, nó sẽ clear hết JSON Document đang dùng cho các tác vụ (POST,GET) khác.
  // ====> làm sai lệch dữ liệu.
  // Do đó dùng statusDoc là biến cục bộ trong hàm, ghi độc lập với tác vụ POST khác.
  StaticJsonDocument<512> statusDoc;

  // Nạp dữ liệu vào tài liệu (an toàn)
  statusDoc["auto_manual"] = Auto_Manual;
  statusDoc["jog_command"] = jogCommand;
  statusDoc["auto_chain_mode"] = autoChainMode;
  statusDoc["auto_chain_step"] = autoChainStep;
  statusDoc["total_steps"] = TotalStep;
  statusDoc["position_X"] = Axis_Base.currentPosition();
  statusDoc["position_Y"] = Axis_Shoulder.currentPosition();
  statusDoc["position_Z"] = Axis_Elbow.currentPosition();
  statusDoc["position_T"] = readEncoderCount();

  // Thêm thông tin về số lượng và kích thước vật
  statusDoc["object_count_small"] = Number_of_Objects_small;
  statusDoc["object_count_medium"] = Number_of_Objects_medium;
  statusDoc["object_count_large"] = Number_of_Objects_large;
  statusDoc["object_size"] = Object_Size;

  char statusBuffer[512];
  
  // Chuyển đổi tài liệu thành chuỗi JSON
  // Dùng sizeof(statusBuffer) để đảm bảo không bị tràn bộ đệm
  serializeJson(statusDoc, statusBuffer, sizeof(statusBuffer));
  
  server.send(200, "application/json", statusBuffer);
}

// Setup Web Server
void setup_webserver() {
  server.on("/command", HTTP_POST, handleUnifiedCommand);
  server.on("/status", HTTP_GET, handleStatus_Unified);
  server.begin();
  Serial.println("Web server started on /command and /status");
}

// ====== WiFi setup helper ======
void setupWiFi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ====== Khởi tạo các task ======

// ====== Hàm setup ======
void setup()
{
  Serial.begin(115200);
  pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);
  pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP);
  pinMode(LIM_BASE_PIN, INPUT_PULLUP);

  //===== (SỬA) Cấu hình PWM cho Gripper ======
  ledcAttachChannel(Axis_Gripper_forward_PIN, 5000, 8, 0);  // Kênh 0 cho mở càng
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
  setupWiFi();
  setup_webserver(); // Khởi tạo web server

  Serial.println("Robot da san sang.");
  Serial.println("Chuyen sang che do AUTO (0).");
}

// ====== Hàm loop ======
void loop()
{
  server.handleClient(); // Xử lý client requests
  // vTaskDelay(1 / portTICK_PERIOD_MS);
}