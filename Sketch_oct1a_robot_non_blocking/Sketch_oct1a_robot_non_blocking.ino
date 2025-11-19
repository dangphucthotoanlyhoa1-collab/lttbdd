#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>

// Function prototypes
void IRAM_ATTR onEncoderPulse();                                // intererupt handler for encoder
long readEncoderCount();                                        // read encoder count safely
void MoveAxisGripper(int direction, int speed);                 // direction: 0=close, 1=open, -1=stop
int DetachObject();                                             // detect if object is gripped and measure size
int homeAxis(AccelStepper &axis, int limPin, const char *name); // homing for one axis
void SetHomeAll();                                              // homing for all axes
void ReturnToHome(void *parameter);                             // non-blocking return to home task
void MoveJog(void *parameter);                                  // JOG movement task
void motor_task_AUTO(void *parameter);                          // AUTO and classify movement task
void setup_task();                                              // initialize RTOS tasks
void setup_webserver();                                         // initialize web server

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
XYZT_Coordinates CurrentPosition;

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
volatile int command_jog = 0;       // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0;     // Vận tốc cố định 600
volatile int Mode = -3;            // -3= Stop, -2= SetHome ,-1=ReturnHome, 0=jog, 1=chain, 2=classify
volatile int autoChainStep = 0;    // bước hiện tại trong chuỗi
volatile int autoClassifyStep = 0; // bước hiện tại trong phân loại
int TotalSteps = 0;                 // Số bước chạy auto (sẽ được cập nhật khi nhận lệnh từ web)
int TotalStep = 0;                  // legacy alias (some older code refers to TotalStep)

//=======Biến điều khiển encoder gripper ======
const int ENCODER_PIN = 35; // Chân đọc encoder
volatile long g_encoderCount = 0;
volatile float Object_Size = 0.0;        // Kích thước vật cầm nắm (cm)
const float GRIPPER_PULSES_PER_CM = 0.1; // Số xung encoder trên mỗi mm di chuyển của gripper

// (MỚI) Hằng số phát hiện kẹp (Stall)
const int GRIPPER_CHECK_INTERVAL = 50; // ms - Kiểm tra 20 lần/giây
const int GRIPPER_STALL_THRESHOLD = 5; // Xung - Nếu ít hơn 5 xung/interval -> coi là Kẹp
const int GRIPPER_MOVE_THRESHOLD = 15; // Xung - Phải thấy nhiều hơn 10 xung/interval
volatile int torque = 140;             // Lực giữ khi kẹp vật
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
  if (Mode == -2)
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
  int HomingBase = 0;
  int HomingShoulder = 0;
  int HomingElbow = 0;
  int HomingGripper = 0;
  int time_out = 0;
  while (time_out < 20000)
  { // timeout 20s
    if (Mode == -2)
    {

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
        return;
      }
    }
  }
  return;
}

//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void ReturnToHome(void *parameter)
{
  if (Mode == -1)
  {
    while (Axis_Base.distanceToGo() == 0 &&
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
      MoveAxisGripper(0, 220); // đóng càng với tốc độ 220

      Axis_Base.run();
      Axis_Shoulder.run();
      Axis_Elbow.run();

      vTaskDelay(20 / portTICK_PERIOD_MS);
    }
  }
}

int Axis_Gripper_setSpeed = 0;
int direction_Gripper_Jog = -1;
// ====== Move Jog Task (Chế độ JOG) ======
void MoveJog(void *parameter)
{

  while (1)
  {
    if (Mode == 0)
    { // Chỉ chạy khi ở chế độ MANUAL (JOG)
      Axis_Base.setSpeed(0);
      Axis_Shoulder.setSpeed(0);
      Axis_Elbow.setSpeed(0);
      Axis_Gripper_setSpeed = 0;
      direction_Gripper_Jog = -1;
      switch (command_jog)
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

int Direction_Gripper = 0;
int Axis_Gripper_Speed = 0;
int Axis_Gripper_Next_Position = 0;
//====== (SỬA LẠI HOÀN TOÀN) Task chạy chế độ AUTO (Đồng bộ Stepper + Gripper) ======
void motor_task_AUTO(void *parameter)
{
  while (1)
  {
    if (Mode == 2)
    { // Chỉ chạy khi ở chế độ AUTO
      // Chạy chuỗi tự động
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
      // Đặt tốc độ MỚI và mục tiêu MỚI nếu bước thay đổi
      Axis_Base.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_X);
      Axis_Shoulder.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Y);
      Axis_Elbow.setMaxSpeed(Moving_Coordinates[autoChainStep].Speed_Z);
      Axis_Gripper_Speed = Moving_Coordinates[autoChainStep].Speed_T;

      Axis_Base.moveTo(Moving_Coordinates[autoChainStep].X);
      Axis_Shoulder.moveTo(Moving_Coordinates[autoChainStep].Y);
      Axis_Elbow.moveTo(Moving_Coordinates[autoChainStep].Z);
      Axis_Gripper_Next_Position = Moving_Coordinates[autoChainStep].T;

      // Đặt mục tiêu MỚI

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
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

//===================== Task chạy chế độ CLASSIFY =========================
int Axis_Gripper_Stop = 0;
int Detach_object_flag = 0;
int Disable_Axis_Gripper = 0;
void motor_task_Classify(void *parameter)
{
  while (1)
  {
    if (Mode == 2)
    {
      if (Axis_Base.distanceToGo() == 0 &&
          Axis_Shoulder.distanceToGo() == 0 &&
          Axis_Elbow.distanceToGo() == 0 &&
          Axis_Gripper_Stop == 1)
      {
        autoClassifyStep++;
        Axis_Gripper_Stop = 0; // Chuyển sang bước tiếp theo
        if (autoClassifyStep == 6)
        {
          disable_Axis_Gripper = 0;
          Axis_Gripper_Stop = 0;
          Detach_object_flag = 0;
        }
        if (autoClassifyStep > 6)
        {
          autoClassifyStep = 0;
        }
      }

      Axis_Base.setMaxSpeed(Moving_Coordinates[autoClassifyStep].Speed_X);
      Axis_Shoulder.setMaxSpeed(Moving_Coordinates[autoClassifyStep].Speed_Y);
      Axis_Elbow.setMaxSpeed(Moving_Coordinates[autoClassifyStep].Speed_Z);
      Axis_Gripper_Speed = Moving_Coordinates[autoClassifyStep].Speed_T;

      Axis_Base.moveTo(Moving_Coordinates[autoClassifyStep].X);
      Axis_Shoulder.moveTo(Moving_Coordinates[autoClassifyStep].Y);
      Axis_Elbow.moveTo(Moving_Coordinates[autoClassifyStep].Z);
      Axis_Gripper_Next_Position = Moving_Coordinates[autoClassifyStep].T;
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
    if (autoClassifyStep == 3)
    {
      Detach_object_flag = DetachObject(); // lên 1 trong 1 chu kỳ quét
    }
    if (disable_Axis_Gripper == 0)
    {
      if (Detach_object_flag == 0)
      {
        if (g_encoderCount != Axis_Gripper_Next_Position)
        {
          MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
        }
        if (g_encoderCount == Axis_Gripper_Next_Position)
        {
          MoveAxisGripper(-1, 0); // dừng càng
          Axis_Gripper_Stop = 1;
        }
      }
      else
      {
        Axis_Gripper_Stop = 1;
        MoveAxisGripper(Direction_Gripper, torque); // giữ lực kẹp
        disable_Axis_Gripper = 1;
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ====== Khởi tạo các task ======
void setup_task()
{
  xTaskCreate(MoveJog, "MoveJog", 2048, NULL, 2, NULL);
  xTaskCreate(motor_task_AUTO, "MotorRunAUTO", 4096, NULL, 3, NULL);
  xTaskCreate(ReturnToHome, "ReturnToHome", 4096, NULL, 1, NULL);
  xTaskCreate(motor_task_Classify, "MotorRunClassify", 4096, NULL, 3, NULL);
}

//================================================================================//
//===================== HTTP HANDLERS - Nhận dữ liệu từ Server ===================//
//================================================================================//
void UpdateCurrentPosition()
{
  CurrentPosition.X = Axis_Base.currentPosition();
  CurrentPosition.Y = Axis_Shoulder.currentPosition();
  CurrentPosition.Z = Axis_Elbow.currentPosition();
  CurrentPosition.T = g_encoderCount;
  CurrentPosition.Speed_X = Axis_Base.speed();
  CurrentPosition.Speed_Y = Axis_Shoulder.speed();
  CurrentPosition.Speed_Z = Axis_Elbow.speed();
}

void handleUnifiedCommand() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }
  
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  
  String task = jsonDocument["task"];
  Serial.print("Web Command: "); Serial.println(task);

  // --- STOP ---
  if (task == "Stop") {
    Mode = -3; // Stop tất cả
    Axis_Base.stop(); Axis_Shoulder.stop(); Axis_Elbow.stop();
    MoveAxisGripper(-1, 0);
    server.send(200, "application/json", "{\"status\":\"Stopped\"}");
  }
  // --- JOG ---
  else if (task == "Jog") {
    Mode = 0;
    command_jog = jsonDocument["command_jog"];
    server.send(200, "application/json", "{\"status\":\"Jogging\"}");
  }
  // --- AUTO CHAIN ---
  else if (task == "Autochain") {
    Mode = -2; // Stop trước khi nạp
    TotalSteps = jsonDocument["TotalSteps"];
    JsonArray coords = jsonDocument["Coordinates_auto"];
    
    for(int i=0; i<TotalSteps; i++) {
      Moving_Coordinates[i].X = coords[i]["X"];
      Moving_Coordinates[i].Y = coords[i]["Y"];
      Moving_Coordinates[i].Z = coords[i]["Z"];
      Moving_Coordinates[i].T = coords[i]["T"];
      Moving_Coordinates[i].Speed_X = coords[i]["speed_X"];
      Moving_Coordinates[i].Speed_Y = coords[i]["speed_Y"];
      Moving_Coordinates[i].Speed_Z = coords[i]["speed_Z"];
      Moving_Coordinates[i].Speed_T = coords[i]["speed_T"];
    }
    autoChainStep = 0;
    Mode = 1; // Start Auto
    server.send(200, "application/json", "{\"status\":\"Auto Started\"}");
  }
  // --- CLASSIFY ---
  else if (task == "Classify") {
    Mode = -2;
    TotalSteps = jsonDocument["TotalSteps"];
    JsonArray coords = jsonDocument["Coordinates_classify"];
    
    for(int i=0; i<TotalSteps; i++) {
      Classify[i].X = coords[i]["X"];
      Classify[i].Y = coords[i]["Y"];
      Classify[i].Z = coords[i]["Z"];
      Classify[i].T = coords[i]["T"];
      Classify[i].Speed_X = coords[i]["speed_X"];
      Classify[i].Speed_Y = coords[i]["speed_Y"];
      Classify[i].Speed_Z = coords[i]["speed_Z"];
      Classify[i].Speed_T = coords[i]["speed_T"];
    }
    currentStep = 0;
    Mode = 2; // Start Classify
    server.send(200, "application/json", "{\"status\":\"Classify Started\"}");
  }
  else {
    server.send(400, "application/json", "{\"error\":\"Unknown task\"}");
  }
}

// Setup Web Server
void setup_webserver()
{
  server.on("/command", HTTP_POST, handleUnifiedCommand);
  server.on("/status", HTTP_GET, handleStatus_Unified);
  server.on("/current_position", HTTP_GET, UpdateCurrentPosition);
  server.begin();
  Serial.println("Web server started on /command and /status");
}

// ====== WiFi setup helper ======
void setupWiFi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED)
  {
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
  pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);    // cấu hình chân công tắc hành trình trục khuỷu
  pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP); // cấu hình chân công tắc hành trình trục nâng
  pinMode(LIM_BASE_PIN, INPUT_PULLUP);     // cấu hình chân công tắc hành trình trục quay

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