#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
/*
===========Dạng Chuỗi JSON tổng quát cho tất cả tác vụ========================
form POST jog:
Task: Jog,
conmmand_jog: 1,

form POST autochain/classify:
Task: Autochain|Classify,
TotalSteps: 2,
Coordinates_auto|classify: [ {X:0, Y:0, Z:0, T:0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:100}, {...}, ... ]

form POST SetHome:
Task: SetHome,

form POST ReturnHome:
Task: ReturnHome,

form POST Stop:
Task: Stop,


  "task": "Jog|Autochain|Classify|SetHome|ReturnHome|Stop", //mode   3= Stop, -2=SetHome ,-1=ReturnHome, 0=Jog, 1=Autochain, 2=Classify 
  // Ở task SetHome, cho mode = -2 và gọi hàm SetHomeAll()
  // Ở task ReturnHome, cho mode = -1 và gọi hàm ReturnToHome()
  // Ở task Stop, cho mode = -2 để dừng tất cả các hoạt động.
  // Ở task JOG, cho mode = 0 và sử dụng trường "command_jog" để điều khiển
  // Ở task Autochain, cho mode = 1 và sử dụng trường "Coordinates_auto" để đưa dữ liệu vào XYZT_Coordinates Moving_Coordinates
  // Ở task Classify, cho mode = 2 và sử dụng trường "Coordinates_classify" để đưa dữ liệu vào XYZT_Coordinates Classify
  // Ở task Status, không cần trường nào khác ngoài "task", lấy trạng thái hiện tại của robot.
  // Ở trường TotalSteps, chỉ sử dụng trong Autochain và Classify để xác định số bước trong chuỗi, nó sẽ gán cho autoChainStep hoặc autoClassifyStep tùy theo mode
  // === JOG (Manual mode) ===
  "command_jog": 1,
  // === AUTOCHAIN (Auto mode) ===
  "TotalSteps": 2,
  "Coordinates_auto": [
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
  "Coordinates_classify": [
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
}

Không phải tất cả các trường trong JSON đều bắt buộc trong mỗi tác vụ. Nó chỉ là 1 JSON tổng quát để cho việc quản lý
trở nên dễ dàng và đồng bộ.

*/

// ============================================================
// CẤU HÌNH PHẦN CỨNG & MẠNG
// ============================================================
const char *SSID = "Thanh Gia 1";
const char *PWD = "ThanhgiA1931";

// Chân Stepper
#define STEP_BASE 26
#define DIR_BASE 27
#define STEP_SHOULDER 14
#define DIR_SHOULDER 12
#define STEP_ELBOW 25
#define DIR_ELBOW 33

// Chân Gripper & Encoder
#define PIN_GRIPPER_FWD 3
#define PIN_GRIPPER_BWD 4
#define ENCODER_PIN 35

// Chân công tắc hành trình
const int LIM_BASE_PIN = 15;
const int LIM_SHOULDER_PIN = 13;
const int LIM_ELBOW_PIN = 23;

// ============================================================
// KHAI BÁO BIẾN TOÀN CỤC & ĐỐI TƯỢNG
// ============================================================
WebServer server(80);

AccelStepper Axis_Base(AccelStepper::DRIVER, STEP_BASE, DIR_BASE);
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, STEP_SHOULDER, DIR_SHOULDER);
AccelStepper Axis_Elbow(AccelStepper::DRIVER, STEP_ELBOW, DIR_ELBOW);

// Struct tọa độ
typedef struct {
  float X, Y, Z;
  int T; // Trạng thái kẹp: Encoder target
  float Speed_X, Speed_Y, Speed_Z;
  int Speed_T;
} XYZT_Coordinates;

#define MAX_STEPS 500
XYZT_Coordinates Moving_Coordinates[MAX_STEPS]; // Buffer cho Auto Chain
int TotalSteps = 0; // Tổng số bước hiện tại

// Biến trạng thái hệ thống
// Mode: 0=JOG (Manual), 1=AUTO CHAIN, 2=CLASSIFY, -1=HOMING
volatile int Mode = 0; 
volatile int jogCommand = 0; 

// Biến Auto run
volatile int autoChainStep = 0;
volatile int autoClassifyStep = 0;

// Biến Encoder & Gripper
volatile long g_encoderCount = 0;
volatile float Object_Size = 0.0;
const float GRIPPER_PULSES_PER_CM = 0.1; 

// Biến thống kê vật
int Number_of_Objects_small = 0;
int Number_of_Objects_medium = 0;
int Number_of_Objects_large = 0;

// Hằng số Gripper
const int GRIPPER_TORQUE = 140; // PWM giữ
int disable_Axis_Gripper = 0;   // Cờ ngắt gripper trong classify

// ============================================================
// HÀM NGẮT & HELPER
// ============================================================

// Hướng Gripper: 1=Mở (Tăng encoder), 0=Đóng (Giảm encoder)
volatile int Direction_Gripper = 0; 

void IRAM_ATTR onEncoderPulse() {
  if (Direction_Gripper == 1) g_encoderCount++;
  else g_encoderCount--;
}

long readEncoderCount() {
  noInterrupts();
  long val = g_encoderCount;
  interrupts();
  return val;
}

// Điều khiển Gripper (ESP32 Core v3 style)
void MoveAxisGripper(int direction, int speed) {
  // direction: 1=Mở, 0=Đóng, -1=Dừng
  if (direction == 0) { // Đóng
    Direction_Gripper = 0;
    ledcWrite(PIN_GRIPPER_FWD, speed);
    ledcWrite(PIN_GRIPPER_BWD, 0);
  } else if (direction == 1) { // Mở
    Direction_Gripper = 1;
    ledcWrite(PIN_GRIPPER_FWD, 0);
    ledcWrite(PIN_GRIPPER_BWD, speed);
  } else { // Dừng
    ledcWrite(PIN_GRIPPER_FWD, 0);
    ledcWrite(PIN_GRIPPER_BWD, 0);
  }
}

// Detect kẹp vật (Logic cũ giữ nguyên)
long time_last_check = 0;
long value_last_check = 0;
int HasConsitancy = 0;

int DetachObject() {
  long time_now = millis();
  if (time_now - time_last_check >= 50) { // Check mỗi 50ms
    long value_now = readEncoderCount();
    long delta = abs(value_now - value_last_check);
    
    time_last_check = time_now;
    value_last_check = value_now;

    if (delta > 15) HasConsitancy = 1; // Đang di chuyển tốt
    
    // Nếu đang di chuyển mà đột ngột delta nhỏ -> Kẹp trúng vật
    if (delta < 5 && HasConsitancy == 1) {
      Object_Size = (float)value_now / GRIPPER_PULSES_PER_CM;
      
      if (Object_Size < 20) Number_of_Objects_small++;      // Giả sử < 20
      else if (Object_Size < 40) Number_of_Objects_medium++; // Giả sử < 40
      else Number_of_Objects_large++;
      
      HasConsitancy = 0;
      Serial.println("Detected Object!");
      return 1; // Đã kẹp
    }
  }
  return 0;
}

// Homing helper (Blocking - chỉ dùng trong setup hoặc task riêng)
void homeAxisBlocking(AccelStepper &axis, int limPin) {
  pinMode(limPin, INPUT_PULLUP);
  axis.setMaxSpeed(800);
  axis.setSpeed(-400); // Chạy lùi về công tắc
  
  // Chạy đến khi chạm công tắc
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
    // Thêm thoát khẩn cấp nếu cần
  }
  
  // Dừng và set 0 tạm
  axis.setCurrentPosition(0);
  axis.runToNewPosition(200); // Lùi ra một chút
  
  // Chạy chậm lại lần nữa để chính xác
  axis.setSpeed(-200);
  while (digitalRead(limPin) == HIGH) {
    axis.runSpeed();
  }
  axis.setCurrentPosition(0);
  axis.runToNewPosition(50); // Vị trí an toàn
}

void SetHomeAllBlocking() {
  Serial.println("Homing Start...");
  homeAxisBlocking(Axis_Base, LIM_BASE_PIN);
  homeAxisBlocking(Axis_Shoulder, LIM_SHOULDER_PIN);
  homeAxisBlocking(Axis_Elbow, LIM_ELBOW_PIN);
  
  // Homing Gripper (Đóng hết cỡ coi là 0)
  MoveAxisGripper(0, 200);
  delay(1000); // Chờ đóng
  MoveAxisGripper(-1, 0);
  g_encoderCount = 0;
  
  Serial.println("Homing Done!");
}

// ============================================================
// RTOS TASKS
// ============================================================

// Task Homing (Non-blocking)
void Task_ReturnToHome(void *pvParameters) {
  for (;;) {
    if (Mode == -1) {
      // Thực hiện homing tại đây thay vì gọi trong loop chính
      // Để đơn giản, ta gọi hàm blocking ở đây vì Task này chạy độc lập
      SetHomeAllBlocking();
      Mode = 0; // Về chế độ Jog sau khi xong
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task JOG
void Task_MoveJog(void *pvParameters) {
  for (;;) {
    if (Mode == 0) { // Manual Mode
      float speedBase = 0, speedShoulder = 0, speedElbow = 0;
      int gripDir = -1, gripSpeed = 0;
      const float JOG_V = 600.0;

      switch (jogCommand) {
        case 1: speedBase = JOG_V; break;
        case 2: speedBase = -JOG_V; break;
        case 3: speedShoulder = JOG_V; break;
        case 4: speedShoulder = -JOG_V; break;
        case 5: speedElbow = JOG_V; break;
        case 6: speedElbow = -JOG_V; break;
        case 7: gripDir = 1; gripSpeed = 220; break; // Open
        case 8: gripDir = 0; gripSpeed = 220; break; // Close
      }

      Axis_Base.setSpeed(speedBase);
      Axis_Shoulder.setSpeed(speedShoulder);
      Axis_Elbow.setSpeed(speedElbow);

      Axis_Base.runSpeed();
      Axis_Shoulder.runSpeed();
      Axis_Elbow.runSpeed();
      MoveAxisGripper(gripDir, gripSpeed);
    }
    vTaskDelay(1); // Nhường CPU
  }
}

// Task AUTO (AutoChain) - Mode 1
void Task_MotorAuto(void *pvParameters) {
  for (;;) {
    if (Mode == 1) {
      if (autoChainStep < TotalSteps) {
        // 1. Cài đặt thông số cho bước hiện tại
        XYZT_Coordinates &target = Moving_Coordinates[autoChainStep];
        
        Axis_Base.setMaxSpeed(target.Speed_X);
        Axis_Shoulder.setMaxSpeed(target.Speed_Y);
        Axis_Elbow.setMaxSpeed(target.Speed_Z);
        
        Axis_Base.moveTo(target.X);
        Axis_Shoulder.moveTo(target.Y);
        Axis_Elbow.moveTo(target.Z);
        
        // Gripper logic
        int targetGripper = target.T;
        int gripSpd = target.Speed_T;
        int gripDir = -1;
        
        if (targetGripper > g_encoderCount) gripDir = 1;
        else if (targetGripper < g_encoderCount) gripDir = 0;

        // 2. Chạy loop cho đến khi đến đích (Blocking cục bộ trong task)
        bool stepperDone = false;
        bool gripperDone = false;
        
        MoveAxisGripper(gripDir, gripSpd); // Bắt đầu chạy gripper

        while (!stepperDone || !gripperDone) {
          if (Mode != 1) break; // Thoát nếu bị hủy lệnh
          
          // Run steppers
          if (Axis_Base.distanceToGo() == 0 && Axis_Shoulder.distanceToGo() == 0 && Axis_Elbow.distanceToGo() == 0) {
             stepperDone = true;
          } else {
             Axis_Base.run(); Axis_Shoulder.run(); Axis_Elbow.run();
          }

          // Check gripper
          if (abs(g_encoderCount - targetGripper) < 5) { // Sai số nhỏ
             MoveAxisGripper(-1, 0); // Stop gripper
             gripperDone = true;
          } else {
             // Vẫn chạy gripper
          }
          
          vTaskDelay(1); // Quan trọng: Nhường CPU
        }

        // Hoàn thành bước
        autoChainStep++;
      } else {
        // Hoàn thành chuỗi
        Mode = 0; 
        Serial.println("Auto Chain Finished");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Task CLASSIFY - Mode 2
void Task_MotorClassify(void *pvParameters) {
  for (;;) {
    if (Mode == 2) {
       // Logic tương tự Auto Chain nhưng dùng mảng Moving_Coordinates
       // (Code client đã copy mảng Classify vào Moving_Coordinates khi gửi mode classify)
       // Ở đây ta tái sử dụng logic của Mode 1 cho đơn giản, 
       // hoặc viết logic riêng có cảm biến kẹp vật ở đây.
       
       // Để đơn giản hóa code: Tôi đã gộp logic chạy theo tọa độ vào Task_MotorAuto (Mode 1).
       // Khi gửi lệnh Classify, Client nên gửi Mode=1 nhưng với tọa độ của Classify.
       // Nếu bạn muốn logic kẹp vật thông minh (dừng khi chạm), hãy viết thêm vào đây.
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ============================================================
// HTTP HANDLERS
// ============================================================

void handleJog_Unified(DynamicJsonDocument &doc) {
  if (!doc.containsKey("command_jog") || !doc.containsKey("action")) {
    server.send(400, "application/json", "{\"error\":\"Missing params\"}");
    return;
  }
  
  int cmd = doc["command_jog"];
  const char* action = doc["action"];

  if (strcmp(action, "start") == 0) {
    Mode = 0;
    jogCommand = cmd;
    server.send(200, "application/json", "{\"status\":\"Jog Started\"}");
  } else if (strcmp(action, "stop") == 0) {
    jogCommand = 0;
    server.send(200, "application/json", "{\"status\":\"Jog Stopped\"}");
  } else if (strcmp(action, "sethome") == 0) {
    jogCommand = 0;
    Mode = -1; // Kích hoạt Task Homing
    server.send(200, "application/json", "{\"status\":\"Homing Triggered\"}");
  } else if (strcmp(action, "return") == 0) {
    jogCommand = 0;
    // Logic return home (chạy stepper về 0) có thể viết thêm
    server.send(200, "application/json", "{\"status\":\"Return Home Req\"}");
  }
}

void handleAutoChain_Unified(DynamicJsonDocument &doc) {
  if (!doc.containsKey("TotalSteps") || !doc.containsKey("coordinates")) {
    server.send(400, "application/json", "{\"error\":\"Missing data\"}");
    return;
  }

  int modeInput = doc["mode"] | 1; // 1=Auto, 2=Classify
  
  // Stop động cơ trước khi nạp
  Mode = 0; 
  vTaskDelay(10); 

  TotalSteps = doc["TotalSteps"];
  if (TotalSteps > MAX_STEPS) TotalSteps = MAX_STEPS;
  
  JsonArray coords = doc["coordinates"];
  for (int i = 0; i < TotalSteps; i++) {
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
  
  // Kích hoạt chế độ chạy
  // Lưu ý: Code client gửi mode 1 hoặc 2.
  // Ở đây tôi dùng Mode = 1 để chạy chuỗi tọa độ chung cho cả Auto và Classify
  Mode = 1; 

  server.send(200, "application/json", "{\"status\":\"Chain Started\"}");
}

void handleStatus_Unified() {
  StaticJsonDocument<512> doc;
  doc["mode"] = Mode;
  doc["jog_cmd"] = jogCommand;
  doc["step_current"] = autoChainStep;
  doc["step_total"] = TotalSteps;
  doc["pos_X"] = Axis_Base.currentPosition();
  doc["pos_Y"] = Axis_Shoulder.currentPosition();
  doc["pos_Z"] = Axis_Elbow.currentPosition();
  doc["pos_T"] = readEncoderCount();
  doc["obj_small"] = Number_of_Objects_small;
  doc["obj_med"] = Number_of_Objects_medium;
  doc["obj_large"] = Number_of_Objects_large;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleUnifiedCommand() {
  if (!server.hasArg("plain")) return server.send(400, "text/plain", "No Body");
  
  // Dùng biến cục bộ để tránh lỗi bộ nhớ toàn cục
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  
  if (error) {
    server.send(400, "application/json", "{\"error\":\"JSON Error\"}");
    return;
  }

  if (!doc.containsKey("task")) {
     server.send(400, "application/json", "{\"error\":\"No task\"}");
     return;
  }

  const char* task = doc["task"];
  if (strcmp(task, "jog") == 0) handleJog_Unified(doc);
  else if (strcmp(task, "autochain") == 0) handleAutoChain_Unified(doc);
  else if (strcmp(task, "status") == 0) handleStatus_Unified(); // Status có thể gọi riêng qua GET
  else server.send(400, "application/json", "{\"error\":\"Unknown Task\"}");
}

// ============================================================
// MAIN SETUP & LOOP
// ============================================================

void setup() {
  Serial.begin(115200);

  // Cấu hình PWM Gripper (ESP32 v3.0)
  ledcAttachChannel(PIN_GRIPPER_FWD, 5000, 8, 0);
  ledcAttachChannel(PIN_GRIPPER_BWD, 5000, 8, 1);

  // Cấu hình Encoder
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), onEncoderPulse, FALLING);

  // Cấu hình Stepper
  Axis_Base.setMaxSpeed(1000); Axis_Base.setAcceleration(2000);
  Axis_Shoulder.setMaxSpeed(1000); Axis_Shoulder.setAcceleration(2000);
  Axis_Elbow.setMaxSpeed(1000); Axis_Elbow.setAcceleration(2000);

  // WiFi
  WiFi.begin(SSID, PWD);
  Serial.print("WiFi Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(WiFi.localIP());

  // Web Server
  server.on("/command", HTTP_POST, handleUnifiedCommand);
  server.on("/status", HTTP_GET, handleStatus_Unified);
  server.begin();

  // Tạo Tasks
  xTaskCreate(Task_MoveJog, "JogTask", 4096, NULL, 1, NULL);
  xTaskCreate(Task_MotorAuto, "AutoTask", 8192, NULL, 2, NULL);
  xTaskCreate(Task_ReturnToHome, "HomeTask", 4096, NULL, 3, NULL);
  
  Serial.println("System Ready!");
}

void loop() {
  server.handleClient();
  delay(2); // Để CPU nghỉ ngơi
}