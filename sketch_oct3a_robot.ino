#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>

//========== Khai báo cấu trúc tọa độ XYZ và Tốc độ chuyển động ======

//Định nghĩa các hàm trong file



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
int maxSteps = 0;

typedef struct Axis
{
  float Current_Position;
  float Target_Position;
  float Velocity;
};

//========== Khai báo các đối tượng stepper motor ======
AccelStepper Axis_Base(AccelStepper::DRIVER, 26, 27); // Step, Dir
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 14, 12); // Step, Dir
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 25, 33); // Step, Dir

//========== Khai báo các biến toàn cục ======


// Thông tin kết nối Wifi
const char *SSID = "PHONG_B34";
const char *PWD = "viettel1";

// Khai báo đối tượng webserver
WebServer server(80);

// Khai báo tài nguyên JSON
StaticJsonDocument<250> jsonDocument;
char buffer[250];

// ====== Khai báo chân ======
const int LDR_PIN = 34;   // Quang trở
const int POT1_PIN = 35;  // Biến trở 1
const int POT2_PIN = 32;  // Biến trở 2
const int LED_R = 5;   // LED đỏ
const int LED_G = 18;  // LED xanh lá
const int LED_B = 19;  // LED xanh dương

// ====== Chân công tắc hành trình ======
const int LIM_BASE_PIN     = 15; // chỉnh theo phần cứng
const int LIM_SHOULDER_PIN = 13; // chỉnh theo phần cứng
const int LIM_ELBOW_PIN    = 23; // chỉnh theo phần cứng

// ====== Các biến điều khiển jog ======
volatile int robotMode = 0; // 0=AUTO (dùng run()), 1=JOG (dùng runSpeed())
volatile int jogCommand = 0; // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0; // Vận tốc cố định 600

//====== Các biến cho chạy chuỗi tự động
volatile int autoChainMode = 0; // 0=STOP, 1=RUNNING
volatile int autoChainStep = 0; // Đang ở bước thứ mấy
int maxSteps = 1000; // (Ví dụ, bạn phải định nghĩa maxSteps)

//====== Các biến dữ liệu ============
float data1 = 0;  // quang trở
float data2 = 0;  // biến trở 1
float data3 = 0;  // biến trở 2

uint8_t control1 ;  // LED đỏ
uint8_t control2 ;  // LED xanh lá
uint8_t control3 ;  // LED xanh dương


// ====== Các hàm tạo và thêm JSON ======
void create_json(const char *tag, float value) {
  jsonDocument.clear();
  jsonDocument["type"] = tag;
  jsonDocument["value"] = value;
  serializeJson(jsonDocument, buffer);
}

void add_json_object(const char *tag, float value) {
  JsonObject obj = jsonDocument.createNestedObject();
  obj["type"] = tag;
  obj["value"] = value;
}

// ====== Các hàm đọc dữ liệu ======
void getData1() {
  Serial.println("Get data 1");
  create_json("Data1", data1);
  server.send(200, "application/json", buffer);
}

void getData2() {
  Serial.println("Get data 2");
  create_json("Data2", data2);
  server.send(200, "application/json", buffer);
}

void getData3() {
  Serial.println("Get data 3");
  create_json("Data3", data3);
  server.send(200, "application/json", buffer);
}

void getAllData() {
  Serial.println("Get all data");
  jsonDocument.clear();
  add_json_object("Data1", data1);
  add_json_object("Data2", data2);
  add_json_object("Data3", data3);
  serializeJson(jsonDocument, buffer);
  server.send(200, "application/json", buffer);
}

// ====== Hàm xử lý POST từ client ======
void handlePost() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
    return;
  }

  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  // In ra để kiểm tra
  Serial.println("Received POST:");
  Serial.println(body);

  uint8_t control1 = jsonDocument["control1"];
  uint8_t control2 = jsonDocument["control2"];
  uint8_t control3 = jsonDocument["control3"];

  Serial.println("control1 = " + String(control1));
  Serial.println("control2 = " + String(control2));
  Serial.println("control3 = " + String(control3));

  //Điều khiển led
  ledcWrite(LED_R, control1);
  ledcWrite(LED_G, control2);
  ledcWrite(LED_B, control3);
  server.send(200, "application/json", "{}");
}

// ====== (MỚI) Hàm xử lý JOG từ client (có khóa chéo) ======
void handleJog() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }

  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);
  Serial.println(body); // In ra để debug

  // Lấy dữ liệu từ JSON
  String axis = jsonDocument["axis"];
  String direction = jsonDocument["direction"];

  int newCommand = 0; // Mặc định là STOP (thả nút)

  // 1. Chuyển đổi lệnh text sang số
  if (axis == "base") {
    if (direction == "positive") newCommand = 1;
    else if (direction == "negative") newCommand = 2;
  } else if (axis == "shoulder") {
    if (direction == "positive") newCommand = 3;
    else if (direction == "negative") newCommand = 4;
  } else if (axis == "elbow") {
    if (direction == "positive") newCommand = 5;
    else if (direction == "negative") newCommand = 6;
  }
  // Nếu axis == "stop", newCommand sẽ giữ là 0

  // 2. LOGIC KHÓA CHÉO VÀ CHUYỂN CHẾ ĐỘ
  
  // Trường hợp 1: Thả nút (lệnh DỪNG)
  if (newCommand == 0) {
    jogCommand = 0; // Dừng chạy JOG
    robotMode = 0;  // Trả robot về chế độ AUTO (run())
    Serial.println("JOG STOP -> Chuyen sang che do AUTO");
  } 
  // Trường hợp 2: Nhấn nút (lệnh CHẠY)
  else {
    // KHÓA CHÉO: Chỉ chấp nhận lệnh chạy MỚI nếu robot đang DỪNG
    // (robotMode == 0) -> Đang ở chế độ AUTO (chắc chắn đang dừng JOG)
    // (jogCommand == 0) -> Đang ở JOG, nhưng không chạy trục nào
    if (robotMode == 0 || jogCommand == 0) {
      robotMode = 1; // Chuyển sang chế độ JOG (runSpeed())
      jogCommand = newCommand; // Gán lệnh chạy
      Serial.print("JOG START -> Chay lenh: "); Serial.println(jogCommand);
    }
    // Trường hợp 3: else (robotMode=1 và jogCommand != 0)
    // Robot đang JOG và đang chạy, BỎ QUA lệnh mới -> Đã khóa chéo
  }

  server.send(200, "application/json", "{}");
}

// Hàm đọc dữ liệu
void readSensors() {
  // Đọc ADC (0..4095) và quy đổi sang mV (3.3V max)
  data1 = analogReadMilliVolts(LDR_PIN) / 1000.0;
  data2 = analogReadMilliVolts(POT1_PIN) / 1000.0;
  data3 = analogReadMilliVolts(POT2_PIN) / 1000.0;
}

// ====== Tác vụ LED nhấp nháy ======
void blinking_led(void *parameter) {
  pinMode(2, OUTPUT);
  for (;;) {
    digitalWrite(2, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(2, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ====== Cấu hình server routing ======
void setup_routing() {
  server.on("/data1", getData1);
  server.on("/data2", getData2);
  server.on("/data3", getData3);
  server.on("/data", getAllData);
  server.on("/control", HTTP_POST, handlePost);
  server.on("/jog", HTTP_POST, handleJog); // <-- thêm đây
  server.begin();
}

void read_all_data(void *parameter) {
  for (;;) {
    readSensors();                          // cập nhật dữ liệu
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1 giây
  }
}
// ====== Khởi tạo các task ======
void setup_task() {
  xTaskCreate(read_all_data, "Read all data", 1000, NULL, 1, NULL);
  xTaskCreate(blinking_led, "Blinking led", 1000, NULL, 1, NULL);

  // tạo task cho jog (priority cao hơn nhỏ tùy chỉnh nếu cần)
  xTaskCreate(MoveJog, "MoveJog", 2000, (void*)(intptr_t)0, 2, NULL);
  xTaskCreate(MoveAbsoluteChain, "MoveAbsoluteChain", 4000, (void*)(intptr_t)0, 1, NULL);
}
  

void setupPWM() {
  ledcAttachChannel(LED_R, 5000, 8, 0);
  ledcAttachChannel(LED_G, 5000, 8, 1);
  ledcAttachChannel(LED_B, 5000, 8, 2);

}
//================================================================================//
//================================================================================//
//==== các tác vụ điều khiển robot ====
//Blocking: tuần tự, chờ hoàn thành mới chạy lệnh tiếp theo
// Non-blocking: không chờ, chạy song song với lệnh khác
// cặp đôi axis.setSpeed(0); và axis.runSpeed(); dùng để dừng trục, blocking (tuần tự).

// Hàm homing cho 1 trục (approach nhanh, rút lui, approach chậm để bù sai số)
void SetHome(AccelStepper &axis, const char *name) {

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

// Hàm homing cho cả 3 trục (sequential để ổn định)
void SetHomeAll() {
  Serial.println("Starting homing sequence...");
  homeAxis(Axis_Base, "Axis Base");
  homeAxis(Axis_Shoulder,"Axis Shoulder");
  homeAxis(Axis_Elbow, "Axis Elbow");
  Serial.println("Homing completed!");
}

//===== Hàm trở về vị trí home ======
void ReturnToHome() {
  Axis_Base.moveTo(0);
  Axis_Shoulder.moveTo(0);
  Axis_Elbow.moveTo(0);

  while (Axis_Base.distanceToGo() != 0 || Axis_Shoulder.distanceToGo() != 0 || Axis_Elbow.distanceToGo() != 0) {
    Axis_Base.run();
    Axis_Shoulder.run();
    Axis_Elbow.run();
  }

  Serial.println("Returned to home position!");
}


// ====== Move Jog Task ======
void MoveJog(void *parameter) {
  if (robotMode == 1) {
    
  switch (jogCommand)
  {
  case 0: // STOP
    Axis_Base.setSpeed(0);
    Axis_Shoulder.setSpeed(0);
    Axis_Elbow.setSpeed(0);
    Axis_Base.runSpeed();
    Axis_Shoulder.runSpeed();
    Axis_Elbow.runSpeed();
    break;

  case 1: // Base +
    Axis_Base.setSpeed(JOG_SPEED);
    Axis_Base.runSpeed(); 
    break;

  case 2: // Base -
    Axis_Base.setSpeed(-JOG_SPEED);
    Axis_Base.runSpeed(); 
    break;

  case 3: // Shoulder +
    Axis_Shoulder.setSpeed(JOG_SPEED);
    Axis_Shoulder.runSpeed(); 
    break;

  case 4: // Shoulder -
    Axis_Shoulder.setSpeed(-JOG_SPEED);
    Axis_Shoulder.runSpeed(); 
    break;
    
  case 5: // Elbow +
    Axis_Elbow.setSpeed(JOG_SPEED);
    Axis_Elbow.runSpeed(); 
    break;

  case 6: // Elbow -
    Axis_Elbow.setSpeed(-JOG_SPEED);
    Axis_Elbow.runSpeed(); 
    break; 

  default:    Axis_Base.setSpeed(0);
              Axis_Shoulder.setSpeed(0);
              Axis_Elbow.setSpeed(0);
              Axis_Base.runSpeed();
              Axis_Shoulder.runSpeed();
              Axis_Elbow.runSpeed();
    break;
  }
}
  vTaskDelay(1 / portTICK_PERIOD_MS); //cập nhật lệnh mỗi 1ms
}

//================================================================================//
//======Move absolute to target position (non-blocking) ======//
int MoveAbsolute (long target_X, long target_Y, long target_Z, int Target_T, float speed_X, float speed_Y, float speed_Z, int speed_T) {
  // Cài đặt tọa độ mục tiêu
  Axis_Base.moveTo(target_X);
  Axis_Shoulder.moveTo(target_Y);
  Axis_Elbow.moveTo(target_Z);
  // Cài đặt tốc độ tối đa
  Axis_Base.setMaxSpeed(speed_X);
  Axis_Shoulder.setMaxSpeed(speed_Y);
  Axis_Elbow.setMaxSpeed(speed_Z);

  while (Axis_Base.distanceToGo() != 0 || Axis_Shoulder.distanceToGo() != 0 || Axis_Elbow.distanceToGo() != 0)
  {
    Axis_Base.run();
    Axis_Shoulder.run();
    Axis_Elbow.run();
  }
  if (Axis_Base.distanceToGo() != 0 || Axis_Shoulder.distanceToGo() != 0 || Axis_Elbow.distanceToGo() != 0){
      Serial.println("Move to target position completed!");
      return 1;
  }
  return 0;
}

void MoveAbsoluteChain(void *parameter) {
  int EndOfMove = 0;
  if (jogCommand == 0){
  for (int i=0; i < maxSteps; i++) {
    EndOfMove = MoveAbsolute(
      Moving_Coordinates[i].X,
      Moving_Coordinates[i].Y,
      Moving_Coordinates[i].Z,
      Moving_Coordinates[i].T,
      Moving_Coordinates[i].Speed_X,
      Moving_Coordinates[i].Speed_Y,
      Moving_Coordinates[i].Speed_Z,    
      Moving_Coordinates[i].Speed_T
    );

    if (EndOfMove == 1) {
      Serial.print("Step ");
      Serial.print(i);
      Serial.println(" completed.");
    } 
    else {
      Serial.print("Error moving to step ");
      Serial.println(i);
      break;
    }
  }
}
  vTaskDelay(1 / portTICK_PERIOD_MS); // Giảm tải CPU
}



// ====== Hàm setup ======
void setup() {
  //=====Cấu hình chân công tắc hành trình=====
  pinMode(LIM_ELBOW_PIN, INPUT_PULLUP); 
  pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP);
  pinMode(LIM_BASE_PIN, INPUT_PULLUP); 

  //=====Cài đặt thông số động cơ bước=====
  Axis_Base.setMaxSpeed(1000);
  Axis_Base.setAcceleration(2000);
  Axis_Shoulder.setMaxSpeed(1000);
  Axis_Shoulder.setAcceleration(2000);
  Axis_Elbow.setMaxSpeed(1000);
  Axis_Elbow.setAcceleration(2000);

  //=====Kết nối Wi-Fi=====
  Serial.begin(115200);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(SSID, PWD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  //=====Cấu hình PWM cho LED=====
  setupPWM();

  // ===== Thực hiện homing trước khi khởi tạo task =====
  homeAll();

  //=====Khởi tạo các task RTOS=====
  setup_task();
  setup_routing();
}

// ====== Hàm loop ======
void loop() {
  server.handleClient();
}
