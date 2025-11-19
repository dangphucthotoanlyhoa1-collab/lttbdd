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
  // Ở task Stop, cho mode = -3 để dừng tất cả các hoạt động.
  // Ở task JOG, cho mode = 0 và sử dụng trường "command_jog" để điều khiển
  // Ở task Autochain, cho mode = 1 và sử dụng trường "Coordinates_auto" để đưa dữ liệu vào XYZT_Coordinates Moving_Coordinates
  // Ở task Classify, cho mode = 2 và sử dụng trường "Coordinates_classify" để đưa dữ liệu vào XYZT_Coordinates Classify
  // Ở task Status, không cần trường nào khác ngoài "task", lấy trạng thái hiện tại của robot.
  // Ở trường TotalSteps, chỉ sử dụng trong Autochain để xác định số bước trong chuỗi, nó sẽ gán cho autoChainStep
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
// Function prototypes
void IRAM_ATTR onEncoderPulse();                                // intererupt handler for encoder
long readEncoderCount();                                        // read encoder count safely
void resetEncoderCount();                                    // reset encoder count
void MoveAxisGripper(int direction, int speed);                 // direction: 0=close, 1=open, -1=stop
int DetachObject();                                             // detect if object is gripped and measure size
int homeAxis(AccelStepper &axis, int limPin, const char *name); // homing for one axis
void SetHomeAll();                                              // homing for all axes
void ReturnToHome(void *parameter);                             // non-blocking return to home task
void MoveJog(void *parameter);                                  // JOG movement task
void motor_task_AUTO(void *parameter);                          // AUTO and classify movement task
void setup_task();                                              // initialize RTOS tasks
void setup_webserver();                                         // initialize web server
void handleUnifiedCommand();                                    // POST: handle all incoming commands
void UpdateCurrentPosition();                                   // GET: update current position

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // encoder priority access mutex

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
XYZT_Coordinates Classify[7];
XYZT_Coordinates CurrentPosition;

//========== Khai báo Web Server và JSON Document ======
WebServer server(80);
StaticJsonDocument<4096> jsonDocument;
// ====== Wi‑Fi credentials (edit to your network) ======
const char *SSID = "Thanh Gia 1";
const char *PWD = "ThanhgiA1931";

//========== Khai báo các đối tượng stepper motor ======
AccelStepper Axis_Base(AccelStepper::DRIVER, 26, 27);     // Step, Dir
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 32, 33); // Step, Dir
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 25, 21);    // Step, Dir
#define Axis_Gripper_forward_PIN 16                        // kẹp lại
#define Axis_Gripper_backward_PIN 4                       // mở ra
volatile int Direction_Gripper = 0;                       // Hướng kẹp vật: 0= đóng càng, 1= mở càng
// (XÓA) 2 biến tốc độ này, chúng ta sẽ đọc từ mảng
// int speedGripperForward = 0;
// int speedGripperBackward = 0;

//========== Khai báo các biến toàn cục ======

// ====== Chân công tắc hành trình ======
const int LIM_BASE_PIN = 13;     // chỉnh theo phần cứng
const int LIM_SHOULDER_PIN = 23; // chỉnh theo phần cứng
const int LIM_ELBOW_PIN = 19;    // chỉnh theo phần cứng

// ====== Các biến điều khiển robot (State Machine) ======
volatile int command_jog = 0;      // 0=STOP, 1=Base+, 2=Base-, 3=Shoulder+, 4=Shoulder-, 5=Elbow+, 6=Elbow-
const float JOG_SPEED = 600.0;     // Vận tốc cố định 600
volatile int Mode = -3;            // -3= Stop, -2= SetHome ,-1=ReturnHome, 0=jog, 1=chain, 2=classify
volatile int autoChainStep = 0;    // bước hiện tại trong chuỗi
volatile int autoClassifyStep = 0; // bước hiện tại trong phân loại
int TotalSteps = 0;                // Số bước chạy auto (sẽ được cập nhật khi nhận lệnh từ web)

//=======Biến điều khiển encoder gripper ======
const int ENCODER_PIN = 18; // Chân đọc encoder
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
int Flag_Object_size = 0; // 1: vật nhỏ , 2: vật vừa, 3: vật lớn

// Biến tạm cho hàm DetachObject
long time_now = 0;
long value_now = 0;
long time_last = 0;
long value_last = 0;
int HasConsitancy = 0;

// Biến cho Gripper ở Jog mode
int Axis_Gripper_setSpeed = 0;
int direction_Gripper_Jog = -1;

// Biến cho Gripper ở Auto mode
int Axis_Gripper_Speed = 0;
int Axis_Gripper_Next_Position = 0;

// Biến cho Classify mode
int Axis_Gripper_Stop = 0;
int Detach_object_flag = 0;
int Disable_Axis_Gripper = 0;
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
  portENTER_CRITICAL(&mux);             // TẮT ngắt
  count_copy = g_encoderCount; // Sao chép giá trị
  portEXIT_CRITICAL(&mux);                // BẬT ngắt lại ngay
  return count_copy;
}

void resetEncoderCount() {
  portENTER_CRITICAL(&mux);
  g_encoderCount = 0;
  portEXIT_CRITICAL(&mux);
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
        Flag_Object_size = 1;
      }
      else if (Object_Size < Object_Medium_size)
      {
        Number_of_Objects_medium++;
        Flag_Object_size = 2;
      }
      else
      {
        Number_of_Objects_large++;
        Flag_Object_size = 3;
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
    long Encoder_Position = readEncoderCount(); // đọc vị trí encoder gripper
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
        resetEncoderCount();    // reset encoder count
      }
      if (HomingBase && HomingShoulder && HomingElbow && HomingGripper)
      {
        Serial.println("Homing completed!");
        Mode = -3; // set to Stop after homing
        return;
      }
    }
  }
  return;
}

//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void ReturnToHome(void *parameter)
{
  while (1)
  {
    if (Mode == -1)
    {
      long Encoder_Position = readEncoderCount();
      if (Axis_Base.distanceToGo() == 0 &&
          Axis_Shoulder.distanceToGo() == 0 &&
          Axis_Elbow.distanceToGo() == 0 &&
          Encoder_Position == 0) // reset encoder count
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
      else
      {
        Mode = -3; // set to Stop after returning home
      }
    }
    else
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

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
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

//====== (SỬA LẠI HOÀN TOÀN) Task chạy chế độ AUTO (Đồng bộ Stepper + Gripper) ======
void motor_task_AUTO(void *parameter)
{
  while (1)
  {
    if (Mode == 1)
    { 
      long Encoder_Position = readEncoderCount();
      // Chỉ chạy khi ở chế độ AUTO
      // Chạy chuỗi tự động
      // Kiểm tra xem 3 trục đã dừng lại (hoàn thành bước) chưa
      if (Axis_Base.distanceToGo() == 0 &&
          Axis_Shoulder.distanceToGo() == 0 &&
          Axis_Elbow.distanceToGo() == 0 &&
          abs(Encoder_Position - Axis_Gripper_Next_Position) <= 5)
      {
        // và kẹp đã dừng{
        // Đã hoàn thành bước autoChainStep
        Serial.print("Hoan thanh buoc: ");
        Serial.println(autoChainStep);
        autoChainStep++; // Chuyển sang bước tiếp theo

        if (autoChainStep > TotalSteps)
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
      Encoder_Position = readEncoderCount();
      if (Axis_Gripper_Next_Position > Encoder_Position)
      {
        Direction_Gripper = 1; // mở càng
      }
      else if (Axis_Gripper_Next_Position < Encoder_Position)
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

      Encoder_Position = readEncoderCount();
      if (Encoder_Position != Axis_Gripper_Next_Position)
      {
        MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
      }

      else
      {
        MoveAxisGripper(-1, 0); // dừng càng
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
}

//===================== Task chạy chế độ CLASSIFY =========================
/*
Classify[0] = {X: vật, Y: vật, Z: vật, T: mở càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255} // sẽ cộng Y thêm 400 để tránh va chạm
Classify[1] = {X: vật, Y: vật, Z: vật, T: mở càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255}
Classify[2] = {X: vật, Y: vật, Z: vật, T: đóng càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255} // bước kẹp vật
Classify[3] = {X: vật, Y: vật, Z: vật, T: đóng càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque}  //cộng Y thêm 400 để tránh va chạm
Classify[4] = {X: vị trí thả nhỏ, Y: vị trí thả nhỏ, Z: vị trí thả nhỏ, T: đóng càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[5] = {X: vị trí thả vừa, Y: vị trí thả vừa, Z: vị trí thả vừa, T: đóng càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[6] = {X: vị trí thả lớn, Y: vị trí thả lớn, Z: vị trí thả lớn, T: đóng càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[7] = {X: thùng, Y: thùng, Z: thùng, T: mở càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255}
*/

void motor_task_Classify(void *parameter)
{
  int MoveToStep_7 = 0;

  while (1)
  {
    if (Mode == 2)
    {
      long Encoder_Position = readEncoderCount();
      if (Axis_Base.distanceToGo() == 0 &&
          Axis_Shoulder.distanceToGo() == 0 &&
          Axis_Elbow.distanceToGo() == 0 &&
          Axis_Gripper_Stop == 1)
      {
        autoClassifyStep++;
        Axis_Gripper_Stop = 0; // Chuyển sang bước tiếp theo
        if (MoveToStep_7 == 1)
        {
          autoClassifyStep = 7;
          MoveToStep_7 = 0;
        }
        if (autoClassifyStep == 4)
        {
          switch (Flag_Object_size)
          {
          case 1:
            autoClassifyStep = 4;
            MoveToStep_7 = 1;
            Flag_Object_size = 0;
            break;
          case 2:
            autoClassifyStep = 5;
            MoveToStep_7 = 1;
            Flag_Object_size = 0;
            break;
          case 3:
            autoClassifyStep = 6;
            Flag_Object_size = 0;
            break;
          default:
            break;
          }
        }
        if (autoClassifyStep == 7)
        {
          Disable_Axis_Gripper = 0;
          Axis_Gripper_Stop = 0;
          Detach_object_flag = 0;
        }
        if (autoClassifyStep > 7)
        {
          autoClassifyStep = 0;
          MoveToStep_7 = 0;
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

      Encoder_Position = readEncoderCount();
      if (Axis_Gripper_Next_Position > Encoder_Position)
      {
        Direction_Gripper = 1; // mở càng
      }
      else if (Axis_Gripper_Next_Position < Encoder_Position)
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
      if (autoClassifyStep == 2)
      {
        Detach_object_flag = DetachObject(); // lên 1 trong 1 chu kỳ quét
      }
      if (Disable_Axis_Gripper == 0)
      {
        if (Detach_object_flag == 0)
        {
          Encoder_Position = readEncoderCount();
          if (Encoder_Position != Axis_Gripper_Next_Position)
          {
            MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
          }
          if (abs(Encoder_Position - Axis_Gripper_Next_Position) <= 5)
          {
            MoveAxisGripper(-1, 0); // dừng càng
            Axis_Gripper_Stop = 1;
          }
        }
        else // đã kẹp vật
        {
          Axis_Gripper_Stop = 1;
          MoveAxisGripper(Direction_Gripper, torque); // giữ lực kẹp
          Disable_Axis_Gripper = 1;
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
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
  long encoderPos = readEncoderCount();
  CurrentPosition.T = encoderPos;
  CurrentPosition.Speed_X = Axis_Base.speed();
  CurrentPosition.Speed_Y = Axis_Shoulder.speed();
  CurrentPosition.Speed_Z = Axis_Elbow.speed();
}

void handleUnifiedCommand()
{
  if (!server.hasArg("plain"))
  {
    server.send(400, "application/json", "{\"error\":\"No data\"}");
    return;
  }

  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  String task = jsonDocument["task"];
  Serial.print("Web Command: ");
  Serial.println(task);

  // --- STOP ---
  if (task == "Stop")
  {
    Mode = -3; // Stop tất cả
    Axis_Base.stop();
    Axis_Shoulder.stop();
    Axis_Elbow.stop();
    MoveAxisGripper(-1, 0);
    server.send(200, "application/json", "{\"status\":\"Stopped\"}");
  }
  // --- JOG ---
  else if (task == "Jog")
  {
    Mode = 0;
    command_jog = jsonDocument["command_jog"];
    server.send(200, "application/json", "{\"status\":\"Jogging\"}");
  }
  // --- AUTO CHAIN ---
  else if (task == "Autochain")
  {
    Mode = -2; // Stop trước khi nạp
    TotalSteps = jsonDocument["TotalSteps"];
    JsonArray coords = jsonDocument["Coordinates_auto"];

    for (int i = 0; i < TotalSteps; i++)
    {
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
  else if (task == "Classify")
  {
    Mode = -3; // Stop trước khi nạp
    JsonArray coords = jsonDocument["Coordinates_classify"];

    for (int i = 0; i < 6; i++)
    {
      Classify[i].X = coords[i]["X"];
      Classify[i].Y = coords[i]["Y"];
      Classify[i].Z = coords[i]["Z"];
      Classify[i].T = coords[i]["T"];
      Classify[i].Speed_X = coords[i]["speed_X"];
      Classify[i].Speed_Y = coords[i]["speed_Y"];
      Classify[i].Speed_Z = coords[i]["speed_Z"];
      Classify[i].Speed_T = coords[i]["speed_T"];
    }
    Classify[0].Y = Classify[0].Y + 400;
    Classify[4].Y = Classify[4].Y + 400;
    Classify[5].Y = Classify[5].Y + 400;
    Classify[6].Y = Classify[6].Y + 400;
    autoClassifyStep = 0;
    Mode = 2; // Start Classify
    server.send(200, "application/json", "{\"status\":\"Classify Started\"}");
  }
  else if (task == "SetHome")
  {
    Mode = -2; // Start Homing
    SetHomeAll();
    server.send(200, "application/json", "{\"status\":\"Homing Started\"}");
  }
  else if (task == "ReturnHome")
  {
    Mode = -1; // Start Return Home
    server.send(200, "application/json", "{\"status\":\"Returning Home\"}");
  }
  else
  {
    server.send(400, "application/json", "{\"error\":\"Unknown task\"}");
  }
}

// Setup Web Server
void setup_webserver()
{
  server.on("/command", HTTP_POST, handleUnifiedCommand);
  // server.on("/status", HTTP_GET, handleStatus_Unified);
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