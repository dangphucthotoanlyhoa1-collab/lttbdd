#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
// Không xác định
extern String task;
extern volatile int jogCommand;
extern long readEncoderCount();

// khởi tạo biến nút nhấn kết nối wifi
extern unsigned long buttonTimer;
extern bool buttonActive;
// ==========================================
// 1. ĐỊNH NGHĨA CẤU TRÚC & HẰNG SỐ (MACRO)
// ==========================================
#define AP_SSID_NAME "ROBOT_SETUP_WIFI"
#define AP_IP_ADDRESS IPAddress(192, 168, 4, 1)
#define FIREBASE_HOST "robot-d7d8a-default-rtdb.firebaseio.com"
#define FIREBASE_KEY "AIzaSyDL4nqw7i4Ymx5Rxn2mnUcYYaRDJbWg93g"
#define USER_EMAIL "ok642005@gmail.com"
#define USER_PASS "dinhkhoi12340"
#define ROBOT_NODE "/tt_robot_xyz"
#define DATABASE_URL "https://robot-d7d8a-default-rtdb.firebaseio.com"

// Cấu trúc tọa độ
typedef struct
{
    float x;
    float y;
    float z;
    int t; // 0=Stepper, 1=Kẹp, 2=Nhả
    float speedX;
    float speedY;
    float speedZ;
    int speedT;
} XYZT_Coordinates;

// Định nghĩa Chân (Pin Definitions) - Chuyển sang #define cho gọn
#define Axis_Gripper_forward_PIN 16
#define Axis_Gripper_backward_PIN 4
#define LIM_BASE_PIN 13
#define LIM_SHOULDER_PIN 23
#define LIM_ELBOW_PIN 25
#define ENCODER_PIN 18

// Thông số kỹ thuật
#define JOG_SPEED 400.0
#define GRIPPER_PULSES_PER_CM 0.1
#define GRIPPER_CHECK_INTERVAL 50
#define GRIPPER_STALL_THRESHOLD 5
#define GRIPPER_MOVE_THRESHOLD 15


// ==========================================
// 2. EXTERN CÁC ĐỐI TƯỢNG PHẦN CỨNG
// ==========================================
extern WebServer serverAP;
// extern StaticJsonDocument<4096> jsonDocument;

extern AccelStepper Axis_Base;
extern AccelStepper Axis_Shoulder;
extern AccelStepper Axis_Elbow;

// ==========================================
// 3. EXTERN CÁC BIẾN TOÀN CỤC (LOGIC)
// ==========================================

// Mảng dữ liệu
extern XYZT_Coordinates Coordinates_auto[50];
extern XYZT_Coordinates Classify[7];
extern XYZT_Coordinates CurrentPosition;

// Biến điều khiển Gripper
extern volatile int Direction_Gripper;
extern volatile long g_encoderCount;
extern volatile int torque;
extern int Axis_Gripper_setSpeed;
extern int direction_Gripper_Jog;
extern int Axis_Gripper_Speed;
extern int Axis_Gripper_Next_Position;
extern int Axis_Gripper_Stop;
extern int Detach_object_flag;
extern int Disable_Axis_Gripper;

// Biến điều khiển Robot (State Machine)
extern volatile int command_jog;
extern volatile int Mode;
extern volatile int autoChainStep;
extern volatile int autoClassifyStep;
extern int TotalSteps;

// Biến phân loại vật
extern int Object_Small_size;
extern int Object_Large_size;
extern int Object_Medium_size;
extern int Number_of_Objects_small;
extern int Number_of_Objects_medium;
extern int Number_of_Objects_large;
extern volatile float Object_Size;
extern volatile int Flag_Object_size;
extern float Size_of_objects[100];
extern int Counter_Size_of_objects;

// Biến tạm (Helper variables)
extern long time_now;
extern long value_now;
extern long time_last;
extern long value_last;
extern int HasConsitancy;

// Biến cờ (Flag) fix lỗi Race Condition (từ câu hỏi trước)
extern volatile bool isNewStepLoaded;

// Mutex (Nếu cần dùng ở file khác)
extern portMUX_TYPE mux;

#endif // Kết thúc CONFIG_H