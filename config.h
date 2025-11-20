#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <AccelStepper.h>

// ================= CẤU HÌNH WIFI & FIREBASE =================
#define AP_SSID_NAME "ROBOT_SETUP_WIFI"
#define AP_IP_ADDRESS IPAddress(192, 168, 4, 1)

#define FIREBASE_HOST "robot-d7d8a.firebaseapp.com"
#define FIREBASE_KEY  "AIzaSyDL4nqw7i4Ymx5Rxn2mnUcYYaRDJbWg93g"
#define USER_EMAIL    "ok642005@gmail.com"
#define USER_PASS     "dinhkhoi12340"
#define ROBOT_NODE    "/tt_robot_xyz"
#define DATABASE_URL  "https://robot-d7d8a-default-rtdb.firebaseio.com"
// ================= CẤU HÌNH PHẦN CỨNG (PINOUT) =================
const int LIM_BASE_PIN = 15;
const int LIM_SHOULDER_PIN = 13;
const int LIM_ELBOW_PIN = 23;
const int ENCODER_PIN = 35;
#define GRIPPER_FWD_PIN 3
#define GRIPPER_BWD_PIN 4

// ================= BIẾN TOÀN CỤC (EXTERN) =================
// Dùng extern để các file khác có thể truy cập biến này mà không cần khai báo lại
extern AccelStepper Axis_Base;
extern AccelStepper Axis_Shoulder;
extern AccelStepper Axis_Elbow;

extern volatile int jogCommand;
extern volatile int Mode; // -2=Stop, -1=Homing, 0=Jog, 1=Chain, 2=Classify
extern volatile int autoChainMode;
extern int TotalStep;
extern volatile float Object_Size;
extern int Number_of_Objects_small;
extern int Number_of_Objects_medium;
extern int Number_of_Objects_large;

extern long readEncoderCount(); // Hàm đọc encoder từ file chính

// Struct tọa độ (như code bạn của bạn)
typedef struct {
  float X; float Y; float Z; int T;
  float Speed_X; float Speed_Y; float Speed_Z; int Speed_T;
} XYZT_Coordinates;

extern XYZT_Coordinates Moving_Coordinates[1000];
extern XYZT_Coordinates Classify[1000];

#endif