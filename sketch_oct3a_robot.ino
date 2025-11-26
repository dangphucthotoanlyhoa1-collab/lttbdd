#include "config.h"
#include "wifiAP.h"
#include "FirebaseLogic.h"

// Function prototypes
void IRAM_ATTR onEncoderPulse();                                // intererupt handler for encoder
long readEncoderCount();                                        // read encoder count safely
void resetEncoderCount();                                       // reset encoder count
void MoveAxisGripper(int direction, int speed);                 // direction: 0=close, 1=open, -1=stop
int DetachObject();                                             // detect if object is gripped and measure size
int homeAxis(AccelStepper &axis, int limPin, const char *name); // homing for one axis
void SetHomeAll(void *parameter);                                              // homing for all axes
void ReturnToHome(void *parameter);                             // non-blocking return to home task
void MoveJog(void *parameter);                                  // JOG movement task
void Robot_task_AUTO(void *parameter);                          // AUTO and classify movement task
void Robot_task_Classify(void *parameter);                      // CLASSIFY movement task
void setup_task();                                              // initialize RTOS tasks
void setup_webserver();                                         // initialize web server
void handleUnifiedCommand();                                    // POST: handle all incoming commands
void handleGetCurrentPosition();                                // GET: update current position

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // encoder priority access mutex

//========== Khai báo cấu trúc tọa độ XYZT và Tốc độ chuyển động ======
XYZT_Coordinates Coordinates_auto[50];
XYZT_Coordinates Classify[7];
XYZT_Coordinates CurrentPosition;

// StaticJsonDocument<4096> jsonDocument;
//  khởi tạo các đối tượng AccelStepper cho các trục
AccelStepper Axis_Base(AccelStepper::DRIVER, 21, 19);
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 32, 33);
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 26, 27);

volatile int Direction_Gripper = 0;

// Các biến toàn cục
volatile int command_jog = 0;
volatile int Mode = -3;
volatile int autoChainStep = 0;
volatile int autoClassifyStep = 0;
int TotalSteps = 0;

// Biến điều khiển Gripper
volatile long g_encoderCount = 0;
volatile int torque = 140;

// Biến phân loại vật
int Object_Small_size = 0;
int Object_Large_size = 2;
int Object_Medium_size = 4;
int Number_of_Objects_small = 0;
int Number_of_Objects_medium = 0;
int Number_of_Objects_large = 0;
volatile float Object_Size = 0.0;
volatile int Flag_Object_size = 0;
float Size_of_objects[100];
int Counter_Size_of_objects = 0;

// Biến tạm cho logic phân loại vật
long time_now = 0;
long value_now = 0;
long time_last = 0;
long value_last = 0;
int HasConsitancy = 0;

// Biến tạm cho Gripper
int Axis_Gripper_setSpeed = 0;
int direction_Gripper_Jog = -1;
int Axis_Gripper_Speed = 0;
int Axis_Gripper_Next_Position = 0;
int Axis_Gripper_Stop = 0;
int Detach_object_flag = 0;
int Disable_Axis_Gripper = 0;

// Khai báo biến nút nhấn kết nối wifi
unsigned long buttonTimer = 0;
bool buttonActive = false;

void handleSerialInput()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0)
            return;

        char commandChar = input.charAt(0);
        int value = input.substring(1).toInt();

        // --- M: CHUYỂN MODE THỦ CÔNG ---
        if (commandChar == 'M' || commandChar == 'm')
        {
            Mode = value;
            command_jog = 0; // Reset jog
            Serial.print(">>> Mode changed to: ");
            Serial.println(Mode);
            if (Mode == -3)
            {
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                MoveAxisGripper(-1, 0);
                Serial.println(">>> EMERGENCY STOP");
            }
            if (Mode == -2)
            {
                Serial.println(">>> Homing...");
               
            }
        }

        // --- J: LỆNH JOG ---
        else if (commandChar == 'J' || commandChar == 'j')
        {
            if (Mode == 0)
            {
                command_jog = value;
                Serial.print(">>> Jogging: ");
                Serial.println(command_jog);
            }
            else
            {
                Serial.println("!!! Error: Must be in Mode 0 (Jog) first (Type 'M0')");
            }
        }

        // --- A: TEST AUTO MODE (Chạy thử chuỗi 2 điểm) ---
        else if (commandChar == 'A' || commandChar == 'a')
        {
            Serial.println(">>> LOADING DEMO DATA FOR AUTOCHAIN...");

            // 1. Giả lập dữ liệu tọa độ (2 bước)
            TotalSteps = 2;

            // Bước 1: Vươn ra vị trí 1000, Mở càng
            Coordinates_auto[0].x = 300;
            Coordinates_auto[0].speedX = 600;
            Coordinates_auto[0].y = 300;
            Coordinates_auto[0].speedY = 600;
            Coordinates_auto[0].z = 500;
            Coordinates_auto[0].speedZ = 600;
            Coordinates_auto[0].t = 0;
            Coordinates_auto[0].speedT = 255; // 1=Mở

            // Bước 2: Về 0, Đóng càng
            Coordinates_auto[1].x = 0;
            Coordinates_auto[1].speedX = 400;
            Coordinates_auto[1].y = 0;
            Coordinates_auto[1].speedY = 400;
            Coordinates_auto[1].z = 0;
            Coordinates_auto[1].speedZ = 400;
            Coordinates_auto[1].t = 0;
            Coordinates_auto[1].speedT = 255; // 0=Đóng

            //  Thiết lập biến chạy
            autoChainStep = 0;
            Mode = -2; // Stop trước khi chạy
            delay(100);
            Mode = 1; // Kích hoạt Mode Auto
            Serial.println(">>> AUTOCHAIN STARTED (2 Steps Demo)");
        }

        // --- C: TEST CLASSIFY MODE (Chạy thử quy trình phân loại) ---
        else if (commandChar == 'C' || commandChar == 'c')
        {
            Serial.println(">>> LOADING DEMO DATA FOR CLASSIFY...");

            // Nạp dữ liệu giả cho 8 bước Classify (Giống trong JSON mẫu)
            // Lưu ý: Bạn hãy sửa các số 1000, 500... thành tọa độ thực tế robot của bạn để test an toàn

            // Bước 0: Vị trí chờ/nhìn
            Classify[0] = {0, 0, 0, 1, 800, 800, 800, 255};

            // Bước 1: Tiếp cận vật
            Classify[1] = {1000, 500, 200, 1, 800, 800, 800, 255};

            // Bước 2: Kẹp vật (T=0, speed=255 để kẹp nhanh)
            Classify[2] = {1000, 500, 200, 0, 800, 800, 800, 255};

            // Bước 3: Nhấc lên (Giữ lực kẹp = torque)
            Classify[3] = {1000, 500, 0, 0, 800, 800, 800, torque};

            // Bước 4: Thả vật NHỎ (Ví dụ: X=2000)
            Classify[4] = {2000, 500, 0, 0, 800, 800, 800, torque};

            // Bước 5: Thả vật VỪA
            Classify[5] = {3000, 500, 0, 0, 800, 800, 800, torque};

            // Bước 6: Thả vật LỚN
            Classify[6] = {4000, 500, 0, 0, 800, 800, 800, torque};

            // Bước 7: Về vị trí an toàn, mở càng
            Classify[7] = {0, 0, 0, 1, 800, 800, 800, 255};

            // Xử lý Logic bù tọa độ Y giống như trong HTTP Handler
            Classify[0].y += 400;
            Classify[4].y += 400;
            Classify[5].y += 400;
            Classify[6].y += 400;

            // Reset biến chạy
            autoClassifyStep = 0;
            Disable_Axis_Gripper = 0;
            Axis_Gripper_Stop = 0;
            Detach_object_flag = 0;

            Mode = -2;
            delay(100);
            Mode = 2; // Kích hoạt Mode Classify
            Serial.println(">>> CLASSIFY STARTED (Demo Sequence)");
        }

        // --- ?: HELP ---
        else if (commandChar == '?')
        {
            Serial.println("\n--- MENU TEST SERIAL ---");
            Serial.println(" [A] : Test AUTO Mode (Chay 2 diem mau)");
            Serial.println(" [C] : Test CLASSIFY Mode (Nap data gia lap)");
            Serial.println(" [M0]: Ve che do JOG");
            Serial.println("    -> J1/J2 (Base), J3/J4 (Shoulder)...");
            Serial.println(" [M-2]: Set Home All");
            Serial.println(" [M-3]: STOP ALL");
        }
    }
}

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
    portENTER_CRITICAL(&mux);    // TẮT ngắt
    count_copy = g_encoderCount; // Sao chép giá trị
    portEXIT_CRITICAL(&mux);     // BẬT ngắt lại ngay
    return count_copy;
}

void resetEncoderCount()
{
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
    {                                            // Dừng
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
            if (Mode == 2)
            {
                Size_of_objects[Counter_Size_of_objects] = Object_Size;
                Counter_Size_of_objects++;
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
#define HOME_SPEED_FAST    800.0
#define HOME_SPEED_SLOW    200.0
#define HOME_SPEED_RETREAT 400.0
#define HOME_RETREAT_DIST  400   // Bước lùi
#define LIMIT_ACTIVE_LEVEL 1
//===== Hàm trở về vị trí home (NON-BLOCKING) ======
void SetHomeAll(void *parameter) 
{
    // --- BIẾN TRẠNG THÁI CỤC BỘ (STATIC để giữ giá trị qua các vòng lặp) ---
    
    // 0: Idle, 1: Fast, 2: Stopping1, 3: Retreat, 4: Slow, 5: Stopping2, 6: Done
    static int state_Elbow = 0;
    static int state_Shoulder = 0;
    static int state_Base = 0;

    // Biến chống nhiễu (Debounce)
    static unsigned long debounce_timer_Elbow = 0;
    static int last_read_Elbow = !LIMIT_ACTIVE_LEVEL;
    
    static unsigned long debounce_timer_Shoulder = 0;
    static int last_read_Shoulder = !LIMIT_ACTIVE_LEVEL;

    static unsigned long debounce_timer_Base = 0;
    static int last_read_Base = !LIMIT_ACTIVE_LEVEL;

    // Cài đặt gia tốc
    Axis_Base.setAcceleration(5000);
    Axis_Shoulder.setAcceleration(5000);
    Axis_Elbow.setAcceleration(5000);

    for (;;) 
    {
        // Chỉ chạy khi Mode = -2
        if (Mode == -2) 
        {
            // ============================================================
            // TRỤC 1: ELBOW (Ưu tiên Home trước để nâng tay lên)
            // ============================================================
            
            // Bắt đầu kích hoạt
            if (state_Elbow == 0) {
                Axis_Elbow.setMaxSpeed(HOME_SPEED_FAST);
                Axis_Elbow.moveTo(-1000000); // Chạy về hướng âm vô cùng
                state_Elbow = 1; // Chuyển sang chạy nhanh
                Serial.println("ELBOW: Start Homing Fast...");
            }

            // Giai đoạn 1: Chạy nhanh tìm công tắc
            if (state_Elbow == 1) {
                // --- Logic đọc nút nhấn chống rung ---
                int reading = digitalRead(LIM_ELBOW_PIN);
                if (reading != last_read_Elbow) debounce_timer_Elbow = millis();
                last_read_Elbow = reading;
                
                bool isPressed = ((millis() - debounce_timer_Elbow) > 50) && (reading == LIMIT_ACTIVE_LEVEL);
                // -------------------------------------

                if (isPressed) {
                    Axis_Elbow.stop(); // Ra lệnh dừng
                    state_Elbow = 2;   // Chờ dừng hẳn
                    Serial.println("ELBOW: Touched! Stopping...");
                } else {
                    Axis_Elbow.run(); // Tiếp tục chạy
                }
            }

            // Giai đoạn 2: Chờ động cơ dừng hẳn (do quán tính)
            if (state_Elbow == 2) {
                if (!Axis_Elbow.isRunning()) {
                    Axis_Elbow.setCurrentPosition(0); // Tạm set 0
                    Axis_Elbow.setMaxSpeed(HOME_SPEED_RETREAT);
                    Axis_Elbow.moveTo(HOME_RETREAT_DIST); // Lùi ra
                    state_Elbow = 3;
                    Serial.println("ELBOW: Stopped. Retreating...");
                } else {
                    Axis_Elbow.run();
                }
            }

            // Giai đoạn 3: Lùi ra
            if (state_Elbow == 3) {
                if (Axis_Elbow.distanceToGo() == 0) {
                    Axis_Elbow.setMaxSpeed(HOME_SPEED_SLOW);
                    Axis_Elbow.moveTo(-1000000); // Dò lại chậm
                    state_Elbow = 4;
                    Serial.println("ELBOW: Retreat Done. Slow Approach...");
                } else {
                    Axis_Elbow.run();
                }
            }

            // Giai đoạn 4: Chạy chậm dò chính xác
            if (state_Elbow == 4) {
                int reading = digitalRead(LIM_ELBOW_PIN);
                if (reading != last_read_Elbow) debounce_timer_Elbow = millis();
                last_read_Elbow = reading;
                bool isPressed = ((millis() - debounce_timer_Elbow) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                if (isPressed) {
                    Axis_Elbow.stop();
                    state_Elbow = 5; // Chờ dừng lần cuối
                    Serial.println("ELBOW: Touched Precision! Stopping...");
                } else {
                    Axis_Elbow.run();
                }
            }

            // Giai đoạn 5: Chốt điểm 0
            if (state_Elbow == 5) {
                if (!Axis_Elbow.isRunning()) {
                    Axis_Elbow.setCurrentPosition(0); // GỐC TỌA ĐỘ CHUẨN
                    Axis_Elbow.moveTo(0);
                    state_Elbow = 6; // Xong
                    Serial.println("ELBOW: HOMED ✅");
                } else {
                    Axis_Elbow.run();
                }
            }

            // ============================================================
            // TRỤC 2: SHOULDER (Chỉ chạy khi Elbow đã xong: state_Elbow == 6)
            // ============================================================
            
            if (state_Elbow == 6) 
            {
                if (state_Shoulder == 0) {
                    Axis_Shoulder.setMaxSpeed(HOME_SPEED_FAST);
                    Axis_Shoulder.moveTo(-1000000);
                    state_Shoulder = 1;
                    Serial.println("SHOULDER: Start Homing Fast...");
                }

                if (state_Shoulder == 1) {
                    int reading = digitalRead(LIM_SHOULDER_PIN);
                    if (reading != last_read_Shoulder) debounce_timer_Shoulder = millis();
                    last_read_Shoulder = reading;
                    bool isPressed = ((millis() - debounce_timer_Shoulder) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed) {
                        Axis_Shoulder.stop();
                        state_Shoulder = 2;
                        Serial.println("SHOULDER: Touched! Stopping...");
                    } else {
                        Axis_Shoulder.run();
                    }
                }

                if (state_Shoulder == 2) {
                    if (!Axis_Shoulder.isRunning()) {
                        Axis_Shoulder.setCurrentPosition(0);
                        Axis_Shoulder.setMaxSpeed(HOME_SPEED_RETREAT);
                        Axis_Shoulder.moveTo(HOME_RETREAT_DIST);
                        state_Shoulder = 3;
                    } else {
                        Axis_Shoulder.run();
                    }
                }

                if (state_Shoulder == 3) {
                    if (Axis_Shoulder.distanceToGo() == 0) {
                        Axis_Shoulder.setMaxSpeed(HOME_SPEED_SLOW);
                        Axis_Shoulder.moveTo(-1000000);
                        state_Shoulder = 4;
                    } else {
                        Axis_Shoulder.run();
                    }
                }

                if (state_Shoulder == 4) {
                    int reading = digitalRead(LIM_SHOULDER_PIN);
                    if (reading != last_read_Shoulder) debounce_timer_Shoulder = millis();
                    last_read_Shoulder = reading;
                    bool isPressed = ((millis() - debounce_timer_Shoulder) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed) {
                        Axis_Shoulder.stop();
                        state_Shoulder = 5;
                    } else {
                        Axis_Shoulder.run();
                    }
                }

                if (state_Shoulder == 5) {
                    if (!Axis_Shoulder.isRunning()) {
                        Axis_Shoulder.setCurrentPosition(0);
                        Axis_Shoulder.moveTo(0);
                        state_Shoulder = 6; // Xong
                        Serial.println("SHOULDER: HOMED ✅");
                    } else {
                        Axis_Shoulder.run();
                    }
                }
            }

            // ============================================================
            // TRỤC 3: BASE (Chỉ chạy khi Shoulder đã xong: state_Shoulder == 6)
            // ============================================================

            if (state_Shoulder == 6) 
            {
                if (state_Base == 0) {
                    Axis_Base.setMaxSpeed(HOME_SPEED_FAST);
                    Axis_Base.moveTo(-1000000);
                    state_Base = 1;
                    Serial.println("BASE: Start Homing Fast...");
                }

                if (state_Base == 1) {
                    int reading = digitalRead(LIM_BASE_PIN);
                    if (reading != last_read_Base) debounce_timer_Base = millis();
                    last_read_Base = reading;
                    bool isPressed = ((millis() - debounce_timer_Base) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed) {
                        Axis_Base.stop();
                        state_Base = 2;
                        Serial.println("BASE: Touched! Stopping...");
                    } else {
                        Axis_Base.run();
                    }
                }

                if (state_Base == 2) {
                    if (!Axis_Base.isRunning()) {
                        Axis_Base.setCurrentPosition(0);
                        Axis_Base.setMaxSpeed(HOME_SPEED_RETREAT);
                        Axis_Base.moveTo(HOME_RETREAT_DIST);
                        state_Base = 3;
                    } else {
                        Axis_Base.run();
                    }
                }

                if (state_Base == 3) {
                    if (Axis_Base.distanceToGo() == 0) {
                        Axis_Base.setMaxSpeed(HOME_SPEED_SLOW);
                        Axis_Base.moveTo(-1000000);
                        state_Base = 4;
                    } else {
                        Axis_Base.run();
                    }
                }

                if (state_Base == 4) {
                    int reading = digitalRead(LIM_BASE_PIN);
                    if (reading != last_read_Base) debounce_timer_Base = millis();
                    last_read_Base = reading;
                    bool isPressed = ((millis() - debounce_timer_Base) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed) {
                        Axis_Base.stop();
                        state_Base = 5;
                    } else {
                        Axis_Base.run();
                    }
                }

                if (state_Base == 5) {
                    if (!Axis_Base.isRunning()) {
                        Axis_Base.setCurrentPosition(0);
                        Axis_Base.moveTo(0);
                        state_Base = 6; // Xong
                        Serial.println("BASE: HOMED! ");
                    } else {
                        Axis_Base.run();
                    }
                }
            }

            // ============================================================
            // KIỂM TRA HOÀN TẤT
            // ============================================================
            if (state_Elbow == 6 && state_Shoulder == 6 && state_Base == 6) {
                Serial.println(" ALL DONE! ");
                
                // Reset biến trạng thái về 0 để lần sau chạy tiếp được
                state_Elbow = 0;
                state_Shoulder = 0;
                state_Base = 0;
                
                // Thoát mode Homing
                Mode = -3; 
                
                vTaskDelay(1000 / portTICK_PERIOD_MS); // Nghỉ tí
            }

            // Delay cực nhỏ để không chiếm 100% CPU (Watchdog)
            // Vì ta dùng .run() (có gia tốc) nên 1ms delay là chấp nhận được
            vTaskDelay(1 / portTICK_PERIOD_MS); 
        }
        else 
        {
            // Nếu không phải mode Homing, ngủ dài
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void ReturnToHome(void *parameter)
{
    // Biến cờ để biết đã cài đặt thông số ban đầu chưa
    // static giúp biến này nhớ giá trị qua các vòng lặp
    static bool isHomingStarted = false; 

    while (1)
    {
        if (Mode == -1)
        {
            // --- GIAI ĐOẠN 1: SETUP (CHỈ CHẠY 1 LẦN ĐẦU TIÊN) ---
            if (!isHomingStarted) 
            {
                Serial.println(">> START RETURN HOME...");
                
                // Cài đặt tốc độ
                Axis_Base.setMaxSpeed(600);
                Axis_Shoulder.setMaxSpeed(600);
                Axis_Elbow.setMaxSpeed(600);

                // Ra lệnh về 0
                Axis_Base.moveTo(0);
                Axis_Shoulder.moveTo(0);
                Axis_Elbow.moveTo(0);
                
                MoveAxisGripper(0, 220); // Đóng càng

                isHomingStarted = true; // Đánh dấu đã setup xong
            }

            // --- GIAI ĐOẠN 2: RUN (CHẠY LIÊN TỤC) ---
            // Hàm run() phải được gọi liên tục để sinh xung
            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();

            // --- GIAI ĐOẠN 3: KIỂM TRA HOÀN TẤT ---
            // Kiểm tra xem tất cả đã về đích chưa
            if (Axis_Base.distanceToGo() == 0 &&
                Axis_Shoulder.distanceToGo() == 0 &&
                Axis_Elbow.distanceToGo() == 0) 
            {
                Serial.println("✅ RETURN HOME COMPLETE!");
                
                Mode = -3;             // Chuyển sang chế độ dừng/nghỉ
                isHomingStarted = false; // Reset cờ để lần sau dùng lại được
                
                // Reset Encoder về 0 nếu cần thiết (để đồng bộ)
                // resetEncoder(0); 
            }

            // Delay cực nhỏ để nhường CPU nhưng vẫn đảm bảo motor chạy mượt
            // Với AccelStepper, delay 1ms là chấp nhận được
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            // Nếu không phải mode Return Home, reset cờ và ngủ dài
            isHomingStarted = false; 
            vTaskDelay(100 / portTICK_PERIOD_MS);
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
            Axis_Base.setMaxSpeed(JOG_SPEED);
            Axis_Shoulder.setMaxSpeed(JOG_SPEED);
            Axis_Elbow.setMaxSpeed(JOG_SPEED);

            Axis_Gripper_setSpeed = 0;
            direction_Gripper_Jog = -1;
            switch (command_jog)
            {
            case 1:
                Axis_Base.moveTo(10000000); // quay trái
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Base Left");
                break;
            case 2:
                Axis_Base.moveTo(-10000000); // quay phải
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Base Right");
                break;
            case 3:
                Axis_Shoulder.moveTo(10000000); // nâng lên
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Shoulder Up");
                break;
            case 4:
                Axis_Shoulder.moveTo(-10000000); // hạ xuống
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Shoulder Down");
                break;
            case 5:
                Axis_Elbow.moveTo(10000000); // vươn ra
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                // Serial.println("Jog Elbow Extend");
                break; // vươn ra
            case 6:
                Axis_Elbow.moveTo(-10000000); // thu lại
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                // Serial.println("Jog Elbow Retract");
                break; // thu lại
            case 7:
                direction_Gripper_Jog = 1;
                Axis_Gripper_setSpeed = 220;
                // Serial.println("Jog Gripper Open");
                break; // mở càng
            case 8:
                direction_Gripper_Jog = 0;
                Axis_Gripper_setSpeed = 220;
                // Serial.println("Jog Gripper Close");
                break; // đóng càng
            default:
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                break;
            }

            MoveAxisGripper(direction_Gripper_Jog, Axis_Gripper_setSpeed);
            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
}

void Robot_task_AUTO(void *parameter)
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
                // Đã hoàn thành bước autoChainStep
                Serial.print("Hoan thanh buoc: ");
                Serial.println(autoChainStep);
                autoChainStep++; // Chuyển sang bước tiếp theo

                if (autoChainStep >= TotalSteps)
                {
                    autoChainStep = 0;
                }
            }
            // Đặt tốc độ MỚI và mục tiêu MỚI nếu bước thay đổi
            Axis_Base.setMaxSpeed(Coordinates_auto[autoChainStep].speedX);
            Axis_Shoulder.setMaxSpeed(Coordinates_auto[autoChainStep].speedY);
            Axis_Elbow.setMaxSpeed(Coordinates_auto[autoChainStep].speedZ);
            Axis_Gripper_Speed = Coordinates_auto[autoChainStep].speedT;

            Axis_Base.moveTo(Coordinates_auto[autoChainStep].x);
            Axis_Shoulder.moveTo(Coordinates_auto[autoChainStep].y);
            Axis_Elbow.moveTo(Coordinates_auto[autoChainStep].z);
            Axis_Gripper_Next_Position = Coordinates_auto[autoChainStep].t;

            // Đặt mục tiêu mới
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

            Encoder_Position = readEncoderCount();
            if (Encoder_Position != Axis_Gripper_Next_Position)
            {
                MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
            }

            else
            {
                MoveAxisGripper(-1, 0); // dừng càng
            }
            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
}

//===================== Task chạy chế độ CLASSIFY =========================
/*
Classify[0] = {X: vật, Y: vật, Z: vật, T: 30, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255} // sẽ cộng Y thêm 400 để tránh va chạm
Classify[1] = {X: vật, Y: vật, Z: vật, T: 30, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255}
Classify[2] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255} // bước kẹp vật
Classify[3] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque}  //cộng Y thêm 400 để tránh va chạm
Classify[4] = {X: vị trí thả nhỏ, Y: vị trí thả nhỏ, Z: vị trí thả nhỏ, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[5] = {X: vị trí thả vừa, Y: vị trí thả vừa, Z: vị trí thả vừa, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[6] = {X: vị trí thả lớn, Y: vị trí thả lớn, Z: vị trí thả lớn, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:torque} //cộng Y thêm 400 để tránh va chạm
Classify[7] = {X: thùng, Y: thùng, Z: thùng, T: mở càng, speed_X:800, speed_Y:800, speed_Z:800, speed_T:255}
*/

void Robot_task_Classify(void *parameter)
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
                if (autoClassifyStep > 7)
                {
                    autoChainStep = 0;
                }
            }

            Axis_Base.setMaxSpeed(Classify[autoClassifyStep].speedX);
            Axis_Shoulder.setMaxSpeed(Classify[autoClassifyStep].speedY);
            Axis_Elbow.setMaxSpeed(Classify[autoClassifyStep].speedZ);
            Axis_Gripper_Speed = Classify[autoClassifyStep].speedT;

            Axis_Base.moveTo(Classify[autoClassifyStep].x);
            Axis_Shoulder.moveTo(Classify[autoClassifyStep].y);
            Axis_Elbow.moveTo(Classify[autoClassifyStep].z);
            Axis_Gripper_Next_Position = Classify[autoClassifyStep].t;

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
            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
}

// ====== Khởi tạo các task ======
void setup_task()
{
    xTaskCreate(MoveJog, "MoveJog", 2048, NULL, 3, NULL);
    xTaskCreate(Robot_task_AUTO, "MotorRunAUTO", 4096, NULL, 2, NULL);
    xTaskCreate(ReturnToHome, "ReturnToHome", 4096, NULL, 1, NULL);
    xTaskCreate(Robot_task_Classify, "MotorRunClassify", 4096, NULL, 2, NULL);
    xTaskCreate(SetHomeAll, "SetHomeAll", 4096, NULL, 1, NULL);
}

void setup()
{
    Serial.begin(115200);
    pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);    // cấu hình chân công tắc hành trình trục khuỷu
    pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP); // cấu hình chân công tắc hành trình trục nâng
    pinMode(LIM_BASE_PIN, INPUT_PULLUP);     // cấu hình chân công tắc hành trình trục quay
    //l298n driver input
    ledcAttachChannel(Axis_Gripper_forward_PIN, 5000, 8, 0);  // mở càng
    ledcAttachChannel(Axis_Gripper_backward_PIN, 5000, 8, 1); // đóng càng
    // cấu hình encoder
    pinMode(ENCODER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), onEncoderPulse, FALLING);
    Serial.println("Encoder interrupt ready!");
    //Cấu hình trục
    Axis_Base.setMaxSpeed(2000);
    Axis_Base.setAcceleration(4000);
    Axis_Shoulder.setMaxSpeed(2000);
    Axis_Shoulder.setAcceleration(4000);
    Axis_Elbow.setMaxSpeed(2000);
    Axis_Elbow.setAcceleration(4000);
    Axis_Base.moveTo(0);
    Axis_Shoulder.moveTo(0);
    Axis_Elbow.moveTo(0);

    // Serial.println("Robot chua san sang.");
    pinMode(0, INPUT_PULLUP);
    setupWifiAP();
    if (isWifiConnected())
    {
        delay(2000);
        setupFirebase();
        Serial.println("connect");
    }
    else
    {
        Serial.println("disconect");
    }
    setup_task();

    Serial.println("Robot da san sang.");
    Serial.println("Chuyen sang che do AUTO (0).");
}

// ====== Hàm loop ======
// Khai báo biến đếm thời gian (để bên ngoài hoặc static bên trong)
unsigned long lastDebugTime = 0;
const int DEBUG_INTERVAL = 500; // Thời gian in lại (ms) - Chỉnh số này nếu muốn nhanh/chậm hơn

void loop()
{

    loopWifiAP();
    if (isWifiConnected())
    {
        loopFirebase();
    }
    if (digitalRead(0) == LOW)
    {
        // Nếu mới bắt đầu nhấn
        if (!buttonActive)
        {
            buttonActive = true;
            buttonTimer = millis(); // Bắt đầu tính giờ
            Serial.println(">> Dang giu nut... (Giu 3s de Reset WiFi)");
        }

        // Nếu đang giữ nút, kiểm tra xem đã đủ 3 giây chưa
        if ((millis() - buttonTimer > 3000))
        {
            Serial.println(">> DA DU 3 GIAY! TIEN HANH RESET...");
            resetWifiConfig();    // Gọi hàm reset bên WifiAP.cpp
            buttonActive = false; // Reset trạng thái
        }
    }
    else
    {
        // Nếu nhả nút ra
        if (buttonActive)
        {
            buttonActive = false;
            Serial.println(">> Da nha nut (Huy Reset).");
        }
    }

    // server.handleClient(); // Xử lý client requests
    handleSerialInput(); // Xử lý lệnh bàn phím

    // --- 2. Phần Debug (Chỉ chạy mỗi 500ms) ---
    if (millis() - lastDebugTime > DEBUG_INTERVAL)
    {
        lastDebugTime = millis(); // Cập nhật thời gian

        // In gọn trên 1 dòng để dễ quan sát biến động
        Serial.print("POS [Base]: ");
        Serial.print(Axis_Base.currentPosition());

        Serial.print(" | [Shoulder]: ");
        Serial.print(Axis_Shoulder.currentPosition());

        Serial.print(" | [Elbow]: ");
        Serial.print(Axis_Elbow.currentPosition()); // (Đã sửa nhãn từ 'base' thành 'Elbow')

        Serial.print(" | [Gripper]: ");
        // Nên dùng hàm readEncoderCount() để đảm bảo an toàn ngắt thay vì đọc trực tiếp biến
        Serial.println(readEncoderCount());
    }
}