#include "config.h"
#include "wifiAP.h"
#include "FirebaseLogic.h"

// Function prototypes
void IRAM_ATTR onEncoderPulse();                // intererupt handler for encoder
long readEncoderCount();                        // read encoder count safely
void resetEncoderCount();                       // reset encoder count
void MoveAxisGripper(int direction, int speed); // direction: 0=close, 1=open, -1=stop
int DetachObject();                             // detect if object is gripped and measure size
void SetHomeAll(void *parameter);               // homing for all axes
void ReturnToHome(void *parameter);             // non-blocking return to home task
void MoveJog(void *parameter);                  // JOG movement task
void Robot_task_AUTO(void *parameter);          // AUTO and classify movement task
void Robot_task_Classify(void *parameter);      // CLASSIFY movement task
void setup_task();                              // initialize RTOS tasks
void setup_webserver();                         // initialize web server
void handleUnifiedCommand();                    // POST: handle all incoming commands
void handleGetCurrentPosition();                // GET: update current position

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // encoder priority access mutex

//========== Khai báo cấu trúc tọa độ XYZT và Tốc độ chuyển động ======
XYZT_Coordinates Coordinates_auto[50];
XYZT_Coordinates Classify[8];
XYZT_Coordinates CurrentPosition;

// StaticJsonDocument<4096> jsonDocument;
//  khởi tạo các đối tượng AccelStepper cho các trục
AccelStepper Axis_Base(AccelStepper::DRIVER, 21, 19);     // Pin STEP=21, DIR=19
AccelStepper Axis_Shoulder(AccelStepper::DRIVER, 33, 32); // Pin STEP=33, DIR=32
AccelStepper Axis_Elbow(AccelStepper::DRIVER, 26, 27);    // Pin STEP=26, DIR=27

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
int Object_Small_size = 28.5;
int Object_Large_size = 32.0; // unused
int Object_Medium_size = 32.0;
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

// biến cục bộ file sketch_oct3a_robot.ino
const float MM_PER_PULSE = 0.139;
// const float SIZE_SMALL_LIMIT  = 28.5;
// const float SIZE_MEDIUM_LIMIT = 32.0;

int step_state = 0;
long target_pos = 0;
long start_step_pos = 0;
unsigned long time_step_start = 0;

int SPEED_NORMAL = 10;
int SPEED_CRAWL = 10;

float Kp = 2.5;
float Ki = 0.01;
float Kd = 10.0;
#define PID_MIN_PWM 10
#define PID_MAX_PWM 100
#define PID_TOLERANCE 2 // Sai số cho phép (+/- 2 xung) khi lùi ra
#define PID_SAMPLE_TIME 10
const int STEP_PULSES = 5;

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
            Coordinates_auto[0].t = 800;
            Coordinates_auto[0].speedT = 10; // 1=Mở

            // Bước 2: Về 0, Đóng càng
            Coordinates_auto[1].x = 0;
            Coordinates_auto[1].speedX = 400;
            Coordinates_auto[1].y = 0;
            Coordinates_auto[1].speedY = 400;
            Coordinates_auto[1].z = 0;
            Coordinates_auto[1].speedZ = 400;
            Coordinates_auto[1].t = 0;
            Coordinates_auto[1].speedT = 10; // 0=Đóng

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
            Classify[0] = {1000, 0, 1200, 0, 800, 800, 800, 0};

            // Bước 1: Tiếp cận vật
            Classify[1] = {1000, 0, 1200, 0, 800, 800, 800, 0};

            // Bước 2: Kẹp vật (T=0, speed=255 để kẹp nhanh)
            Classify[2] = {1000, 0, 1200, 0, 800, 800, 800, 0};

            // Bước 3: Nhấc lên (Giữ lực kẹp = torque)
            Classify[3] = {1000, 0, 0, 0, 800, 800, 800, 0};

            // Bước 4: Thả vật NHỎ (Ví dụ: X=2000)
            Classify[4] = {2000, 0, 1000, 0, 800, 800, 800, 0};

            // Bước 5: Thả vật VỪA
            Classify[5] = {3000, 0, 800, 0, 800, 800, 800, 0};

            // Bước 6: Thả vật LỚN
            Classify[6] = {4000, 0, 600, 0, 800, 800, 800, 0};

            // Bước 7: Về vị trí an toàn, mở càng
            Classify[7] = {0, 0, 0, 0, 800, 800, 800, 0};

            // Xử lý Logic bù tọa độ Y giống như trong HTTP Handler
            Classify[0].y += 400;
            Classify[3].y += 400;
            Classify[4].y += 400;
            Classify[5].y += 400;
            Classify[6].y += 400;

            // Reset biến chạy
            autoClassifyStep = 0;
            Disable_Axis_Gripper = 0;
            Axis_Gripper_Stop = 0;
            Detach_object_flag = 0;

            Mode = -3;
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
    portENTER_CRITICAL_ISR(&mux);
    int a = digitalRead(ENCODER_PIN_A);
    int b = digitalRead(ENCODER_PIN_B);
    if (a == b)
        g_encoderCount++;
    else
        g_encoderCount--;
    portEXIT_CRITICAL_ISR(&mux);
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
    if (speed > 1023)
        speed = 1023;
    if (speed < 0)
        speed = 0;
    if (direction == 0)
    { // đóng càng
        Direction_Gripper = 0;
        ledcWrite(Axis_Gripper_forward_PIN, speed);
        ledcWrite(Axis_Gripper_backward_PIN, 0);
    }
    else if (direction == 1)
    { // mở càng
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

int move_state = 0; // Biến trạng thái riêng cho hàm di chuyển
float pid_integral = 0;
float pid_prev_error = 0;
unsigned long pid_last_time = 0;
unsigned long pid_stable_start = 0;
bool pid_is_stable = false;
#define HYBRID_APPROACH_SPEED 200 // Tốc độ lao tới (Nhanh)
#define HYBRID_SWITCH_DIST 40     // Cách đích bao nhiêu xung thì bắt đầu thả trôi vào PID?
                                  // (Số càng lớn thì PID càng sớm can thiệp, số càng nhỏ thì vọt lố càng xa)

int Task_MoveAbsolute_Hybrid(long target)
{
    long current = readEncoderCount();
    unsigned long now = millis();

    switch (move_state)
    {
    // --- GIAI ĐOẠN 0: KHỞI TẠO ---
    case 0:
        Serial.printf(">> HYBRID: Moving to %ld...\n", target);
        move_state = 1;
        return 0;

    // --- GIAI ĐOẠN 1: LAO TỚI (OPEN LOOP) ---
    case 1:
    {
        long dist = target - current;
        // Nếu còn xa -> Chạy nhanh
        if (abs(dist) > HYBRID_SWITCH_DIST)
        {
            if (dist > 0)
                MoveAxisGripper(1, HYBRID_APPROACH_SPEED);
            else
                MoveAxisGripper(0, HYBRID_APPROACH_SPEED);
            return 0;
        }
        // Nếu đã gần -> Chuyển sang PID
        else
        {
            pid_integral = 0;
            pid_prev_error = 0;
            pid_last_time = now;
            pid_is_stable = false;

            move_state = 2; // Chuyển sang giai đoạn 2
            return 0;
        }
    }

    // --- GIAI ĐOẠN 2: PID TINH CHỈNH ---
    case 2:
    {
        if (now - pid_last_time < PID_SAMPLE_TIME)
            return 0;

        float dt = (now - pid_last_time) / 1000.0;
        pid_last_time = now;

        long error = target - current;

        // --- KIỂM TRA ỔN ĐỊNH ---
        if (abs(error) <= PID_TOLERANCE)
        {
            if (!pid_is_stable)
            {
                pid_stable_start = now;
                pid_is_stable = true;
            }
            else
            {
                // Nếu đã giữ yên vị trí trong 200ms -> THÀNH CÔNG
                if (now - pid_stable_start > 200)
                {
                    MoveAxisGripper(-1, 0); // Dừng motor
                    Serial.printf(">> DONE. Target: %ld | Final: %ld\n", target, current);
                    return 1; // ==> TRẢ VỀ 1: ĐÃ XONG
                }
            }
        }
        else
        {
            pid_is_stable = false;
        }

        // --- TÍNH TOÁN PID ---
        float P = Kp * error;

        pid_integral += error * dt;
        if (pid_integral > 500)
            pid_integral = 500;
        if (pid_integral < -500)
            pid_integral = -500;
        float I = Ki * pid_integral;

        float derivative = (error - pid_prev_error) / dt;
        float D = Kd * derivative;
        pid_prev_error = error;

        float output = P + I + D;

        // --- ĐIỀU KHIỂN MOTOR (ĐÃ SỬA ĐỂ CHỐNG RUNG) ---
        int pwm_final = 0;

        // [QUAN TRỌNG] VÙNG CHẾT (DEADBAND):
        // Nếu sai số chỉ còn 0 hoặc 1 xung -> NGẮT ĐIỆN LUÔN
        // Để ma sát tự giữ động cơ, tránh việc PID cố chỉnh qua lại gây rung
        if (abs(error) <= 1)
        {
            pwm_final = 0;
        }
        else
        {
            // Logic bù ma sát
            int min_pwm_dynamic = PID_MIN_PWM;

            // Khi vào vùng rất gần (dưới 15 xung), giảm lực bù ma sát xuống thấp
            // để tránh bị "đá" vọt lố
            if (abs(error) < 15)
                min_pwm_dynamic = 60;

            if (output > 0)
            {
                pwm_final = (int)output + min_pwm_dynamic;
                if (pwm_final > PID_MAX_PWM)
                    pwm_final = PID_MAX_PWM;
                MoveAxisGripper(1, pwm_final);
            }
            else
            {
                pwm_final = (int)output - min_pwm_dynamic;
                if (pwm_final < -PID_MAX_PWM)
                    pwm_final = -PID_MAX_PWM;
                MoveAxisGripper(0, abs(pwm_final));
            }
        }

        // Nếu PWM = 0 thì gọi StopMotor để ngắt cầu H
        if (pwm_final == 0)
            MoveAxisGripper(-1, 0);

        return 0; // Vẫn đang chạy PID
    }
    }
    return 0;
}
int k = 0;
int state_home = 0;
int Task_Homing_Gripper()
{
    long current = readEncoderCount();
    unsigned long now = millis();

    switch (state_home)
    {
    // ====================================================
    // GIAI ĐOẠN 1: ĐÓNG CÀNG TỪNG BƯỚC ĐỂ TÌM ĐIỂM KẸT
    // ====================================================
    case 0: // KHỞI TẠO BƯỚC MỚI (Di chuyển 20 xung về phía đóng)
        start_step_pos = current;
        target_pos = current - STEP_PULSES;

        MoveAxisGripper(0, 10); // 0 = Close (Chạy chậm dò vật)
        time_step_start = millis();
        state_home = 1;
        break;

    case 1: // GIÁM SÁT KẸT (STALL DETECTION)
        // A. Kiểm tra đã đi đủ 20 xung chưa?
        if (abs(current - start_step_pos) >= STEP_PULSES)
        {
            Serial.printf(">>Step %d: %ld ms\n", k, now - time_step_start);
            state_home = 0; // Chưa kẹt -> Lặp lại bước mới
            k++;            // Tăng biến đếm an toàn (để tránh kẹt giả lúc đầu)
        }
        // B. Kiểm tra thời gian (STALL DETECT)
        // Nếu quá 80ms mà chưa đi được 20 xung -> Đã kẹt
        else if ((millis() - time_step_start > 20) && k > 10)
        {
            MoveAxisGripper(-1, 0); // StopMotor
            Serial.println(">> HOMING: Hard Stop Detected!");
            state_home = 2; // Chuyển sang bước lùi ra
        }
        break;

    // ====================================================
    // GIAI ĐOẠN 2: LÙI RA 40 XUNG BẰNG HÀM HYBRID
    // ====================================================
    case 2:
        k = 0;
        target_pos = current + 40; // Mục tiêu: Lùi ra dương 40 xung
        // --- QUAN TRỌNG: RESET TRẠNG THÁI HÀM HYBRID ---
        move_state = 0;
        // -----------------------------------------------
        Serial.printf(">> HOMING: Hybrid Backoff to %ld\n", target_pos);
        state_home = 3;
        break;

    case 3: // THỰC HIỆN LÙI
        // Gọi hàm Hybrid và chờ nó trả về 1 (Xong)
        if (Task_MoveAbsolute_Hybrid(target_pos) == 1)
        {

            // Đã đến đích lùi và dừng ổn định
            MoveAxisGripper(-1, 0); // StopMotor
            delay(300);             // Chờ một chút cho cơ khí thư giãn hoàn toàn

            // CÀI ĐẶT GỐC TỌA ĐỘ
            resetEncoderCount();

            Serial.println(" HOMING COMPLETE: Zero Set (Precise).");

            // Reset toàn bộ trạng thái về IDLE
            state_home = 0;
            return 1;
            // currentMode = MODE_IDLE;
        }
        break;
    }
    return 0;
}

int i = 0;                    // Biến đếm vòng lặp cho Stall Detection
unsigned long wait_timer = 0; // Biến lưu mốc thời gian chờ
// float Object_Size = 0;
int Task_Classify()
{
    long current = readEncoderCount();
    unsigned long now = millis();

    switch (step_state)
    {
    // ====================================================
    // GIAI ĐOẠN 1: MỞ RA 400 XUNG (DÙNG HYBRID PID)
    // ====================================================
    case 0:
        if (Task_Homing_Gripper() == 1)
        {
            target_pos = 400;
            move_state = 0; // Reset trạng thái hàm di chuyển

            Serial.println(">> CLASS: Opening 400 pulses...");
            step_state = 1;
            return 0;
        }
        break;
    case 1:
        // Gọi hàm di chuyển.
        // Nếu hàm trả về 1 (Đã đến đích), TA KHÔNG DỪNG LẠI
        // Mà chuyển sang chế độ "GIỮ VỊ TRÍ" (Active Hold)
        Serial.println("In case 2");
        if (Task_MoveAbsolute_Hybrid(target_pos) == 1)
        {
            Serial.println(">> CLASS: Arrived 400. HOLDING POSITION 3s...");
            Serial.println("Please put object to gripper!");

            // Lưu thời điểm bắt đầu chờ
            wait_timer = now;

            // Chuyển sang bước giữ (Case 10) thay vì dừng hẳn
            step_state = 10;
        }
        break;

    // ====================================================
    // GIAI ĐOẠN PHỤ: GIỮ VỊ TRÍ (ACTIVE HOLD) TRONG 3 GIÂY
    // ====================================================
    case 10:
        // VẪN GỌI HÀM DI CHUYỂN LIÊN TỤC!
        // Việc này giúp PID liên tục sửa lỗi nếu động cơ bị trôi
        Task_MoveAbsolute_Hybrid(target_pos);

        // Kiểm tra thời gian
        if (now - wait_timer > 3000)
        {
            Serial.println(">> CLASS: Closing step-by-step...");
            MoveAxisGripper(-1, 0); // Ngắt lực giữ của PID
            delay(50);              // Nghỉ 50ms để thả lỏng cơ khí
            // Hết 3s, chuyển sang đóng
            step_state = 2;
        }
        break;

    // ====================================================
    // GIAI ĐOẠN 2: ĐÓNG TỪNG BƯỚC & DÒ VẬT (GIỮ NGUYÊN)
    // ====================================================
    case 2: // KHỞI TẠO BƯỚC ĐÓNG (20 XUNG)
        start_step_pos = current;
        target_pos = current - STEP_PULSES; // Đóng (-)

        MoveAxisGripper(0, SPEED_CRAWL); // Close (Chạy thường để dò dòng)
        time_step_start = now;
        step_state = 3;
        break;

    case 3: // GIÁM SÁT KẸT
        if (abs(current - start_step_pos) >= STEP_PULSES)
        {
            // Đi xong 20 xung an toàn -> Tiếp tục đi tiếp
            Serial.printf(">>Step %d: %ld ms\n", i, now - time_step_start);
            i++;
            step_state = 2;
        }
        else if ((now - time_step_start > 15) && i > 20)
        {
            // KẸP TRÚNG VẬT -> Dừng lại
            i = 0;
            MoveAxisGripper(-1, 0);
            Serial.printf(">> CLASS: Gripped at %ld\n", current);
            step_state = 4;
        }
        break;

    // ====================================================
    // GIAI ĐOẠN 3: LÙI RA 20 XUNG (DÙNG HYBRID PID)
    // ====================================================
    case 4:

        Object_Size = (current + 20) * MM_PER_PULSE + 19.83; // offset 23mm cho phần cơ khí
        Serial.println("============== RESULT ==============");
        Serial.printf(" Encoder Pos : %ld\n", current + 20);
        Serial.printf(" Size        : %.2f mm\n", Object_Size);

        if (Object_Size < Object_Small_size)
        {
            Serial.println(" -> TYPE     : SMALL (Nho)");
            Number_of_Objects_small++;
            Flag_Object_size = 1;
        }
        else if (Object_Size < Object_Medium_size)
        {
            Serial.println(" -> TYPE     : MEDIUM (Vua)");
            Number_of_Objects_medium++;
            Flag_Object_size = 2;
        }
        else
        {
            Serial.println(" -> TYPE     : LARGE (Lon)");
            Number_of_Objects_large++;
            Flag_Object_size = 3;
        }

        Serial.println("====================================");
        target_pos = current + 80;
        move_state = 0; // Reset hàm di chuyển

        Serial.println(">> CLASS: Precision Backoff (+80)...");
        step_state = 5;
        break;

    case 5:
        // Lùi ra xong thì DỪNG HẲN (vì đã xong việc)
        if (Task_MoveAbsolute_Hybrid(target_pos) == 1)
        {
            MoveAxisGripper(-1, 0); // Lúc này mới được phép cắt điện
            Serial.println("Classify done! ");
            step_state = 0;
            Size_of_objects[Counter_Size_of_objects] = Object_Size;
            Counter_Size_of_objects++;
            // currentMode = MODE_IDLE;
            return 1;
        }
        break;
    }
    return 0;
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
            if (Mode == 2)
            {
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
            else
            {
                time_now = 0;
                value_now = 0;
                time_last = 0;
                value_last = 0;
                HasConsitancy = 0;
                HasConsitancy = 0;
                return 1; // đã kẹp vật
            }
        }
        return 0; // chưa kẹp vật
    }
}
// Hàm homing cho 1 trục (blocking)
#define HOME_SPEED_FAST 800.0
#define HOME_SPEED_SLOW 200.0
#define HOME_SPEED_RETREAT 400.0
#define HOME_RETREAT_DIST 400 // Bước lùi
#define LIMIT_ACTIVE_LEVEL 0  // Mức kích hoạt công tắc hành trình = 0 test nên để bằng 1

void SetHomeAll(void *parameter)
{
    static int state_Elbow = 0;
    static int state_Shoulder = 0;
    static int state_Base = 0;

    static unsigned long debounce_timer_Elbow = 0;
    static int last_read_Elbow = !LIMIT_ACTIVE_LEVEL;

    static unsigned long debounce_timer_Shoulder = 0;
    static int last_read_Shoulder = !LIMIT_ACTIVE_LEVEL;

    static unsigned long debounce_timer_Base = 0;
    static int last_read_Base = !LIMIT_ACTIVE_LEVEL;

    static int complete_homing_gripper = 0;

    Axis_Base.setAcceleration(5000);
    Axis_Shoulder.setAcceleration(5000);
    Axis_Elbow.setAcceleration(5000);

    for (;;)
    {
        if (Mode == -2)
        {
            // ============================================================
            // TRỤC 1: BASE
            // ============================================================
            if (state_Base == 0)
            {
                Axis_Base.setMaxSpeed(HOME_SPEED_FAST);
                Axis_Base.moveTo(-1000000);
                state_Base = 1;
                Serial.println("BASE: Start Homing Fast...");
            }

            if (state_Base == 1)
            {
                int reading = digitalRead(LIM_BASE_PIN);
                if (reading != last_read_Base)
                    debounce_timer_Base = millis();
                last_read_Base = reading;
                Serial.print("Base Limit: ");
                Serial.print(reading);
                Serial.print(" / ActiveLevel: ");
                Serial.print(LIMIT_ACTIVE_LEVEL);
                Serial.print(" / DebounceOK: ");
                Serial.println((millis() - debounce_timer_Base) > 50);

                bool isPressed = (reading == LIMIT_ACTIVE_LEVEL);

                if (isPressed)
                {
                    Axis_Base.stop();
                    state_Base = 2;
                }
                else
                    Axis_Base.run();
            }

            if (state_Base == 2)
            {
                if (!Axis_Base.isRunning())
                {
                    Axis_Base.setCurrentPosition(0);
                    Axis_Base.setMaxSpeed(HOME_SPEED_RETREAT);
                    Axis_Base.moveTo(HOME_RETREAT_DIST);
                    state_Base = 3;
                }
                else
                    Axis_Base.run();
            }

            if (state_Base == 3)
            {
                if (Axis_Base.distanceToGo() == 0)
                {
                    Axis_Base.setMaxSpeed(HOME_SPEED_SLOW);
                    Axis_Base.moveTo(-1000000);
                    state_Base = 4;
                }
                else
                    Axis_Base.run();
            }

            if (state_Base == 4)
            {
                int reading = digitalRead(LIM_BASE_PIN);
                if (reading != last_read_Base)
                    debounce_timer_Base = millis();
                last_read_Base = reading;

                bool isPressed = ((millis() - debounce_timer_Base) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                if (isPressed)
                {
                    Axis_Base.stop();
                    state_Base = 5;
                }
                else
                    Axis_Base.run();
            }

            if (state_Base == 5)
            {
                if (!Axis_Base.isRunning())
                {
                    Axis_Base.setCurrentPosition(0);
                    Axis_Base.moveTo(0);
                    state_Base = 6;
                    Serial.println("BASE: HOMED");
                }
                else
                    Axis_Base.run();
            }

            // ============================================================
            // TRỤC 2: SHOULDER (Chỉ chạy khi Base xong)
            // ============================================================
            if (state_Base == 6)
            {
                if (state_Shoulder == 0)
                {
                    Axis_Shoulder.setMaxSpeed(HOME_SPEED_FAST);
                    Axis_Shoulder.moveTo(-1000000);
                    state_Shoulder = 1;
                    Serial.println("SHOULDER: Start Homing Fast...");
                }

                if (state_Shoulder == 1)
                {
                    int reading = digitalRead(LIM_SHOULDER_PIN);
                    if (reading != last_read_Shoulder)
                        debounce_timer_Shoulder = millis();
                    last_read_Shoulder = reading;

                    bool isPressed = ((millis() - debounce_timer_Shoulder) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed)
                    {
                        Axis_Shoulder.stop();
                        state_Shoulder = 2;
                    }
                    else
                        Axis_Shoulder.run();
                }

                if (state_Shoulder == 2)
                {
                    if (!Axis_Shoulder.isRunning())
                    {
                        Axis_Shoulder.setCurrentPosition(0);
                        Axis_Shoulder.setMaxSpeed(HOME_SPEED_RETREAT);
                        Axis_Shoulder.moveTo(HOME_RETREAT_DIST);
                        state_Shoulder = 3;
                    }
                    else
                        Axis_Shoulder.run();
                }

                if (state_Shoulder == 3)
                {
                    if (Axis_Shoulder.distanceToGo() == 0)
                    {
                        Axis_Shoulder.setMaxSpeed(HOME_SPEED_SLOW);
                        Axis_Shoulder.moveTo(-1000000);
                        state_Shoulder = 4;
                    }
                    else
                        Axis_Shoulder.run();
                }

                if (state_Shoulder == 4)
                {
                    int reading = digitalRead(LIM_SHOULDER_PIN);
                    if (reading != last_read_Shoulder)
                        debounce_timer_Shoulder = millis();
                    last_read_Shoulder = reading;

                    bool isPressed = ((millis() - debounce_timer_Shoulder) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed)
                    {
                        Axis_Shoulder.stop();
                        state_Shoulder = 5;
                    }
                    else
                        Axis_Shoulder.run();
                }

                if (state_Shoulder == 5)
                {
                    if (!Axis_Shoulder.isRunning())
                    {
                        Axis_Shoulder.setCurrentPosition(0);
                        Axis_Shoulder.moveTo(0);
                        state_Shoulder = 6;
                        Serial.println("SHOULDER: HOMED");
                    }
                    else
                        Axis_Shoulder.run();
                }
            }

            // ============================================================
            // TRỤC 3: ELBOW (Chỉ chạy khi Shoulder xong)
            // ============================================================
            if (state_Shoulder == 6)
            {
                if (state_Elbow == 0)
                {
                    Axis_Elbow.setMaxSpeed(400);
                    Axis_Elbow.moveTo(-1000000);
                    state_Elbow = 1;
                    Serial.println("ELBOW: Start Homing Fast...");
                }

                if (state_Elbow == 1)
                {
                    int reading = digitalRead(LIM_ELBOW_PIN);
                    if (reading != last_read_Elbow)
                        debounce_timer_Elbow = millis();
                    last_read_Elbow = reading;

                    bool isPressed = ((millis() - debounce_timer_Elbow) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed)
                    {
                        Axis_Elbow.stop();
                        state_Elbow = 2;
                    }
                    else
                        Axis_Elbow.run();
                }

                if (state_Elbow == 2)
                {
                    if (!Axis_Elbow.isRunning())
                    {
                        Axis_Elbow.setCurrentPosition(0);
                        Axis_Elbow.setMaxSpeed(HOME_SPEED_RETREAT);
                        Axis_Elbow.moveTo(HOME_RETREAT_DIST);
                        state_Elbow = 3;
                    }
                    else
                        Axis_Elbow.run();
                }

                if (state_Elbow == 3)
                {
                    if (Axis_Elbow.distanceToGo() == 0)
                    {
                        Axis_Elbow.setMaxSpeed(HOME_SPEED_SLOW);
                        Axis_Elbow.moveTo(-1000000);
                        state_Elbow = 4;
                    }
                    else
                        Axis_Elbow.run();
                }

                if (state_Elbow == 4)
                {
                    int reading = digitalRead(LIM_ELBOW_PIN);
                    if (reading != last_read_Elbow)
                        debounce_timer_Elbow = millis();
                    last_read_Elbow = reading;

                    bool isPressed = ((millis() - debounce_timer_Elbow) > 50) && (reading == LIMIT_ACTIVE_LEVEL);

                    if (isPressed)
                    {
                        Axis_Elbow.stop();
                        state_Elbow = 5;
                    }
                    else
                        Axis_Elbow.run();
                }

                if (state_Elbow == 5)
                {
                    if (!Axis_Elbow.isRunning())
                    {
                        Axis_Elbow.setCurrentPosition(0);
                        Axis_Elbow.moveTo(0);
                        state_Elbow = 6;
                        Serial.println("ELBOW: HOMED");
                    }
                    else
                        Axis_Elbow.run();
                }
            }

            // RUN GRIPPER AT LAST
            if (state_Elbow == 6)
            {
                if (Task_Homing_Gripper() == 1)
                    complete_homing_gripper = 1;
            }

            // ============================================================
            // FINISH CHECK
            // ============================================================
            if (state_Base == 6 && state_Shoulder == 6 && state_Elbow == 6 && complete_homing_gripper == 1)
            {
                Serial.println("ALL DONE!");

                state_Elbow = state_Shoulder = state_Base = 0;
                complete_homing_gripper = 0;

                debounce_timer_Elbow = debounce_timer_Shoulder = debounce_timer_Base = millis();
                last_read_Elbow = last_read_Shoulder = last_read_Base = !LIMIT_ACTIVE_LEVEL;

                Mode = -3;
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }

            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            // RESET NGAY KHI KHÔNG Ở MODE HOMING
            state_Elbow = state_Shoulder = state_Base = 0;
            complete_homing_gripper = 0;

            debounce_timer_Elbow = debounce_timer_Shoulder = debounce_timer_Base = 0;
            last_read_Elbow = last_read_Shoulder = last_read_Base = !LIMIT_ACTIVE_LEVEL;

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

void ReturnToHome(void *parameter)
{
    // Biến static để nhớ trạng thái khởi tạo
    static bool home_initialized = false;

    while (1)
    {
        if (Mode == -1)
        {
            // --- GIAI ĐOẠN 1: KHỞI TẠO (CHẠY 1 LẦN DUY NHẤT) ---
            if (!home_initialized)
            {
                Serial.println(">> START HOMING SEQUENCE...");

                // 1. Cài đặt Stepper
                Axis_Base.setMaxSpeed(600);
                Axis_Shoulder.setMaxSpeed(600);
                Axis_Elbow.setMaxSpeed(600);

                Axis_Base.moveTo(0);
                Axis_Shoulder.moveTo(0);
                Axis_Elbow.moveTo(0);

                // 2. Reset trạng thái cho hàm Gripper Hybrid
                // QUAN TRỌNG: Phải reset để nó bắt đầu quy trình lao nhanh -> PID
                move_state = 0;

                home_initialized = true; // Đánh dấu đã khởi tạo xong
            }

            // --- GIAI ĐOẠN 2: THỰC THI (CHẠY LIÊN TỤC) ---

            // 1. Chạy Stepper
            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();

            // 2. Chạy Gripper (Dùng hàm Hybrid duy nhất)
            // Lưu kết quả trả về vào biến để kiểm tra sau
            int gripper_status = Task_MoveAbsolute_Hybrid(0);

            // --- GIAI ĐOẠN 3: KIỂM TRA HOÀN TẤT ---
            if (Axis_Base.distanceToGo() == 0 &&
                Axis_Shoulder.distanceToGo() == 0 &&
                Axis_Elbow.distanceToGo() == 0 &&
                gripper_status == 1) // Gripper đã báo xong (trả về 1)
            {
                Serial.println("✅ RETURN HOME COMPLETE!");

                Mode = -3;                // Chuyển sang Mode chờ
                home_initialized = false; // Reset cờ để lần sau dùng lại được
            }

            // Delay cực nhỏ để tránh Reset Watchdog nhưng đủ nhanh cho Stepper
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            // Nếu không phải mode Home, reset cờ để sẵn sàng cho lần gọi tới
            home_initialized = false;
            vTaskDelay(20 / portTICK_PERIOD_MS);
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
            case 4:
            Axis_Base.setMaxSpeed(1200);
                Axis_Base.moveTo(10000000); // quay trái
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Base Left");
                break;
            case 3:
                Axis_Base.moveTo(-10000000); // quay phải
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Base Right");
                break;
            case 1:
                Axis_Shoulder.moveTo(10000000); // nâng lên
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Shoulder Up");
                break;
            case 2:
                Axis_Shoulder.moveTo(-10000000); // hạ xuống
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                // Serial.println("Jog Shoulder Down");
                break;
            case 6:
                Axis_Elbow.moveTo(10000000); // vươn ra
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                // Serial.println("Jog Elbow Extend");
                break; // vươn ra
            case 5:
                Axis_Elbow.moveTo(-10000000); // thu lại
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                // Serial.println("Jog Elbow Retract");
                break; // thu lại
            case 7:
                direction_Gripper_Jog = 1;
                Axis_Gripper_setSpeed = 10;
                // Serial.println("Jog Gripper Open");
                break; // mở càng
            case 8:
                direction_Gripper_Jog = 0;
                Axis_Gripper_setSpeed = 10;
                // Serial.println("Jog Gripper Close");
                break; // đóng càng
            default:
                Axis_Base.moveTo(Axis_Base.currentPosition());
                Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
                Axis_Elbow.moveTo(Axis_Elbow.currentPosition());
                direction_Gripper_Jog = -1;
                Axis_Gripper_setSpeed = 0;
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
                abs(Encoder_Position - Axis_Gripper_Next_Position) <= 2)
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
            Axis_Base.setMaxSpeed(newSpeed);
            Axis_Shoulder.setMaxSpeed(newSpeed);
            Axis_Elbow.setMaxSpeed(newSpeed);
            Axis_Gripper_Speed = newSpeed;

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
            if (abs(Encoder_Position - Axis_Gripper_Next_Position) > 2)
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
Classify[0] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0} // sẽ cộng Y thêm 400 để tránh va chạm
Classify[1] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0}
Classify[2] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0} // bước kẹp vật, gọi hàm Task_Classify()
Classify[3] = {X: vật, Y: vật, Z: vật, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0}  //cộng Y thêm 400 để tránh va chạm
Classify[4] = {X: vị trí thả nhỏ, Y: vị trí thả nhỏ, Z: vị trí thả nhỏ, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0} //cộng Y thêm 400 để tránh va chạm
Classify[5] = {X: vị trí thả vừa, Y: vị trí thả vừa, Z: vị trí thả vừa, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0} //cộng Y thêm 400 để tránh va chạm
Classify[6] = {X: vị trí thả lớn, Y: vị trí thả lớn, Z: vị trí thả lớn, T: 0, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0} //cộng Y thêm 400 để tránh va chạm
Classify[7] = {X: 0, Y: 0, Z: 0, T: 600, speed_X:800, speed_Y:800, speed_Z:800, speed_T:0}
*/
int flag_stop = 0;
int MoveToStep_7 = 0;
void Robot_task_Classify(void *parameter)
{

    while (1)
    {
        if (Mode == 2)
        {
            long Encoder_Position = readEncoderCount();
            if (Axis_Base.distanceToGo() == 0 &&
                Axis_Shoulder.distanceToGo() == 0 &&
                Axis_Elbow.distanceToGo() == 0 &&
                flag_stop == 0
                // && Axis_Gripper_Stop == 1
            )
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
                        Classify[7] = Classify[4]; // lưu vị trí thả nhỏ vào bước 7
                        break;
                    case 2:
                        autoClassifyStep = 5;
                        MoveToStep_7 = 1;
                        Flag_Object_size = 0;
                        Classify[7] = Classify[5]; // lưu vị trí thả vừa vào bước 7
                        break;
                    case 3:
                        autoClassifyStep = 6;
                        MoveToStep_7 = 1;
                        Flag_Object_size = 0;
                        Classify[7] = Classify[6]; // lưu vị trí thả lớn
                        break;
                    default:
                        break;
                    }
                }
                /*
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
                    */
                if (autoClassifyStep > 7)
                {
                    autoClassifyStep = 0;
                }
            }

            Axis_Base.setMaxSpeed(Classify[autoClassifyStep].speedX);
            Axis_Shoulder.setMaxSpeed(Classify[autoClassifyStep].speedY);
            Axis_Elbow.setMaxSpeed(Classify[autoClassifyStep].speedZ);
            // Axis_Gripper_Speed = Classify[autoClassifyStep].speedT;

            Axis_Base.moveTo(Classify[autoClassifyStep].x);
            Axis_Shoulder.moveTo(Classify[autoClassifyStep].y);
            Axis_Elbow.moveTo(Classify[autoClassifyStep].z);
            // Axis_Gripper_Next_Position = Classify[autoClassifyStep].t;

            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();
            /*
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
            */
            if (autoClassifyStep == 2)
            {
                flag_stop = 1;
                if (Task_Classify() == 1)
                {
                    flag_stop = 0;
                }
            }
            else if (autoClassifyStep == 7)
            {
                flag_stop = 1;
                if (Task_MoveAbsolute_Hybrid(600) == 1) // mở càng ra 600 xung
                {
                    flag_stop = 0;
                }
            }
            /*
            if (Disable_Axis_Gripper == 0)
            {
                if (Detach_object_flag == 0)
                {
                    Encoder_Position = readEncoderCount();
                    if (abs(Encoder_Position - Axis_Gripper_Next_Position) > 2)
                    {
                        MoveAxisGripper(Direction_Gripper, Axis_Gripper_Speed);
                    }
                    if (abs(Encoder_Position - Axis_Gripper_Next_Position) <= 2)
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
*/
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
}
void StopAllTask(void *parameter)
{
    while (1)
    {
        if (Mode == -3)
        {
            Axis_Base.setAcceleration(4000);
            Axis_Shoulder.setAcceleration(4000);
            Axis_Elbow.setAcceleration(4000);

            Axis_Base.moveTo(Axis_Base.currentPosition());
            Axis_Shoulder.moveTo(Axis_Shoulder.currentPosition());
            Axis_Elbow.moveTo(Axis_Elbow.currentPosition());

            Axis_Base.setMaxSpeed(2000);
            Axis_Shoulder.setMaxSpeed(2000);
            Axis_Elbow.setMaxSpeed(2000);

            Axis_Base.run();
            Axis_Shoulder.run();
            Axis_Elbow.run();
            MoveAxisGripper(-1, 0); // dừng càng

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
    xTaskCreate(StopAllTask, "StopAllTask", 2048, NULL, 4, NULL);
}

void setup()
{
    Serial.begin(115200);
    pinMode(LIM_ELBOW_PIN, INPUT_PULLUP);    // cấu hình chân công tắc hành trình trục khuỷu
    pinMode(LIM_SHOULDER_PIN, INPUT_PULLUP); // cấu hình chân công tắc hành trình trục nâng
    pinMode(LIM_BASE_PIN, INPUT_PULLUP);     // cấu hình chân công tắc hành trình trục quay
    // l298n driver input
    ledcAttachChannel(Axis_Gripper_forward_PIN, 100, 10, 0);  // mở càng
    ledcAttachChannel(Axis_Gripper_backward_PIN, 100, 10, 1); // đóng càng
    // cấu hình encoder
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), onEncoderPulse, CHANGE);
    Serial.println("Encoder interrupt ready!");
    // Cấu hình trục
    Axis_Base.setMaxSpeed(2000);
    Axis_Base.setAcceleration(4000);
    Axis_Shoulder.setMaxSpeed(2000);
    Axis_Shoulder.setAcceleration(4000);
    Axis_Elbow.setMaxSpeed(2000);
    Axis_Elbow.setAcceleration(4000);
    Axis_Base.moveTo(0);
    Axis_Shoulder.moveTo(0);
    Axis_Elbow.moveTo(0);

    Axis_Shoulder.setPinsInverted(true, false, false); // đảo base
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
    else
    {
        if (task != "STOP")
        {
            Serial.println("!!! MAT KET NOI WIFI - DUNG KHAN CAP !!!");
            task = "STOP";
        }
        // Serial.println(task);
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

    handleSerialInput(); // Xử lý lệnh bàn phím

    // --- 2. Phần Debug (Chỉ chạy mỗi 500ms) ---
    if (millis() - lastDebugTime > DEBUG_INTERVAL)
    {
        lastDebugTime = millis(); // Cập nhật thời gian

        // In trạng thái hiện tại
        Serial.print("Mode: ");
        Serial.print(Mode);
        Serial.print("| POS [Base]: ");
        Serial.print(Axis_Base.currentPosition());

        Serial.print(" | [Shoulder]: ");
        Serial.print(Axis_Shoulder.currentPosition());

        Serial.print(" | [Elbow]: ");
        Serial.print(Axis_Elbow.currentPosition());

        Serial.print(" | [Gripper]: ");
        Serial.println(readEncoderCount());
    }
}