#include "FirebaseLogic.h"
#include "config.h"
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <time.h>
String task = "";
void parseArray(const String &jsonStr);
// 1. KHAI BÁO ĐỐI TƯỢNG FIREBASE
FirebaseData stream;
FirebaseData fbdo;
FirebaseData fbdo_push;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long last_heartbeat_ms = 0;
extern float Size_of_objects[100];
extern int Counter_Size_of_objects;
int last_counter_size = -1;
long last_x = -9999;
long last_y = -9999;
long last_z = -9999;
long last_T = -9999;
float last_object_size = -9999;
int last_Number_of_Objects_small = -99;
int last_Number_of_Objects_medium = -99;
int last_Number_of_Objects_large = -99;
unsigned long sendDataPrevMillis = 0;

// int Object_Small_size;
// int Object_Large_size;
// int Object_Medium_size;
static int beat_count = 0;
volatile int diem_hien_co = 0;
void parseArray(const String &jsonStr)
{
   Mode = -3; 
   // Guard: nếu chuỗi rỗng hoặc chỉ chứa null thì thử fetch từ RTDB một lần
   String payload = jsonStr;
   if (payload.length() == 0 || payload == "null")
   {
      Serial.println("parseArray: empty or null jsonStr, attempting fetch from RTDB...");
      // Thử lấy node Autochain/Coordinates_auto trực tiếp từ DB
      String fullPath = String(ROBOT_NODE) + String("/control/Autochain/Coordinates_auto");
      if (Firebase.get(fbdo_push, fullPath.c_str()))
      {
         payload = fbdo_push.jsonString();
         Serial.print("Fetched length: "); Serial.println(payload.length());
      }
      else
      {
         Serial.print("fetch from RTDB failed: "); Serial.println(fbdo_push.errorReason());
      }
      if (payload.length() == 0 || payload == "null")
      {
         Serial.println("parseArray: no data available after fetch, aborting");
         return;
      }
   }

   // In ra JSON nhận về để debug (theo ví dụ bạn cung cấp)
   Serial.print("JSON nhan ve: ");
   Serial.println(payload);

   // Dự phòng: cấp phát đủ lớn cho mảng tới 1000 phần tử * nhỏ
   // nhưng để an toàn trên ESP32, giới hạn buffer (16KB)
   const size_t BUFFER_SIZE = 16384;
   DynamicJsonDocument doc(BUFFER_SIZE);

   DeserializationError error = deserializeJson(doc, payload);
   if (error)
   {
      Serial.print("Loi JSON Parse: ");
      Serial.println(error.c_str());
      return;
   }

   // Reset counter trước khi nạp mới
   diem_hien_co = 0;

   // Nếu là mảng JSON
   if (doc.is<JsonArray>())
   {
      Serial.println("parseArray: detected JSON Array");
      JsonArray arr = doc.as<JsonArray>();
      for (JsonVariant v : arr)
      {
         if (diem_hien_co >= 1000) break; // giới hạn theo kích thước mảng
         if (v.isNull()) continue;
         JsonObject p = v.as<JsonObject>();

         Coordinates_auto[diem_hien_co].x = p["X"] | 0;
         Coordinates_auto[diem_hien_co].y = p["Y"] | 0;
         Coordinates_auto[diem_hien_co].z = p["Z"] | 0;
         Coordinates_auto[diem_hien_co].t = p["T"] | 0;
         Coordinates_auto[diem_hien_co].speedX = p["speed_X"] | 0;
         Coordinates_auto[diem_hien_co].speedY = p["speed_Y"] | 0;
         Coordinates_auto[diem_hien_co].speedZ = p["speed_Z"] | 0;
         Coordinates_auto[diem_hien_co].speedT = p["speed_T"] | 0;

         diem_hien_co++;
      }
   }
   // Nếu là object với keys '0','1',... (Firebase có thể trả về dạng này)
   else if (doc.is<JsonObject>())
   {
      Serial.println("parseArray: detected JSON Object");
      JsonObject root = doc.as<JsonObject>();
      for (JsonPair kv : root)
      {
         if (diem_hien_co >= 1000) break;
         JsonObject p = kv.value().as<JsonObject>();
         if (p.isNull()) continue;

         Coordinates_auto[diem_hien_co].x = p["X"] | 0;
         Coordinates_auto[diem_hien_co].y = p["Y"] | 0;
         Coordinates_auto[diem_hien_co].z = p["Z"] | 0;
         Coordinates_auto[diem_hien_co].t = p["T"] | 0;
         Coordinates_auto[diem_hien_co].speedX = p["speed_X"] | 0;
         Coordinates_auto[diem_hien_co].speedY = p["speed_Y"] | 0;
         Coordinates_auto[diem_hien_co].speedZ = p["speed_Z"] | 0;
         Coordinates_auto[diem_hien_co].speedT = p["speed_T"] | 0;

         diem_hien_co++;
      }
   }

   // Đồng bộ TotalSteps
   TotalSteps = diem_hien_co;

   // In kết quả
   Serial.print("=> Da cap nhat so diem: ");
   Serial.println(diem_hien_co);

   if (diem_hien_co > 0)
   {
      Serial.printf("Point0 X=%.1f Y=%.1f Z=%.1f T=%d speedX=%.1f speedY=%.1f speedZ=%.1f speedT=%d\n",
                 Coordinates_auto[0].x,
                 Coordinates_auto[0].y,
                 Coordinates_auto[0].z,
                 Coordinates_auto[0].t,
                 Coordinates_auto[0].speedX,
                 Coordinates_auto[0].speedY,
                 Coordinates_auto[0].speedZ,
                 Coordinates_auto[0].speedT);
   }
}

void streamCallback(StreamData data)
{
   Serial.printf("sream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                 data.streamPath().c_str(),
                 data.dataPath().c_str(),
                 data.dataType().c_str(),
                 data.eventType().c_str());
   if (data.dataPath() == "/speed" || data.dataPath() == "/")
   {
      FirebaseJson json = data.jsonObject();
      FirebaseJsonData result;

      json.get(result, "speed");
      if (result.success)
      {
         int newSpeed = result.to<int>();
         Axis_Base.setMaxSpeed(newSpeed);
         Axis_Shoulder.setMaxSpeed(newSpeed);
         Axis_Elbow.setMaxSpeed(newSpeed);
         Serial.printf(" Tốc độ mới: %d\n", newSpeed);
      }
   }

   if (data.dataPath() == "/")
   {
      if (data.dataType() == "json")
      {
         FirebaseJson tmp = data.jsonObject();
         FirebaseJsonData result;

         // 1. Nhận lệnh JOG
         tmp.get(result, "jog/command_jog");
         if (result.success)
         {
            command_jog = result.to<int>();
            Serial.print("Sync JOG: ");
            Serial.println(command_jog);
         }
         tmp.get(result, "size/small");
         if (result.success)
         {
            Object_Small_size = result.to<int>();
            Serial.print("size small: ");
            Serial.println(Object_Small_size);
         }
         tmp.get(result, "size/medium");
         if (result.success)
         {
            Object_Medium_size = result.to<int>();
            Serial.print("size medium: ");
            Serial.println(Object_Medium_size);
         }
         tmp.get(result, "size/large");
         if (result.success)
         {
            Object_Large_size = result.to<int>();
            Serial.print("size large: ");
            Serial.println(Object_Large_size);
         }

         // 2. Nhận lệnh TASK
         tmp.get(result, "funtion/Task");
         if (result.success)
         {
            task = result.to<String>();
            Serial.print("Sync TASK: ");
            Serial.println(task);
         }

         // 3. Nhận các dữ liệu Auto/Classify
         tmp.get(result, "Autochain/TotalSteps");
         if (result.success)
            TotalSteps = result.to<int>();

         tmp.get(result, "Autochain/Coordinates_auto");
         if (result.success)
         {
            parseArray(result.to<String>());
         }

         tmp.get(result, "Classify/TotalSteps");
         if (result.success)
            TotalSteps = result.to<int>();

         tmp.get(result, "Classify/Coordinates_auto");
         if (result.success){
            parseArray(result.to<String>());
         }
            
      }
   }
   // Xử lý các path lẻ (Giữ nguyên logic cũ của bạn)
   else if (data.dataPath() == "/jog/command_jog")
   {
      Mode = 0; // Chuyển về mode JOG khi nhận lệnh JOG mới
      command_jog = data.intData();
      Serial.print("Update JOG: ");
      Serial.println(command_jog);
   }
    if (data.dataPath() == "/size/small")
   {
      Object_Small_size = data.intData();
      Serial.print("Update size small: ");
      Serial.println(Object_Small_size);
   }
    if (data.dataPath() == "/size/medium")
   {
      Object_Medium_size = data.intData();
      Serial.print("Update size medium: ");
      Serial.println(Object_Medium_size);
   }
    if (data.dataPath() == "/size/large")
   {
      Object_Large_size = data.intData();
      Serial.print("Update size large: ");
      Serial.println(Object_Large_size);
   }
    if (data.dataPath() == "/funtion/Task")
   {
      task = data.stringData();
      Serial.print("Update TASK: ");
      Serial.println(task);
      if (task == "AutoChain") {
         Mode = 1;
      } else if (task == "Stop" || task == "Pause") {
         Mode = -3;
      } if (task == "Classify") {
         Mode = 2;
      } else if (task == "ReturnHome") {
         Mode = -1;
      } else if (task == "SetHome") {
         Mode = -2;
      }
      Serial.print("Mode set to: ");
      Serial.println(Mode);
   }
    if (data.dataPath() == "/Autochain/Coordinates_auto")
   {
      Mode = -3; 
      parseArray(data.stringData());
      Serial.println(">> Toa do cua du lieu JSON: ");
      Serial.println(data.stringData());
      Serial.println(">> Toa do nhan duoc tu du lieu JSON: ");
      for (int i = 0; i < 3; i++)
      {
         Serial.print("Diem ");
         Serial.print(i);
         Serial.print(": X=");
         Serial.print(Coordinates_auto[i].x);
         Serial.print(", Y=");
         Serial.print(Coordinates_auto[i].y);
         Serial.print(", Z=");
         Serial.print(Coordinates_auto[i].z);
         Serial.print(", T=");
         Serial.print(Coordinates_auto[i].t);
         Serial.print(", speedX=");
         Serial.print(Coordinates_auto[i].speedX);
         Serial.print(", speedY=");
         Serial.print(Coordinates_auto[i].speedY);
         Serial.print(", speedZ=");
         Serial.print(Coordinates_auto[i].speedZ);
         Serial.print(", speedT=");
         Serial.println(Coordinates_auto[i].speedT);
      }
      
   }
   else if (data.dataPath() == "/Autochain/TotalSteps")
   {
      Mode = 1;
      TotalSteps = data.intData();
      Serial.print("gia tri cua totalsteps ");
      Serial.println(TotalSteps);
   }
   else if (data.dataPath() == "/Classify/Coordinates_auto")
   {
      Mode = 2; // Chuyển về mode CLASSIFY khi nhận lệnh mới
      parseArray(data.jsonString());
      
      
   }
   else if (data.dataPath() == "/Classify/TotalSteps")
   {
      Mode = 2;
      TotalSteps = data.intData();
   }
   else if (data.dataPath() == "/control/Autochain/Coordinates_auto")
   {
      String jsonStr = data.jsonString();

      Serial.println(">> Nhan duoc mang toa do tu App!");

      parseArray(jsonStr);
   }
}

// =================================================================
// HÀM SETUP FIREBASE
// =================================================================
void setupFirebase()
{
   Serial.begin(115200);

   Serial.print("NTP...");
   configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
   int retry = 0;
   while (time(nullptr) < 1000000000 && retry < 20)
   {
      Serial.print(".");
      delay(500);
      retry++;
   }
   Serial.println(" OK");

   config.host = FIREBASE_HOST;
   config.api_key = FIREBASE_KEY;
   auth.user.email = USER_EMAIL;
   auth.user.password = USER_PASS;
   fbdo.setBSSLBufferSize(4096, 1024);
   fbdo.setResponseSize(4096);
   fbdo_push.setBSSLBufferSize(2048, 1024);
   fbdo_push.setResponseSize(2048);
   Firebase.begin(&config, &auth);
   Firebase.reconnectWiFi(true);

   if (Firebase.beginStream(fbdo, ROBOT_NODE "/control"))
   {
      Firebase.setStreamCallback(fbdo, streamCallback, NULL);
      Serial.println("Stream Ready");
   }
}
void loopFirebase()
{
   if (Firebase.ready() && (millis() - sendDataPrevMillis > 150 || sendDataPrevMillis == 0))
   {
      sendDataPrevMillis = millis();
      bool posChanged = (Axis_Base.currentPosition() != last_x) || (Axis_Shoulder.currentPosition() != last_y) || (Axis_Elbow.currentPosition() != last_z) || (readEncoderCount() != last_T);

      if (posChanged)
      {
         FirebaseJson jsonPos;
         jsonPos.set("pos/x", Axis_Base.currentPosition());
         jsonPos.set("pos/y", Axis_Shoulder.currentPosition());
         jsonPos.set("pos/z", Axis_Elbow.currentPosition());
         jsonPos.set("pos/T", readEncoderCount());
         jsonPos.set("speed/x", Axis_Base.speed());
         jsonPos.set("speed/y", Axis_Shoulder.speed());
         jsonPos.set("speed/z", Axis_Elbow.speed());
         Firebase.updateNodeSilent(fbdo_push, ROBOT_NODE "/sensor_data", jsonPos);
         last_x = Axis_Base.currentPosition();
         last_y = Axis_Shoulder.currentPosition();
         last_z = Axis_Elbow.currentPosition();
         last_T = readEncoderCount();
      }
      // 2. XỬ LÝ KÍCH THƯỚC VẬT THỂ (Độc lập, ít thay đổi)
      if (Object_Size != last_object_size)
      {
         Firebase.setFloat(fbdo_push, ROBOT_NODE "/sensor_data/object/size", Object_Size);
         last_object_size = Object_Size;
      }
      if (Number_of_Objects_small != last_Number_of_Objects_small)
      {
         Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/small", Number_of_Objects_small);
         last_Number_of_Objects_small = Number_of_Objects_small;
      }
      if (Number_of_Objects_medium != last_Number_of_Objects_medium)
      {
         Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/medium", Number_of_Objects_medium);
         last_Number_of_Objects_medium = Number_of_Objects_medium;
      }
      if (Number_of_Objects_large != last_Number_of_Objects_large)
      {
         Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/large", Number_of_Objects_large);
         last_Number_of_Objects_large = Number_of_Objects_large;
      }
      if (Counter_Size_of_objects != last_counter_size)
      {
         FirebaseJson json;
         FirebaseJsonArray arr;
         for (int i = 0; i < Counter_Size_of_objects; i++)
         {
            arr.add(Size_of_objects[i]);
         }
         json.set("list_size", arr);
         json.set("total_count", Counter_Size_of_objects);
         Firebase.updateNodeSilent(fbdo_push, ROBOT_NODE "/sensor_data/list", json);
         last_counter_size = Counter_Size_of_objects;
         Serial.printf("Da cap nhat mang phan loai: %d vat\n", Counter_Size_of_objects);
      }
   }
   if (Firebase.ready() && (millis() - last_heartbeat_ms > 2000))
   {
      last_heartbeat_ms = millis();

      beat_count++;
      if (beat_count > 100)
         beat_count = 0; // Reset khi đến 100

      Firebase.setInt(fbdo_push, ROBOT_NODE "/status/heartbeat", beat_count);
   }
}