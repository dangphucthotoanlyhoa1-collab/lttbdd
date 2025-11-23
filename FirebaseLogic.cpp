#include "FirebaseLogic.h"
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <time.h>

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
long last_x = -999999;
long last_y = -999999;
long last_z = -999999;
long last_T = -999999;
float last_object_size = -999999;
int last_Number_of_Objects_small =-99999;
int last_Number_of_Objects_medium =-99999;
int last_Number_of_Objects_large =-99999;
unsigned long sendDataPrevMillis = 0;
static int beat_count = 0;
void parseArray(const String &jsonStr) { 
       static StaticJsonDocument<2048> doc; 
    doc.clear(); // Xóa dữ liệu cũ trước khi nạp mới
    if (!deserializeJson(doc, jsonStr)) {
        JsonArray array = doc.as<JsonArray>();
        diem_hien_co = 0;
        for (JsonObject p : array) {
            if (diem_hien_co >= 50) break; // Giới hạn 50 điểm ảnh để an toàn
            
            // Dùng toán tử | 0 để tránh lỗi nếu dữ liệu null
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
        Serial.print("=> Da cap nhat mang: "); Serial.println(diem_hien_co);
    }
}


void streamCallback(StreamData data) {
  Serial.printf("sream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());

  if (data.dataPath() == "/") {
    if (data.dataType() == "json") {
      FirebaseJson tmp = data.jsonObject();
      FirebaseJsonData result;

      // 1. Nhận lệnh JOG
      tmp.get(result, "jog/command_jog");
      if (result.success) {
         command_jog = result.to<int>();
         Serial.print("Sync JOG: "); Serial.println(command_jog);
      }

      // 2. Nhận lệnh TASK
      tmp.get(result, "funtion/Task");
      if (result.success) {
         task = result.to<String>();
         Serial.print("Sync TASK: "); Serial.println(task);
      }
      
      // 3. Nhận các dữ liệu Auto/Classify
      tmp.get(result, "Autochain/TotalSteps");
      if (result.success) TotalSteps = result.to<int>();

      tmp.get(result, "Autochain/Coordinates_auto");
      if (result.success) parseArray(result.to<String>());
      
      tmp.get(result, "Classify/TotalSteps");
      if (result.success) TotalSteps = result.to<int>();

      tmp.get(result, "Classify/Coordinates_auto");
      if (result.success) parseArray(result.to<String>());
    }
  }
  // Xử lý các path lẻ (Giữ nguyên logic cũ của bạn)
  else if (data.dataPath() == "/jog/command_jog") {
     command_jog = data.intData();
     Serial.print("Update JOG: "); Serial.println(command_jog);
  }
  else if (data.dataPath() == "/funtion/Task") {
     task = data.stringData();
     Serial.print("Update TASK: "); Serial.println(task);
  }
  else if (data.dataPath() == "/Autochain/Coordinates_auto") {
     parseArray(data.jsonString());
  }
  else if (data.dataPath() == "/Autochain/TotalSteps") {
     TotalSteps = data.intData();
  }
  else if (data.dataPath() == "/Classify/Coordinates_auto") {
     parseArray(data.jsonString());
  }
  else if (data.dataPath() == "/Classify/TotalSteps") {
     TotalSteps = data.intData();
  }
  else if (data.dataPath() == "/control/Autochain/Coordinates_auto") {
      String jsonStr = data.jsonString();
      
      Serial.println(">> Nhan duoc mang toa do tu App!");
      
      parseArray(jsonStr); 
  }
}

// =================================================================
// HÀM SETUP FIREBASE
// =================================================================
void setupFirebase() {
  Serial.begin(115200);
  
  // Lưu ý: Đảm bảo WiFi đã được kết nối ở setup() chính
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD); 

  Serial.print("NTP...");
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  int retry = 0;
  while (time(nullptr) < 1000000000 && retry < 20) {
    Serial.print("."); delay(500); retry++;
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

  if (Firebase.beginStream(fbdo, ROBOT_NODE "/control")) {
    Firebase.setStreamCallback(fbdo, streamCallback, NULL);
    Serial.println("Stream Ready");
  }
}
void loopFirebase() {
    if (Firebase.ready() && (millis() - sendDataPrevMillis > 150 || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();
        bool posChanged = (Axis_Base.currentPosition() != last_x) || (Axis_Shoulder.currentPosition() != last_y) || (Axis_Elbow.currentPosition() != last_z) || (readEncoderCount() != last_T);
        
        if (posChanged) {
             FirebaseJson jsonPos;
             jsonPos.set("pos/x", Axis_Base.currentPosition());
             jsonPos.set("pos/y", Axis_Shoulder.currentPosition());
             jsonPos.set("pos/z", Axis_Elbow.currentPosition());
             jsonPos.set("pos/T", readEncoderCount());
             jsonPos.set("speed/x", Axis_Base.speed());
            jsonPos.set("speed/y", Axis_Shoulder.speed());
            jsonPos.set("speed/z", Axis_Elbow.speed());
             Firebase.updateNodeSilent(fbdo_push, ROBOT_NODE "/sensor_data", jsonPos);
             last_x = Axis_Base.currentPosition(); last_y = Axis_Shoulder.currentPosition(); last_z = Axis_Elbow.currentPosition(); last_T =readEncoderCount();
        }
        // 2. XỬ LÝ KÍCH THƯỚC VẬT THỂ (Độc lập, ít thay đổi)
        if (Object_Size != last_object_size) {
             Firebase.setFloat(fbdo_push, ROBOT_NODE "/sensor_data/object/size", Object_Size);
             last_object_size = Object_Size;
        }
         if (Number_of_Objects_small != last_Number_of_Objects_small) {
             Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/small", Number_of_Objects_small );
             last_Number_of_Objects_small = Number_of_Objects_small;
        }
         if (Number_of_Objects_medium != last_Number_of_Objects_medium) {
             Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/medium", Number_of_Objects_medium);
             last_Number_of_Objects_medium = Number_of_Objects_medium;
        }
         if (Number_of_Objects_large != last_Number_of_Objects_large) {
             Firebase.setInt(fbdo_push, ROBOT_NODE "/sensor_data/count/large", Number_of_Objects_large);
             last_Number_of_Objects_large = Number_of_Objects_large;
        }
        if ( Counter_Size_of_objects != last_counter_size) {
            FirebaseJson json;
            FirebaseJsonArray arr; 
               for (int i = 0; i < Counter_Size_of_objects; i++) {
               arr.add(Size_of_objects[i]); 
            }
            json.set("list_size", arr); 
                            json.set("total_count", Counter_Size_of_objects);
            Firebase.updateNodeSilent(fbdo_push, ROBOT_NODE "/sensor_data/list", json);
            last_counter_size = Counter_Size_of_objects;
            Serial.printf("Da cap nhat mang phan loai: %d vat\n", Counter_Size_of_objects);
        }
        }
       if (Firebase.ready() && (millis() - last_heartbeat_ms > 2000)) {
    last_heartbeat_ms = millis();
    
    beat_count++; 
    if (beat_count > 100) beat_count = 0; // Reset khi đến 100
    
    Firebase.setInt(fbdo_push, ROBOT_NODE "/status/heartbeat", beat_count);
}
}



