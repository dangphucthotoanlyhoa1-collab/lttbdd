#include "FirebaseLogic.h"
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <time.h>

FirebaseData stream; 
FirebaseData fbdo;       
FirebaseData fbdo_push;  
FirebaseAuth auth;
FirebaseConfig config;


void parseArray(String jsonStr) {
    DynamicJsonDocument doc(2048); 
    if (!deserializeJson(doc, jsonStr)) {
        JsonArray array = doc.as<JsonArray>();
        diem_hien_co = 0;
        for (JsonObject p : array) {
            if (diem_hien_co >= 100) break; 
            Coordinates_auto[diem_hien_co].x = p["X"];
            Coordinates_auto[diem_hien_co].y = p["Y"];
            Coordinates_auto[diem_hien_co].z = p["Z"];
            Coordinates_auto[diem_hien_co].t = p["T"];
            Coordinates_auto[diem_hien_co].speedX = p["speed_X"];
            Coordinates_auto[diem_hien_co].speedY = p["speed_Y"];
            Coordinates_auto[diem_hien_co].speedZ = p["speed_Z"];
            Coordinates_auto[diem_hien_co].speedT = p["speed_T"];
            diem_hien_co++;
        }
        Serial.print("=> Da cap nhat mang: "); Serial.println(diem_hien_co);
    }
}

void streamCallback(StreamData data) {
  Serial.printf("Path: %s | Type: %s\n", data.dataPath().c_str(), data.dataType().c_str());
  if (data.dataPath() == "/") {
    if (data.dataType() == "json") {
      FirebaseJson tmp = data.jsonObject();
      FirebaseJsonData result;

        tmp.get(result, "jog/command_jog");
      if (result.success) {
         command_jog = result.to<int>();
         Serial.print("Sync JOG: "); Serial.println(command_jog);
      }

  
      tmp.get(result, "funtin/Task");
      if (result.success) {
         task = result.to<String>();
         Serial.print("Sync TASK: "); Serial.println(task);
      }
      
 
      tmp.get(result, "Autochain/TotalSteps");
      if (result.success) TotalSteps = result.to<int>();

      tmp.get(result, "Autochain/Coordinates_auto");
      if (result.success) {
         parseArray(result.to<String>());
      }
        tmp.get(result, "Classify/TotalSteps");
      if (result.success) TotalSteps = result.to<int>();

      tmp.get(result, "Classify/Coordinates_auto");
      if (result.success) {
         parseArray(result.to<String>());
      }

    }
  }
    
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
     Serial.println(TotalSteps);
  }
    else if (data.dataPath() == "/Classify/Coordinates_auto") {
     parseArray(data.jsonString());
  }
    else if (data.dataPath() == "/Classify/TotalSteps") {
     TotalSteps = data.intData();
     Serial.println(TotalSteps);
  }
}

void pushDataTask(void *param) {
  for(;;) {
    if(WiFi.status() == WL_CONNECTED && Firebase.ready()) {
        FirebaseJson json;
        json.set("pos/x", Axis_Base.currentPosition()); 
        json.set("pos/y", Axis_Shoulder.currentPosition());
        json.set("pos/z", Axis_Elbow.currentPosition());
        json.set("pos/T", readEncoderCount());
        json.set("speed/x", Axis_Base.speed());
        json.set("speed/y", Axis_Shoulder.speed());
        json.set("speed/z", Axis_Elbow.speed());
        json.set("object/size", Object_Size);
        
        // Gửi bằng kênh riêng fbdo_push
        Firebase.updateNode(fbdo_push, ROBOT_NODE "/sensor_data", json);
    } else {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setupFirebase() {
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

  // Tối ưu bộ đệm
  fbdo.setBSSLBufferSize(2048, 1024); 
  fbdo.setResponseSize(2048);
  fbdo_push.setBSSLBufferSize(2048, 1024);
  fbdo_push.setResponseSize(2048);

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  if (Firebase.beginStream(fbdo, ROBOT_NODE "/control")) {
    Firebase.setStreamCallback(fbdo, streamCallback, NULL);
    Serial.println("Stream Ready");
  }
  
  xTaskCreatePinnedToCore(pushDataTask, "PushFB", 16384, NULL, 1, NULL, 1); 
}

void loopFirebase() {}