#ifndef FIREBASE_LOGIC_H
#define FIREBASE_LOGIC_H
#include <FirebaseESP32.h>
#include "config.h"
void streamCallback(StreamData data);
void setupFirebase();
void pushDataTask(void *param);
void loopFirebase(); 

#endif