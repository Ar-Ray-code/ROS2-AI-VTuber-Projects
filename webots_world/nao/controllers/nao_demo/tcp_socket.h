#pragma once

#include "rest_c/rest_tcp_server.h"

// tcp socket program (REST API)
// - port : 8080
// - how to access : http://localhost:8080/*** (GET, POST, PUT, DELETE)

struct receive_data {
  int face_emotion; // 顔の表情 (0: OFF, 1: RED (angry), 2: GREEN (happy), 3: BLUE (sad))
  bool hand_wave_enable; // 手を振る
  bool tai_chi_enable; // 体操
  bool wipe_enable; // 汗を拭く
  float hands_angle; // 手の角度 set_hands_angle(0.96);
}

// receive data (PUT)
// - face_emotion : 0, 1, 2, 3
// (ex) curl http://localhost:8080/face_emotion -X PUT -H "Content-Type: application/json" -d '1'
// - hand_wave_enable : 0, 1
// (ex) curl http://localhost:8080/hand_wave_enable -X PUT -H "Content-Type: application/json" -d '1'
// - tai_chi_enable : 0, 1
// (ex) curl http://localhost:8080/tai_chi_enable -X PUT -H "Content-Type: application/json" -d '1'
// - wipe_enable : 0, 1
// (ex) curl http://localhost:8080/wipe_enable -X PUT -H "Content-Type: application/json" -d '1'
// - hands_angle : 0.0 ~ 1.0
// (ex) curl http://localhost:8080/hands_angle -X PUT -H "Content-Type: application/json" -d '0.96'

// send data (GET)
// - face_emotion : 0, 1, 2, 3
// (ex) curl http://localhost:8080/face_emotion -X GET
// - hand_wave_enable : 0, 1
// (ex) curl http://localhost:8080/hand_wave_enable -X GET
// - tai_chi_enable : 0, 1
// (ex) curl http://localhost:8080/tai_chi_enable -X GET
// - wipe_enable : 0, 1
// (ex) curl http://localhost:8080/wipe_enable -X GET
// - hands_angle : 0.0 ~ 1.0
// (ex) curl http://localhost:8080/hands_angle -X GET

// delete data (DELETE)
// (No action)

// =============================================


// create server (8080)

// void socket_

// create socket

// bind socket
