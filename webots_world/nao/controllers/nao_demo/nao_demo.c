/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//---------------------------------------------------------------------------------
//  Description:  Example C controller program for Nao robot.
//                This demonstrates how to access sensors and actuators
//---------------------------------------------------------------------------------

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/utils/motion.h>

#include "rest_c/rest_tcp_server.h"

#ifdef _MSC_VER
#define snprintf sprintf_s
#endif

#define PHALANX_MAX 8

static int time_step = -1;

// simulated devices
static WbDeviceTag CameraTop, CameraBottom;  // cameras
static WbDeviceTag us[2];                    // ultra sound sensors
static WbDeviceTag accelerometer, gyro, inertial_unit;
static WbDeviceTag fsr[2];                        // force sensitive resistors
static WbDeviceTag lfoot_lbumper, lfoot_rbumper;  // left foot bumpers
static WbDeviceTag rfoot_lbumper, rfoot_rbumper;  // right foot bumpers
static WbDeviceTag leds[7];                       // controllable led groupsstatic WbDeviceTag lphalanx[PHALANX_MAX];
static WbDeviceTag rphalanx[PHALANX_MAX];         // right hand motors
static WbDeviceTag lphalanx[PHALANX_MAX];         // left hand motors
static WbDeviceTag RShoulderPitch;
static WbDeviceTag LShoulderPitch;

// motion file handles
static WbMotionRef hand_wave, forwards, backwards, side_step_left, side_step_right, turn_left_60, turn_right_60, tai_chi,
  wipe_forehead;
static WbMotionRef currently_playing = NULL;

static double maxPhalanxMotorPosition[PHALANX_MAX];
static double minPhalanxMotorPosition[PHALANX_MAX];

static void find_and_enable_devices() {
  // camera
  CameraTop = wb_robot_get_device("CameraTop");
  CameraBottom = wb_robot_get_device("CameraBottom");
  wb_camera_enable(CameraTop, 4 * time_step);
  wb_camera_enable(CameraBottom, 4 * time_step);

  // accelerometer
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  // gyro
  gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, time_step);

  // inertial unit
  inertial_unit = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(inertial_unit, time_step);

  // ultrasound sensors
  us[0] = wb_robot_get_device("Sonar/Left");
  us[1] = wb_robot_get_device("Sonar/Right");
  int i;
  for (i = 0; i < 2; i++)
    wb_distance_sensor_enable(us[i], time_step);

  // foot sensors
  fsr[0] = wb_robot_get_device("LFsr");
  fsr[1] = wb_robot_get_device("RFsr");
  wb_touch_sensor_enable(fsr[0], time_step);
  wb_touch_sensor_enable(fsr[1], time_step);

  // foot bumpers
  lfoot_lbumper = wb_robot_get_device("LFoot/Bumper/Left");
  lfoot_rbumper = wb_robot_get_device("LFoot/Bumper/Right");
  rfoot_lbumper = wb_robot_get_device("RFoot/Bumper/Left");
  rfoot_rbumper = wb_robot_get_device("RFoot/Bumper/Right");
  wb_touch_sensor_enable(lfoot_lbumper, time_step);
  wb_touch_sensor_enable(lfoot_rbumper, time_step);
  wb_touch_sensor_enable(rfoot_lbumper, time_step);
  wb_touch_sensor_enable(rfoot_rbumper, time_step);

  // There are 7 controlable LED groups in Webots
  leds[0] = wb_robot_get_device("ChestBoard/Led");
  leds[1] = wb_robot_get_device("RFoot/Led");
  leds[2] = wb_robot_get_device("LFoot/Led");
  leds[3] = wb_robot_get_device("Face/Led/Right");
  leds[4] = wb_robot_get_device("Face/Led/Left");
  leds[5] = wb_robot_get_device("Ears/Led/Right");
  leds[6] = wb_robot_get_device("Ears/Led/Left");

  // get phalanx motor tags
  // the real Nao has only 2 motors for RHand/LHand
  // but in Webots we must implement RHand/LHand with 2x8 motors
  for (i = 0; i < PHALANX_MAX; i++) {
    char name[32];
    sprintf(name, "LPhalanx%d", i + 1);
    lphalanx[i] = wb_robot_get_device(name);
    sprintf(name, "RPhalanx%d", i + 1);
    rphalanx[i] = wb_robot_get_device(name);

    // assume right and left hands have the same motor position bounds
    maxPhalanxMotorPosition[i] = wb_motor_get_max_position(rphalanx[i]);
    minPhalanxMotorPosition[i] = wb_motor_get_min_position(rphalanx[i]);
  }

  // shoulder pitch motors
  RShoulderPitch = wb_robot_get_device("RShoulderPitch");
  LShoulderPitch = wb_robot_get_device("LShoulderPitch");

  // keyboard
  wb_keyboard_enable(10 * time_step);
}

// load motion files
static void load_motion_files() {
  hand_wave = wbu_motion_new("../../motions/HandWave.motion");
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left_60 = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right_60 = wbu_motion_new("../../motions/TurnRight60.motion");
  tai_chi = wbu_motion_new("../../motions/TaiChi.motion");
  wipe_forehead = wbu_motion_new("../../motions/WipeForehead.motion");
}

static void start_motion(WbMotionRef motion) {
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);

  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}

static double clamp(double value, double min, double max) {
  if (min > max) {
    assert(0);
    return value;
  }
  return value < min ? min : value > max ? max : value;
}

// the accelerometer axes are oriented as on the real robot
// however the sign of the returned values may be opposite
static void get_data() {
  const double *acc = wb_accelerometer_get_values(accelerometer);
  const double *vel = wb_gyro_get_values(gyro);
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

  printf("acceleration: [ x y z ] = [%f %f %f]\n", acc[0], acc[1], acc[2]);
  printf("angular velocity: [ x y ] = [%f %f]\n", vel[0], vel[1]);
  printf("roll/pitch/yaw: = [%f %f %f]\n", rpy[0], rpy[1], rpy[2]);
}

static void print_foot_sensors() {
  const double *fsv[2] = {wb_touch_sensor_get_values(fsr[0]), wb_touch_sensor_get_values(fsr[1])};  // force sensor values

  double l[4], r[4];
  double newtonLeft = 0, newtonRight = 0;

  // The coefficients were calibrated against the real
  // robot so as to obtain realistic sensor values.
  l[0] = fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1];  // Left Foot Front Left
  l[1] = fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1];  // Left Foot Front Right
  l[2] = fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1];  // Left Foot Rear Right
  l[3] = fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1];  // Left Foot Rear Left

  r[0] = fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1];  // Right Foot Front Left
  r[1] = fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1];  // Right Foot Front Right
  r[2] = fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1];  // Right Foot Rear Right
  r[3] = fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1];  // Right Foot Rear Left

  int i;
  for (i = 0; i < 4; ++i) {
    l[i] = clamp(l[i], 0, 25);
    r[i] = clamp(r[i], 0, 25);
    newtonLeft += l[i];
    newtonRight += r[i];
  }

  printf("----------foot sensors----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  front\n", l[0], l[1], r[0], r[1]);
  printf("|        | |        |\n");
  printf("|%3.1f  %3.1f| |%3.1f  %3.1f|  back\n", l[3], l[2], r[3], r[2]);
  printf("+--------+ +--------+\n");
  printf("total: %g Newtons, %g kilograms\n", newtonLeft + newtonRight, (newtonLeft + newtonRight) / 9.81);
}

static void print_foot_bumpers() {
  int ll = (int)wb_touch_sensor_get_value(lfoot_lbumper);
  int lr = (int)wb_touch_sensor_get_value(lfoot_rbumper);
  int rl = (int)wb_touch_sensor_get_value(rfoot_lbumper);
  int rr = (int)wb_touch_sensor_get_value(rfoot_rbumper);

  printf("----------foot bumpers----------\n");
  printf("   left       right\n");
  printf("+--------+ +--------+\n");
  printf("|%d      %d| |%d      %d|\n", ll, lr, rl, rr);
  printf("|        | |        |\n");
  printf("|        | |        |\n");
  printf("+--------+ +--------+\n");
}

static void print_ultrasound_sensors() {
  double dist[2];
  int i;
  for (i = 0; i < 2; i++)
    dist[i] = wb_distance_sensor_get_value(us[i]);

  printf("-----ultrasound sensors-----\n");
  printf("left: %f m, right %f m\n", dist[0], dist[1]);
}

static void print_camera_image(WbDeviceTag camera) {
  const int SCALED = 2;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);

  // read rgb pixel values from the camera
  const unsigned char *image = wb_camera_get_image(camera);

  printf("----------camera image (gray levels)---------\n");
  printf("original resolution: %d x %d, scaled to %d x %d\n", width, height, width / SCALED, height / SCALED);

  int y, x;
  char *line = malloc(width / SCALED + 1);
  line[width / SCALED] = 0;  // add line termination
  for (y = 0; y < height; y += SCALED) {
    int count = 0;
    for (x = 0; x < width; x += SCALED) {
      unsigned char gray = wb_camera_image_get_gray(image, width, x, y);
      line[count++] = '0' + gray * 9 / 255;
    }
    line[count++] = 0;
    printf("%s\n", line);
  }
  free(line);
}

// ====================================================== //
static void set_all_leds_color(int rgb) {
  // these leds take RGB values
  int i;
  for (i = 0; i < 5; i++)
    wb_led_set(leds[i], rgb);

  // ear leds are single color (blue)
  // and take values between 0 - 255
  wb_led_set(leds[5], rgb & 0xff);
  wb_led_set(leds[6], rgb & 0xff);
}

static void set_hands_angle(double angle) {
  // we must activate the 8 phalanx motors
  int j;
  for (j = 0; j < PHALANX_MAX; j++) {
    double clampedAngle = angle;
    if (clampedAngle > maxPhalanxMotorPosition[j])
      clampedAngle = maxPhalanxMotorPosition[j];
    else if (maxPhalanxMotorPosition[j] < minPhalanxMotorPosition[j])
      clampedAngle = minPhalanxMotorPosition[j];

    if (rphalanx[j])
      wb_motor_set_position(rphalanx[j], clampedAngle);
    if (lphalanx[j])
      wb_motor_set_position(lphalanx[j], clampedAngle);
  }
}


static void terminate() {
  wb_robot_cleanup();
}

static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
}

enum led_color {
  OFF = 0,
  RED,
  GREEN,
  BLUE
};

void change_color(enum led_color color) {
  switch (color) {
    case RED:
      set_all_leds_color(0xff0000);  // red
      break;
    case GREEN:
      set_all_leds_color(0x00ff00);  // green
      break;
    case BLUE:
      set_all_leds_color(0x0000ff);  // blue
      break;
    case OFF:
      set_all_leds_color(0x000000);  // off
      break;
  }
}

// data pipe
typedef struct receive_data {
  int face_emotion; // 顔の表情 (0: OFF, 1: RED (angry), 2: GREEN (happy), 3: BLUE (sad))
  bool hand_wave_enable; // 手を振る
  bool tai_chi_enable; // 体操
  bool wipe_enable; // 汗を拭く
  float hands_angle; // 手の角度 set_hands_angle(0.96);
} receive_data;

void action_from_rest_C()
{
  receive_data input_data;
  if (global_parse_data.method == PUT || global_parse_data.method == POST) {
    // if global_parse_data.target == "/face_emotion"
    if (strcmp(global_parse_data.target, "/face_emotion") == 0) {
      // if global_parse_data.data == "0"
      if (strcmp(global_parse_data.data, "0") == 0) {
        change_color(OFF);
      } else if (strcmp(global_parse_data.data, "1") == 0) {
        change_color(RED);
      } else if (strcmp(global_parse_data.data, "2") == 0) {
        change_color(GREEN);
      } else if (strcmp(global_parse_data.data, "3") == 0) {
        change_color(BLUE);
      }
    } else if (strcmp(global_parse_data.target, "/hand_wave_enable") == 0) {
      if (strcmp(global_parse_data.data, "0") == 0) {
        input_data.hand_wave_enable = false;
      } else if (strcmp(global_parse_data.data, "1") == 0) {
        input_data.hand_wave_enable = true;
      }
    } else if (strcmp(global_parse_data.target, "/tai_chi_enable") == 0) {
      if (strcmp(global_parse_data.data, "0") == 0) {
        input_data.tai_chi_enable = false;
      } else if (strcmp(global_parse_data.data, "1") == 0) {
        input_data.tai_chi_enable = true;
      }
    } else if (strcmp(global_parse_data.target, "/wipe_enable") == 0) {
      if (strcmp(global_parse_data.data, "0") == 0) {
        input_data.wipe_enable = false;
      } else if (strcmp(global_parse_data.data, "1") == 0) {
        input_data.wipe_enable = true;
      }
    } else if (strcmp(global_parse_data.target, "/hands_angle") == 0) {
        input_data.hands_angle = atof(global_parse_data.data);
    }

    // reset global_parse_data
    global_parse_data.method = 0;
    strcpy(global_parse_data.target, "");
    strcpy(global_parse_data.data, "");
  }
}

// main function
int main() {
  // create rest_c
  restful_server_socket rest_server;
  create_restful_server_socket(&rest_server, 5000);

  // call this before any other call to a Webots function
  wb_robot_init();

  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();

  // initialize stuff
  find_and_enable_devices();
  load_motion_files();

  while (1) {
    action_from_rest_C();
    printf("method: %d, target: %s, data: %s\n",
    global_parse_data.method,
    global_parse_data.target,
    global_parse_data.data);
    sleep(1);

    simulation_step();
    // key = wb_keyboard_get_key();
  }

  return 0;
}
