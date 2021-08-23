#pragma once

#define PIN_PWM_R 4
#define PIN_DIR_R 5
#define PIN_PWM_L 9
#define PIN_DIR_L 6

#define PIN_ENC_R1 3
#define PIN_ENC_R2 2
#define PIN_ENC_L1 18
#define PIN_ENC_L2 19

#define PIN_LED 10

#define PIN_IR_R A4
#define PIN_IR_F A3
#define PIN_IR_L A2

#define IR_SENSOR_MODEL 1

#define NUM_LEDS 2 //Ninja Extra Task
#define BRIGHTNESS 64
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

double period_sec = 0.1;

SharpIR sensor_r(IR_SENSOR_MODEL, PIN_IR_R), sensor_f(IR_SENSOR_MODEL, PIN_IR_F), sensor_l(IR_SENSOR_MODEL, PIN_IR_L);
Encoder encoder_r(PIN_ENC_R1, PIN_ENC_R2), encoder_l(PIN_ENC_L1, PIN_ENC_L2);

std_msgs::Float32 sensor_dist_r, sensor_dist_f, sensor_dist_l;
geometry_msgs::Pose2D pose_2d, pose_msg;

std_msgs::UInt8MultiArray rgb_leds_msg;
geometry_msgs::Twist cmd_vel_msg;

double r = 0.016, b = 0.094, c = 8192;
double v_robot_desired = 0, w_robot_desired = 0, v_robot_real = 0, w_robot_real = 0;
double w_wheel_r_desired = 0, w_wheel_l_desired = 0, w_wheel_r_real = 0, w_wheel_l_real = 0;
long old_val_encoder_r = 0, old_val_encoder_l = 0, NR = 0, NL = 0;

float Kp = 5, Ki = 0, Kd = 0, Pr, Ir, Dr, Pl, Il, Dl; //params for PID
float lastErr_r_wheel = 0, lastErr_l_wheel = 0;

static float pose[3] = {0.0};
