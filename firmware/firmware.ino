#include <ros.h>
#include <math.h>
#include <stdio.h>
#include <Encoder.h>
#include <SharpIR.h>
#include <FastLED.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <rosserial_arduino/Test.h>

#include "firmware.h"

ros::NodeHandle nh;
using rosserial_arduino::Test;

ros::Publisher right_distance_pub("right_distance", &sensor_dist_r);
ros::Publisher front_distance_pub("front_distance", &sensor_dist_f);
ros::Publisher left_distance_pub("left_distance", &sensor_dist_l);
ros::Publisher pose_pub("pose", &pose_2d);

unsigned long lastTime;

void rgbLedsCallback(const std_msgs::UInt8MultiArray &msg) { rgb_leds_msg = msg; }

void setPoseCallback(const geometry_msgs::Pose2D &msg) { pose_msg = msg; }

void cmdVelCallback(const geometry_msgs::Twist &msg)
{
    v_robot_desired = msg.linear.x;
    w_robot_desired = msg.angular.z;
}

void rgbLedsServerCallback(const Test::Request &req, Test::Response &res) //Ninja Extra Task
{
    res.output = setLedsColor(atol(req.input));
}

ros::Subscriber<std_msgs::UInt8MultiArray> rgb_leds_sub("rgb_leds", &rgbLedsCallback);
ros::Subscriber<geometry_msgs::Pose2D> set_pose_sub("set_pose", &setPoseCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmdVelCallback);

ros::ServiceServer<Test::Request, Test::Response> rgb_leds_server("set_rgb_leds", &rgbLedsServerCallback); //Ninja Extra Task

void setup()
{
    nh.initNode();

    nh.advertise(right_distance_pub);
    nh.advertise(front_distance_pub);
    nh.advertise(left_distance_pub);
    nh.advertise(pose_pub);

    nh.advertiseService(rgb_leds_server);

    nh.subscribe(rgb_leds_sub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(set_pose_sub);

    encoder_r.write(0);
    encoder_l.write(0);

    FastLED.addLeds<LED_TYPE, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(BRIGHTNESS);

    lastTime = millis();
}

void loop()
{
    if (millis() - lastTime > period_sec * 1000)
    {
        lastTime = millis();
        sensor_dist_r.data = sensor_r.getDistance();
        sensor_dist_f.data = sensor_f.getDistance();
        sensor_dist_l.data = sensor_l.getDistance();

        right_distance_pub.publish(&sensor_dist_r);
        front_distance_pub.publish(&sensor_dist_f);
        left_distance_pub.publish(&sensor_dist_l);

        calcEncodersChange();
        calcCurrentPose();
        calcDesiredAngularVelocityWeels();
        setPIDWheelVelocities();

        pose_2d.x = pose[0];
        pose_2d.y = pose[1];
        pose_2d.theta = pose[2];

        pose_pub.publish(&pose_2d);

        FastLED.show();

        nh.spinOnce();
    }
}

long calcEncodersChange()
{
    long new_val_r = encoder_r.read();
    long new_val_l = encoder_l.read();

    NR = new_val_r - old_val_encoder_r;
    NL = new_val_l - old_val_encoder_l;

    old_val_encoder_r = new_val_r;
    old_val_encoder_l = new_val_l;
}

void calcDesiredAngularVelocityWeels()
{
    w_wheel_r_desired = (v_robot_desired + b * w_robot_desired / 2.0) / r;
    w_wheel_l_desired = (v_robot_desired - b * w_robot_desired / 2.0) / r;
}

void calcCurrentPose()
{
    v_robot_real = 2.0 * M_PI * r * (NR + NL) / (c * 2.0 * period_sec);
    w_robot_real = 2.0 * M_PI * r * (NR - NL) / (c * b * period_sec);

    double angle_new = atan2(sin(pose[2] + w_robot_real * period_sec), cos(pose[2] + w_robot_real * period_sec));
    double x_new = pose[0] + v_robot_real * cos(angle_new) * period_sec;
    double y_new = pose[1] + v_robot_real * sin(angle_new) * period_sec;

    pose[0] = x_new;
    pose[1] = y_new;
    pose[2] = angle_new;
}

void setVelocities(int wheel_r_velocity, int wheel_l_velocity)
{
    digitalWrite(PIN_DIR_R, wheel_r_velocity > 0 ? LOW : HIGH);
    digitalWrite(PIN_DIR_L, wheel_l_velocity > 0 ? LOW : HIGH);

    wheel_r_velocity = abs(wheel_r_velocity) > 255 ? 255 : abs(wheel_r_velocity);
    wheel_l_velocity = abs(wheel_l_velocity) > 255 ? 255 : abs(wheel_l_velocity);

    analogWrite(PIN_PWM_R, wheel_r_velocity);
    analogWrite(PIN_PWM_L, wheel_l_velocity);
}

void setPIDWheelVelocities()
{
    float err_r = w_wheel_r_desired - w_wheel_r_real;
    float err_l = w_wheel_l_desired - w_wheel_l_real;

    Pr = err_r;
    Pl = err_l;

    Ir = Ir + err_r;
    Il = Il + err_l;

    Dr = err_r - lastErr_r_wheel;
    Dl = err_l - lastErr_l_wheel;

    lastErr_r_wheel = err_r;
    lastErr_l_wheel = err_l;

    int wheel_r_velocity = Pr * Kp + Ir * Ki * period_sec + Dr * Kd / period_sec;
    int wheel_l_velocity = Pl * Kp + Il * Ki * period_sec + Dl * Kd / period_sec;

    setVelocities(wheel_r_velocity, wheel_l_velocity);
}

char *setLedsColor(int color_id) //Ninja Extra Task
{
    switch (color_id)
    {
    case 0: //OFF
        leds[1].r = 0;
        leds[1].g = 0;
        leds[1].b = 0;
        return "off";
    case 1: //white
        leds[1].r = 255;
        leds[1].g = 255;
        leds[1].b = 255;
        return "on";
    case 2: //red
        leds[1].r = 255;
        leds[1].g = 0;
        leds[1].b = 0;
        return "on";
    case 3: //yellow
        leds[1].r = 255;
        leds[1].g = 255;
        leds[1].b = 0;
        return "on";
    case 4: //green
        leds[1].r = 0;
        leds[1].g = 255;
        leds[1].b = 0;
        return "on";
    case 5: //blue
        leds[1].r = 0;
        leds[1].g = 255;
        leds[1].b = 255;
        return "on";
    case 6: //fuchsia
        leds[1].r = 255;
        leds[1].g = 0;
        leds[1].b = 255;
        return "on";
    //case 7: //disco
    //  leds[1] = ColorFromPalette( PartyColors_p, colorIndex, brightness, currentBlending);
    //   return "on";
    default:
        return "Error: unknown color id";
    }
}
