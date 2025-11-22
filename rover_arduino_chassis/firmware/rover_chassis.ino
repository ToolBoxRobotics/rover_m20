// rover_chassis.ino
// Arduino sketch (Uno or Mega). Depends on rosserial_arduino, Adafruit_PWMServoDriver, MPU6050 library

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>   // library: jeff rowberg or compatible
// Encoder library - use attachInterrupt for each encoder
// Replace with your encoder wiring specifics

// --- Hardware constants ---
#define PCA9685_ADDR 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

const int NUM_WHEELS = 6;
const int ENCODER_PINS_A[NUM_WHEELS] = {2,3,18,19,20,21}; // example; change per hardware
const int ENCODER_PINS_B[NUM_WHEELS] = {4,5,22,23,24,25};
volatile long encoder_counts[NUM_WHEELS] = {0};
volatile int last_encoder_state[NUM_WHEELS];

// motor PWM channels on PCA9685 or direct PWM pins
const uint8_t MOTOR_PWM_CHANNEL[NUM_WHEELS] = {0,1,2,3,4,5};
// steering servo channels (4 corners; if 4 servos)
const uint8_t STEER_CHANNEL_FL = 6;
const uint8_t STEER_CHANNEL_FR = 7;
const uint8_t STEER_CHANNEL_RL = 8;
const uint8_t STEER_CHANNEL_RR = 9;

// Map servo angle to PCA9685 pulse:
int angleToPulse(int angle) {
  // angle -90..90
  int pulse = map(angle, -90, 90, 150, 600); // tune per servo
  return pulse;
}

// ROS
ros::NodeHandle nh;
std_msgs::Float32MultiArray enc_pub_msg;
ros::Publisher enc_pub("chassis/encoders", &enc_pub_msg);

void chassis_cmd_cb(const std_msgs::Float32MultiArray& msg) {
  // msg layout: [motor0_speed, ..., motor5_speed, steer_FL, steer_FR, steer_RL, steer_RR]
  for (int i=0;i<NUM_WHEELS;i++){
    float v = msg.data[i];
    // convert v -1..1 to PWM pulse width for motor controllers
    uint16_t pwmPulse = (uint16_t)map((int)(v*100), -100, 100, 0, 4095);
    pwm.setPWM(MOTOR_PWM_CHANNEL[i], 0, pwmPulse);
  }
  // steering angles
  int fl = (int)msg.data[6], fr = (int)msg.data[7], rl = (int)msg.data[8], rr = (int)msg.data[9];
  pwm.setPWM(STEER_CHANNEL_FL, 0, angleToPulse(fl));
  pwm.setPWM(STEER_CHANNEL_FR, 0, angleToPulse(fr));
  pwm.setPWM(STEER_CHANNEL_RL, 0, angleToPulse(rl));
  pwm.setPWM(STEER_CHANNEL_RR, 0, angleToPulse(rr));
}
ros::Subscriber<std_msgs::Float32MultiArray> chassis_cmd_sub("chassis/cmd", &chassis_cmd_cb);

// IMU & GPS publishers
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);
sensor_msgs::NavSatFix gps_msg;
ros::Publisher gps_pub("gps/fix", &gps_msg);

unsigned long last_enc_pub = 0;

void setup() {
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // 50Hz for servos
  nh.initNode();
  nh.advertise(enc_pub);
  nh.subscribe(chassis_cmd_sub);
  nh.advertise(&imu_pub);
  nh.advertise(&gps_pub);

  // encoders set up: attach interrupts etc.
  // MPU6050 init...
  // Serial1 for GPS (if using Mega) or SoftwareSerial for Uno
}

void loop() {
  nh.spinOnce();

  unsigned long now = millis();
  if (now - last_enc_pub > 100) {
    last_enc_pub = now;
    // publish encoder counts array
    enc_pub_msg.data.clear();
    for (int i=0;i<NUM_WHEELS;i++) enc_pub_msg.data.push_back((float)encoder_counts[i]);
    enc_pub.publish(&enc_pub_msg);

    // publish IMU (fill fields)
    // imu_pub.publish(&imu_msg);
    // publish GPS when available
  }
}
