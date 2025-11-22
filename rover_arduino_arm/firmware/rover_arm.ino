// rover_arm.ino for Arduino Mega
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

const int NUM_JOINTS = 5;
// example pins for each A4988: step, dir
int stepPins[NUM_JOINTS] = {2,3,4,5,6};
int dirPins[NUM_JOINTS]  = {7,8,9,10,11};
int homePins[NUM_JOINTS] = {22,23,24,25,26}; // limit switches

volatile long joint_steps[NUM_JOINTS] = {0};
float joint_angles[NUM_JOINTS] = {0.0}; // computed from steps via steps_per_deg
float steps_per_deg[NUM_JOINTS] = {16.0, 16.0, 16.0, 16.0, 16.0}; // tune

sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("arm/joint_states", &joint_state_msg);

std_msgs::Float32MultiArray target_msg;
void arm_cmd_cb(const std_msgs::Float32MultiArray& msg) {
  // msg: target angles in degrees
  for (int i=0;i<NUM_JOINTS;i++){
    float targ = msg.data[i];
    long target_steps = (long)(targ * steps_per_deg[i]);
    // simple stepper move: set dir pin and pulse stepPin until at target
    long delta = target_steps - joint_steps[i];
    bool dir = delta >= 0;
    digitalWrite(dirPins[i], dir ? HIGH : LOW);
    long steps = abs(delta);
    for (long s=0; s<steps; s++){
      digitalWrite(stepPins[i], HIGH);
      delayMicroseconds(800); // tune microstep delay
      digitalWrite(stepPins[i], LOW);
      delayMicroseconds(800);
      joint_steps[i] += dir ? 1 : -1;
    }
  }
}
ros::Subscriber<std_msgs::Float32MultiArray> arm_cmd_sub("arm/command", &arm_cmd_cb);

void setup(){
  for (int i=0;i<NUM_JOINTS;i++){
    pinMode(stepPins[i], OUTPUT);
    pinMode(dirPins[i], OUTPUT);
    pinMode(homePins[i], INPUT_PULLUP);
  }
  nh.initNode();
  nh.advertise(joint_state_pub);
  nh.subscribe(arm_cmd_sub);
}

void loop(){
  // publish joint_state periodically
  joint_state_msg.name = {"joint1","joint2","joint3","joint4","joint5"};
  joint_state_msg.position_length = NUM_JOINTS;
  for (int i=0;i<NUM_JOINTS;i++){
    joint_angles[i] = joint_steps[i] / steps_per_deg[i];
  }
  joint_state_msg.position = joint_angles;
  joint_state_pub.publish(&joint_state_msg);
  nh.spinOnce();
  delay(50);
}
