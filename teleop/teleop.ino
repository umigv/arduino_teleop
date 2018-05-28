#include "RoboClaw.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// BEGIN CONSTS
//TODO: MAKE SURE THIS ADDRESS MATCHES THAT OF THE CONTROLLER
const uint8_t ROBOCLAW0 = 0x80;
const uint8_t ROBOCLAW1 = 0x81;
const bool REVERSE0 = false;
const bool REVERSE1 = true;
const long ROBOCLAW_BAUD_RATE = 115200;
const long ROBOCLAW_TIMEOUT = 10000; 
const long CONTROL_TIMEOUT = 1000; //ms to wait  before killing motors
const float PPR = 1440; // encoder pulses per rotation
const float TRACK = 0.635; // Robot thiccness in METERS
const float WHEEL_RADIUS = 0.1651; // in METERS
const float velToPPS = PPR*(1/(2*M_PI))*(1/WHEEL_RADIUS);
// END CONSTS

// Use uno's Serial1 (same as sabertooth)
RoboClaw roboclaw(&Serial1,ROBOCLAW_TIMEOUT);
unsigned long lastData = 0;

void cmdVelCallback(const geometry_msgs::Twist&);
ros::NodeHandle handle;
ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &cmdVelCallback);

void cmdVelCallback(const geometry_msgs::Twist &twist) {
  // calculations are done with the following assumptions:
  // lin. vel (linear) is in m/s
  // ang. vel (spin) is in rad/s
  // ROBOT_RADIUS is in meters and is 1/2 the distance between the 2 wheels
  lastData = millis();
  const float linear = twist.linear.x;
  const float spin = twist.angular.z;
  // t is track
  // [ 1/2  1/2 ][vl] = [vx]
  // [-1/2t 1/2t][wz] = [wz]
  // by inversion...
  // [1 -t][vx] = [vl]
  // [1  t][wz] = [vr]
  const float vLeft = linear - spin * TRACK;
  const float vRight = linear + spin * TRACK;
  vLeft = REVERSE0 ? -vLeft : vLeft;
  vRight = REVERSE1 ? -vRight : vRight;
  
  roboclaw.SpeedM1M2(ROBOCLAW0, vLeft*velToPPS, vLeft*velToPPS);
  roboclaw.SpeedM1M2(ROBOCLAW1, vRight*velToPPS, vRight*velToPPS);
}

void setup() {
  //Communicate at 38400bps
  roboclaw.begin(ROBOCLAW_BAUD_RATE);
  handle.initNode();
  handle.subscribe(subscriber);
}

void loop(){
  handle.spinOnce();
  if(millis() - lastData >= CONTROL_TIMEOUT){
    roboclaw.SpeedM1M2(ROBOCLAW0, 0, 0);
    roboclaw.SpeedM1M2(ROBOCLAW1, 0, 0);
  }
}
