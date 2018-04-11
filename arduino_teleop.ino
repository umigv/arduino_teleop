#include "RoboClaw.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

// BEGIN CONSTS
//TODO: MAKE SURE THIS ADDRESS MATCHES THAT OF THE CONTROLLER
const uint8_t ROBOCLAW0 = 0x80;
const uint8_t ROBOCLAW1 = 0x81;
const long ROBOCLAW_BAUD_RATE = 38400;
const long TIMEOUT_VALUE_MS = 10000;
const float PULSES_PER_ROTATION = 360; // For the encoders
const float ROBOT_RADIUS = 0.3175; // Robot thiccness/2 in METERS
const float WHEEL_RADIUS = 0.1651; // in METERS
const float velToPPS = (360)*(1/(2*M_PI))*(1/WHEEL_RADIUS);
// END CONSTS

// Use uno's Serial1 (same as sabertooth)
RoboClaw roboclaw(&Serial1,TIMEOUT_VALUE_MS);

void cmdVelCallback(const geometry_msgs::Twist&);
ros::NodeHandle handle;
ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &cmdVelCallback);

void cmdVelCallback(const geometry_msgs::Twist &twist) {
  // calculations are done with the following assumptions:
  // lin. vel (linear) is in m/s
  // ang. vel (spin) is in rad/s
  // ROBOT_RADIUS is in meters and is 1/2 the distance between the 2 wheels
  float linear = twist.linear.x;
  float spin = twist.angular.z;
  float vLeft = linear - spin*ROBOT_RADIUS/2;
  float vRight = linear + spin*ROBOT_RADIUS/2;
  roboclaw.SpeedM1(ROBOCLAW0, vLeft*velToPPS);
  roboclaw.SpeedM2(ROBOCLAW0, vLeft*velToPPS);
  roboclaw.SpeedM1(ROBOCLAW1, vRight*velToPPS);
  roboclaw.SpeedM2(ROBOCLAW1, vRight*velToPPS);
}

void setup() {
  //Communicate at 38400bps
  roboclaw.begin(ROBOCLAW_BAUD_RATE);
  handle.initNode();
  handle.subscribe(subscriber);
}

void loop(){
  handle.spinOnce();
}
