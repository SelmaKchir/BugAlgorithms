#ifndef __IR_Adapter_h__
#define __IR_Adapter_h__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>
#include <numeric>
#include <vector>
#include "./TaskDistanceSensor.h"
using namespace std;
using namespace RTT;
namespace navigation{
class IRAdapter:  public TaskDistanceSensor{
private:
InputPort<sensor_msgs::Range> IR_right_port;  //Access to sensors
InputPort<sensor_msgs::LaserScan> Laser_left_port;  //Access to sensors
double samples_right;
double samples_laser;
double nb_sensors;
double laser_range_max;
double IR_range_max;
public: 
IRAdapter(std::string&);
//----- Robot sensors ------//
//----- Robot sensors ------//
bool obstacleInFrontOfTheRobot();
bool obstacleInWay();
bool obstacleFreeWay();
vector<double> getDistanceSensors();
bool obstacleOnTheRight();
bool obstacleOnTheLeft();
double getDistanceToWall();
double getSafeDistance();
};
}
ORO_CREATE_COMPONENT(navigation::IRAdapter)
#endif
