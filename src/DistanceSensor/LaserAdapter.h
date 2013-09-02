#ifndef __LASER_ADAPTER_h__
#define __LASER_ADAPTER_h__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
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
class LaserAdapter : public TaskDistanceSensor{

private:
InputPort<sensor_msgs::LaserScan> laser_port;  //Access to sensors
double samples;
double nb_sensors;
double range_max;

public: 
LaserAdapter(std::string&);
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
ORO_CREATE_COMPONENT(navigation::LaserAdapter)
#endif
