#ifndef __DISTANCE_SENSOR_h__
#define __DISTANCE_SENSOR_h__

#include <sensor_msgs/LaserScan.h>
#include <vector>

using namespace std;
class DistanceSensor{

public: 
virtual bool obstacleInWay() = 0;
virtual bool obstacleInFrontOfTheRobot() = 0;
virtual bool obstacleFreeWay() = 0;
virtual vector<double> getDistanceSensors() = 0;
virtual bool obstacleOnTheRight() = 0;
virtual bool obstacleOnTheLeft() = 0;
virtual double getDistanceToWall() = 0;
virtual double getSafeDistance() = 0;
//virtual bool goalIsReachable() = 0;
};
#endif
