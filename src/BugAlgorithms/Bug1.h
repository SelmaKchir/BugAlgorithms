#ifndef __BUG1_h__
#define __BUG1_h__



#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <stdlib.h>
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>
#include <numeric>

#include "BugAlgorithm.h"

namespace navigation{
class Bug1 : public BugAlgorithm{
private:
geometry_msgs::Point hitPoint;

public: 
bool found;
double minDistToGoal;
Bug1(const std::string&);
//----- Obstacle Avoidance conditions ----//
void findLeavePoint(geometry_msgs::Point, geometry_msgs::Point); 
bool leavePointFound();
bool researchComplete(geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point); 
void goToLeavePoint(geometry_msgs::Point);
void setMinDistToGoal(double);
double getMinDistToGoal();

};
}
ORO_CREATE_COMPONENT(navigation::Bug1)
#endif
