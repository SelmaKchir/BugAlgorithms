#ifndef __ODOMETRY_ADAPTER_h__
#define __ODOMETRY_ADAPTER_h__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>
#include <numeric>
#include <vector>
#include "./TaskLocalisation.h"

using namespace std;
using namespace RTT;
namespace navigation{
class OdometryAdapter : public TaskLocalisation{
public:
	InputPort<nav_msgs::Odometry> odometry_port; 

OdometryAdapter(std::string&);
//----- localization method ------//
geometry_msgs::Point getPosition();
geometry_msgs::Quaternion getOrientation();
};
}
ORO_CREATE_COMPONENT(navigation::OdometryAdapter)
#endif
