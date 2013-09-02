#ifndef __TASK_DISTANCE_SENSOR_h__
#define __TASK_DISTANCE_SENSOR_h__

#include <rtt/TaskContext.hpp>
#include <sensor_msgs/LaserScan.h>
#include <stdlib.h>
#include <rtt/Component.hpp>
#include <iostream>
#include <vector>
#include <rtt/Operation.hpp>
#include "DistanceSensor.h"

using namespace std;
using namespace RTT;

class TaskDistanceSensor: public RTT::TaskContext, public DistanceSensor{

public: 
TaskDistanceSensor(std::string& name):
	TaskContext(name){
	  this->provides("distanceSensor")->addOperation("getDistanceToWall", &DistanceSensor::getDistanceToWall, this, RTT::ClientThread).doc("returns the distance to the closest obstacle");
	 this->provides("distanceSensor")->addOperation("obstacleInWay", &DistanceSensor::obstacleInWay, this, RTT::ClientThread).doc("checks whether an obstacle is found on the way of the robot to its goal");
	  this->provides("distanceSensor")->addOperation("obstacleInFrontOfTheRobot", &DistanceSensor::obstacleInFrontOfTheRobot, this, RTT::ClientThread).doc("checks whether an obstacle is found in the front of the robot");
	  this->provides("distanceSensor")->addOperation("obstacleOnTheLeft", &DistanceSensor::obstacleOnTheLeft, this, RTT::ClientThread).doc("checks whether an obstacle is detected on the left side of the robot");
	  this->provides("distanceSensor")->addOperation("obstacleOnTheRight", &DistanceSensor::obstacleOnTheRight, this, RTT::ClientThread).doc("checks whether an obstacle is detected on the right side of the robot");		
	  this->provides("distanceSensor")->addOperation("getSafeDistance", &DistanceSensor::getSafeDistance, this, RTT::ClientThread).doc("returns the allowed distance to the closest encountered obstacle");
}

};
#endif
