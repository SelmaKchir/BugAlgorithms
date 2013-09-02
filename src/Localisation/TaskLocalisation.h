#ifndef __TASK_LOCALISATION_h__
#define __TASK_LOCALISATION_h__

#include <rtt/TaskContext.hpp>
#include <geometry_msgs/Point.h>
#include <stdlib.h>
#include <iostream>
#include <rtt/Operation.hpp>
#include "Localisation.h"

using namespace std;
using namespace RTT;
class TaskLocalisation : public RTT::TaskContext, public Localisation{


public:
TaskLocalisation(std::string& name):
	TaskContext(name){

 this->provides("localisation")->addOperation("getPosition", &Localisation::getPosition, this, RTT::ClientThread).doc("returns the robot's current position");   
 this->provides("localisation")->addOperation("getOrientation", &Localisation::getOrientation, this, RTT::ClientThread).doc("returns the robot's current orientation");   
}
};

#endif

