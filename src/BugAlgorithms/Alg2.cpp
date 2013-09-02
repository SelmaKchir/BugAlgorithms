#include "Alg2.h"
using namespace navigation;

Alg2::Alg2(const std::string& name) : BugAlgorithm(name) {
	found = false;
	direction = BugAlgorithm::CLOCKWISE;
}


void Alg2::setMinDistToGoal(double value){
	minDistToGoal = value;
}


double Alg2::getMinDistToGoal(){
	return minDistToGoal;
}

bool Alg2::leavePointFound() {
return found;
}

void Alg2::findLeavePoint(geometry_msgs::Point robotPos, geometry_msgs::Point HP) {
geometry_msgs::Point goal;
static double distMin = distanceEuclid(HP, goal);
static geometry_msgs::Point point = HP;


if(distanceEuclid(robotPos, goal) < distMin && distanceEuclid(robotPos, HP) > 1.5){
	distMin = distanceEuclid(robotPos, goal);
	point = robotPos;
	setMinDistToGoal(distMin);
	recordLeavePoint(robotPos);
	found = true;
}
}
bool Alg2::researchComplete(geometry_msgs::Point robotPos, geometry_msgs::Point HP, geometry_msgs::Point goalPos){
return goalIsReachable(robotPos, goalPos, 3.0);
}


void Alg2::goToLeavePoint(geometry_msgs::Point p){
}






