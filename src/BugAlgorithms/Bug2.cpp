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
double dist_margin = 1.5;

if(distanceEuclid(robotPos, goal) < distMin && distanceEuclid(robotPos, HP) > dist_margin && !isEncounteredPoint(robotPos)){
	distMin = distanceEuclid(robotPos, goal);
	point = robotPos;
	setMinDistToGoal(distMin);
	recordLeavePoint(robotPos);
	found = true;
}
//if the robot passes through an already visited point, it changes its direction to look for other points
else if(isEncounteredPoint(robotPos)){
	cout<<"Already crossed point\n";
	//return to the last hit point			
	wallFollowingToPoint(getLastHitPoint(), !direction);
	//if there is no leave point identified yet, continue wall following on the opposite direction
	if(leavePoint.size() < hitPoint.size()){
		wallFollowing(!direction);
	        if (distanceEuclid(robotPos, goal) < distMin && distanceEuclid(robotPos, HP) > dist_margin && !isEncounteredPoint(robotPos)){
		recordLeavePoint(robotPos);
		found = true;
		}
	}
}
}

bool Alg2::researchComplete(geometry_msgs::Point robotPos, geometry_msgs::Point HP, geometry_msgs::Point goalPos){
return goalIsReachable(robotPos, goalPos, 3.0);
}


void Alg2::goToLeavePoint(geometry_msgs::Point p){
}


bool Alg2::isEncounteredPoint(geometry_msgs::Point point){

    bool encountered = false;
    double distMargin = 0.5;
    double min_dist = 2.0;
	if(hitPoint.size() > 0){
	for (vector<geometry_msgs::Point>::iterator it = hitPoint.begin(); it!=hitPoint.end() - 1; ++it) {
	 	if((distanceEuclid(*it, point) <= DIST_ERROR) && distanceEuclid(getLastHitPoint(), point) > min_dist)
		 encountered = true;
	}
	}
	if(leavePoint.size() > 0){
	for (vector<geometry_msgs::Point>::iterator it = leavePoint.begin(); it!=leavePoint.end() - 1; ++it) {
 	   if((distanceEuclid(*it, point) <= DIST_ERROR) && distanceEuclid(getLastLeavePoint(), point) > min_dist)
		encountered = true;
	}
	}
	return encountered;
}



