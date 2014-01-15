#include "Alg1.h"
using namespace navigation;

Alg1::Alg1(const std::string& name) : BugAlgorithm(name) {
		found = false;
		direction = BugAlgorithm::CLOCKWISE;
}

bool Alg1::leavePointFound() {
	return found;
}

void Alg1::findLeavePoint(geometry_msgs::Point robotPos, geometry_msgs::Point HP) {
        if (isOnMline(robotPos) && closerToGoal(robotPos)&& distanceEuclid(robotPos, HP) > 2.0 && !isEncounteredPoint(robotPos)){
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
	        if (isOnMline(robotPos) && closerToGoal(getPosition())&& distanceEuclid(getPosition(), HP) > 2.0 && !isEncounteredPoint(robotPos)){
		recordLeavePoint(robotPos);
		found = true;
		}
		}
	}
}

bool Alg1::closerToGoal(geometry_msgs::Point p1){
bool closer = false;
		//checks if the current robot position is closer to the goal than all the points
		//ever visited on the mline
		if(hitPoint.size() > 0){
		for (vector<geometry_msgs::Point>::iterator it = hitPoint.begin(); it!=hitPoint.end(); it++) {
			if((distanceEuclid(p1,goalPosition) < distanceEuclid(*it, goalPosition))&& distanceEuclid(p1, getLastHitPoint()) > 2.0 )
				closer = true;
		}
		}

		if(leavePoint.size()> 0){
			for (vector<geometry_msgs::Point>::iterator it = leavePoint.begin(); it!=leavePoint.end(); it++) {
				if((distanceEuclid(p1,goalPosition) < distanceEuclid(*it, goalPosition)) && distanceEuclid(p1, getLastHitPoint()) > 2.0 )
					closer = true;
			}
		}
		return closer;
}



bool Alg1::isOnMline(geometry_msgs::Point p){
	bool mline = false;
	double distErr = 0.5;
	double distanceToMline;
	double a = goalPosition.y-startPosition.y;
	double b = startPosition.x-goalPosition.x;
	double c = startPosition.y*(goalPosition.x-startPosition.x)-startPosition.x*(goalPosition.y-startPosition.y);
	double x0 = p.x;
	double y0 = p.y;
	//calculate the distance between the robot and the Mline
	distanceToMline = abs( a*x0 + b*y0 + c ) / sqrt( a*a+b*b );
	if (distanceToMline < distErr){
		//to ensure that the robot is on the Mline, it means between the start position and the goal position
		if(startPosition.x < goalPosition.x){
			if(p.x > startPosition.x && p.x < goalPosition.x){
				mline = true;
			}
		}
		else if(startPosition.x > goalPosition.x){
			if(p.x < startPosition.x && p.x > goalPosition.x){
				mline = true;
			}
		}
		else if(startPosition.y < goalPosition.y){
			if(p.y > startPosition.y && p.y < goalPosition.y){
				mline = true;
			}
		}
		else{
			if(p.y < startPosition.y && p.y > goalPosition.y){
				mline = true;
			}
		}
	}
	cout<<"robot on mline? "<<mline<<"\n";
        return mline;
}


bool Alg1::researchComplete(geometry_msgs::Point robotPos, geometry_msgs::Point HP, geometry_msgs::Point goalPos){
	return leavePointFound();
}


void Alg1::goToLeavePoint(geometry_msgs::Point p){}


bool Alg1::isEncounteredPoint(geometry_msgs::Point point){

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
