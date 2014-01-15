#include "Bug2.h"
using namespace navigation;

Bug2::Bug2(const std::string& name) : BugAlgorithm(name) {
		found = false;
		direction = BugAlgorithm::CLOCKWISE;
}

bool Bug2::leavePointFound() {
	return found;
}

void Bug2::findLeavePoint(geometry_msgs::Point robotPos, geometry_msgs::Point HP) {
	//cout<<"goal position from findLP : "<<goalPosition.x<<", "<<goalPosition.y<<"\n";
	 if (isOnMline(robotPos) && closerToGoal(robotPos, HP)&& distanceEuclid(robotPos, HP) > 2.0)
		found = true;
}

bool Bug2::closerToGoal(geometry_msgs::Point p1, geometry_msgs::Point p2){
	bool closer = false;
	//checks if the current robot position is closer to the goal than the hitPoint
	if(distanceEuclid(p1,goalPosition) <  distanceEuclid(p2,goalPosition)){
			closer = true;
	}
	cout<<"closer to goal than the hit point?"<<closer<<"\n";
	return closer;
}



bool Bug2::isOnMline(geometry_msgs::Point p){
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


bool Bug2::researchComplete(geometry_msgs::Point robotPos, geometry_msgs::Point HP, geometry_msgs::Point goalPos){
	return leavePointFound();
}


void Bug2::goToLeavePoint(geometry_msgs::Point p){}





