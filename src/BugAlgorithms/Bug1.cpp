#include "Bug1.h"
using namespace navigation;

Bug1::Bug1(const std::string& name) : BugAlgorithm(name) {
	found = false;
	direction = BugAlgorithm::CLOCKWISE;
	startPosition.x = 0.0;
        startPosition.y = 0.0;
}


void Bug1::setMinDistToGoal(double value){
	minDistToGoal = value;
}


double Bug1::getMinDistToGoal(){
	return minDistToGoal;
}

bool Bug1::leavePointFound() {
return found;
}

void Bug1::findLeavePoint(geometry_msgs::Point robotPos, geometry_msgs::Point HP) {

static double distMin = distanceEuclid(HP, goalPosition);
if(distanceEuclid(robotPos, goalPosition) < distMin){
	distMin = distanceEuclid(robotPos, goalPosition);
	setMinDistToGoal(distMin);
	recordLeavePoint(robotPos);
	found = true;
}
}

bool Bug1::researchComplete(geometry_msgs::Point robotPos, geometry_msgs::Point HP, geometry_msgs::Point goalPos){
	bool res = completeCycleAroundObstacle(robotPos, HP);
/*	cout<<"research complete? "<<res<<"\n";
	cout<<"hit point from research complete: "<<HP.x<<","<<HP.y<<"\n";
*/
	return res;        
}


void Bug1::goToLeavePoint(geometry_msgs::Point p){
/*bool dir = false;
if(getTotalPath() / 2 - getMinPathToPoint() < 0){
		dir = true;
	}
	else{
		dir = false;
	}
        cout << "direction : "<<dir;
	wallFollowingToPoint(p, dir);
	geometry_msgs::Twist cmd;
	cmd.linear.x = 1;
	outport.write(cmd);
	faceGoal(getPosition(), goalPosition);
	totalPath = 0;
	distToLeavePoint = 0;
	sleep(1);
	if(obstacleInWay()){
		if(distance(getPosition(), goalPosition) > DIST_MIN){
			cout <<"goal unreachable!!";
			exit(0);
		}
	}
*/
 	wallFollowingToPoint(p, COUNTERCLOCKWISE);
	cout<<"leave point reached \n";
        faceGoal();
	sleep(1);
}





