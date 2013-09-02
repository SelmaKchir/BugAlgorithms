#ifndef __BugAlgorithm_h__
#define __BugAlgorithm_h__

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <rtt/Operation.hpp>
#include <stdlib.h>
#include <rtt/Component.hpp>
#include <iostream>
#include <math.h>
#include <numeric>

#include "../Localisation/TaskLocalisation.h"
#include "../DistanceSensor/TaskDistanceSensor.h"

using namespace std;
using namespace RTT;
namespace navigation{
class BugAlgorithm : public RTT::TaskContext{
public:
	enum {CLOCKWISE = true, COUNTERCLOCKWISE = false};
protected:
static const double DIST_MIN = 2.1;   //minimum distance between the robot and the goal
static const double DIST_ERROR = 1.5;   //minimum allowed distance between the robot and the obstacle
static const double PI = 3.1415926;    
bool firstMove; //used to avoid multiple definitions of hit points around and obstacle
bool direction, leftCheck, rightCheck, hitDefined;
double totalDistance; 
geometry_msgs::Point goalPosition; //the goal position
geometry_msgs::Point startPosition;   //start point position
geometry_msgs::Point leavePoint, hitPoint; //the point where the robot encounters an obstacle
geometry_msgs::Point oldPoint;

//------Access the odometry sensor------//
TaskContext* taskOdom;
TaskContext* taskDistanceSensor;
// get the localisation methods
OperationCaller <geometry_msgs::Point(void)> getPosition;
OperationCaller <geometry_msgs::Quaternion(void)> getOrientation;
// get the distance sensor methods
OperationCaller <double(void)> getSafeDistance;
OperationCaller <bool(void)> obstacleInFrontOfTheRobot;
OperationCaller <bool(void)> obstacleInWay;
OperationCaller <bool(void)> obstacleOnTheLeft;
OperationCaller <bool(void)> obstacleOnTheRight;
OperationCaller <double(void)> getDistanceToWall;

OutputPort<geometry_msgs::Twist> outport; 

public: 
BugAlgorithm(const std::string& name)
:   TaskContext(name),
    outport("cmd_vel"), 
    hitDefined(false),
    firstMove(true),
    totalDistance(0.0),
    getOrientation("getOrientation"),
    getPosition("getPosition"),
    getSafeDistance("getSafeDistance"),
    obstacleInFrontOfTheRobot("obstacleInFrontOfTheRobot"),
    obstacleOnTheLeft("obstacleOnTheLeft"),
    obstacleOnTheRight("obstacleOnTheRight"),
    obstacleInWay("obstacleInWay"),
    getDistanceToWall("getDistanceToWall") {
    //set the goal position
    goalPosition.x = 0.0;
    goalPosition.y = 5.0;
    //set the start position
    startPosition.x = 0.0;
    startPosition.y = -4.0;
   //add the command port     
    this->ports()->addPort(outport).doc("actuator");
    this->requires("localisation")
			->addOperationCaller(getOrientation);	
    this->requires("localisation")
			->addOperationCaller(getPosition);	

    this->requires("distanceSensor")
			->addOperationCaller(getSafeDistance);	
	
    this->requires("distanceSensor")
			->addOperationCaller(obstacleInWay);	

    this->requires("distanceSensor")
			->addOperationCaller(obstacleInFrontOfTheRobot);	

    this->requires("distanceSensor")
			->addOperationCaller(obstacleOnTheLeft);	

    this->requires("distanceSensor")
			->addOperationCaller(obstacleOnTheRight);	

    this->requires("distanceSensor")
			->addOperationCaller(getDistanceToWall);	

    //additional control to avoid performing the same action several times
    leftCheck=false; 
    rightCheck=false;
}


// computation method
double distanceEuclid(geometry_msgs::Point positionA, geometry_msgs::Point positionB){
    double x_diff = positionA.x - positionB.x;
    double y_diff = positionA.y - positionB.y;
return	sqrt((x_diff * x_diff) + (y_diff * y_diff));// + (z_diff * z_diff));
}


void setTotalDistance(double value){
	totalDistance = value;
}

double getTotalDistance(){
	return totalDistance;
}
// ------ virtual methods ----- //
virtual void goToLeavePoint(geometry_msgs::Point) {}
virtual bool researchComplete(geometry_msgs::Point, geometry_msgs::Point, geometry_msgs::Point) {} 
virtual bool leavePointFound() {}
virtual void findLeavePoint(geometry_msgs::Point, geometry_msgs::Point) {}
//----- Robot actions ------//
void faceGoal(){
		geometry_msgs::Twist cmd;
		double cosTeta, angle;
		double relativeVectorX;
		double relativeVectorY;
		double directionX, directionY;
		double x,y;
		goAhead();
		//calculate a vector from current position to target
		relativeVectorX = goalPosition.x - getPosition().x;
		relativeVectorY = goalPosition.y - getPosition().y;
	//	cout<<"In faceGoal(): old point ("<<oldPoint.x<<", "<<oldPoint.y<<")\n";
	//	cout<<"In faceGoal(): current position ("<<getPosition().x<<", "<<getPosition().y<<")\n";

		do{
		x=getPosition().x-oldPoint.x;
		y=getPosition().y-oldPoint.y;
		}
		while(x==0&& y==0);

	//	cout<<"In faceGoal(): current position ("<<getPosition().x<<", "<<getPosition().y<<")\n";
		double directionRobot = atan2(y, x);
		double yRot = 5*(atan2(relativeVectorY,relativeVectorX)-directionRobot);
		angle = atan2(relativeVectorY,relativeVectorX)*180/PI;
	//	cout<<">>>>>>>turn to goal -- angle: "<<yRot<<endl;
		if(yRot!=0)
		{
		//nav_msgs::Odometry position_msg; 
	   	//odometry_port.read(position_msg);
		//double ancienZ= getOrientation();
		//cmd.linear.x = 0;
		cmd.angular.z = yRot;
		outport.write(cmd);

		}
}

void goAhead(){
 geometry_msgs::Twist cmd;
 //   oldPoint.x = getPosition().x;
 //   oldPoint.y = getPosition().y;
    cmd.linear.x = 1;
    outport.write(cmd);
}
void rightHand(){
	geometry_msgs::Twist cmd;
	double distanceToWall = getDistanceToWall();
//	cout<<"distance to wall :"<<distanceToWall<<"\n";
	double new_error = distanceToWall - DIST_ERROR;
//	cout<<"new error :"<<new_error<<"\n";
	if(obstacleOnTheRight()){
		if(new_error > getSafeDistance()) {
			if(leftCheck&&rightCheck){
					goAhead();
					leftCheck = false;
					rightCheck = false;
					//updateData();
			}
			else{
					leftCheck=true;
					cmd.angular.z = -0.5;
					outport.write(cmd);
			}
		}
		if(new_error >= 0 && new_error <= getSafeDistance()) {
			goAhead();
			rightCheck=false;
			leftCheck=false;
			//updateData();
		}
		if(new_error < 0) {
			rightCheck=true;
			cmd.angular.z = 0.3;
			outport.write(cmd);
		}
	}
	else {
		cmd.angular.z = 0.5;
		rightCheck=true;
		outport.write(cmd);
	}
}
void leftHand(){
geometry_msgs::Twist cmd;
	double distanceToWall = getDistanceToWall();
	double new_error = distanceToWall - DIST_ERROR;

	if(obstacleOnTheLeft()){
		if(new_error > getSafeDistance()) {
			if(leftCheck&&rightCheck)
			{
				goAhead();
				//updateData();
				leftCheck=false;
				rightCheck=false;
			}
			else{
				leftCheck=true;
				cmd.angular.z = 0.5;
				outport.write(cmd);
			}
		}
		if(new_error >= 0 && new_error <= getSafeDistance()) {
			goAhead();
			//updateData();
			rightCheck=false;
			leftCheck=false;

		}
		if(new_error < 0) {
			rightCheck=true;
			cmd.angular.z = -0.3;
			outport.write(cmd);
		}

	}
	else {
		cmd.angular.z = -0.5;
		rightCheck=true;
		outport.write(cmd);
	}

}
void wallFollowing(bool direction) {
oldPoint.x = getPosition().x;
oldPoint.y = getPosition().y;
if(direction == true){
			rightHand();
		}
		else if (direction == false){
			leftHand();
		}
}
void wallFollowingToPoint(geometry_msgs::Point p, bool dir){
	double distanceMargin = 0.5;
	while(distanceEuclid(getPosition(), p) > distanceMargin){
		wallFollowing(dir);
	}

	cout<<"at leave point \n";
	halt();
	cout<<"stop ...\n";
	faceGoal();
	cout<<"face goal...\n";
	goAhead();
}

void motionToGoal(){
geometry_msgs::Twist cmd;
		oldPoint.x = getPosition().x;
		oldPoint.y = getPosition().y;

		cmd.linear.x = 1;
		//recordData();
		outport.write(cmd);
		cout<<"motion to goal\n";
		if(firstMove){
			faceGoal();
			firstMove=false;
		}

}
void halt(){
geometry_msgs::Twist cmd;
    cmd.linear.x = 0;
    outport.write(cmd);
}
//----- Obstacle Avoidance ----//

bool completeCycleAroundObstacle(geometry_msgs::Point robotPos, geometry_msgs::Point hitPoint){
	bool res = distanceEuclid(hitPoint, robotPos) <= DIST_ERROR && totalDistance > 120;
	return res;
}


void identifyLeavePoint(bool direction, geometry_msgs::Point robotPos, geometry_msgs::Point goalPos){
recordHitPoint(robotPos);
wallFollowing(direction);
double tot = totalDistance + 1;	
setTotalDistance(tot);
findLeavePoint(robotPos, hitPoint);
}


bool goalReached(geometry_msgs::Point robotPos, geometry_msgs::Point goalPos, double distError){
	return (distanceEuclid(robotPos, goalPos) <= distError);
}

void recordHitPoint(geometry_msgs::Point point){
if(!hitDefined){ 
	//updates the hitpoint
	hitPoint.x = point.x;
	hitPoint.y = point.y;
	//now the robot switches to the boundary following mode
	hitDefined = true;
	//isAlreadyLeftLastHitPoint=false;
	}
}
void recordLeavePoint(geometry_msgs::Point point){	
	leavePoint.x = point.x;
	leavePoint.y = point.y;
}
bool goalIsReachable(geometry_msgs::Point robotPos, geometry_msgs::Point goalPos, double distError){
	bool res = false;
	faceGoal();
	if(obstacleInFrontOfTheRobot() && goalReached(robotPos, goalPos, distError)) 
		res = true;
	else if (not obstacleInFrontOfTheRobot())
		res = true;
	cout<< "Goal reachable from this point?? " <<res<<"\n";
        return res;
}

bool configureHook(){
bool ok = false;
taskOdom = this->getPeer("odometryAdapter");
taskDistanceSensor = this->getPeer("laserAdapter");
//taskDistanceSensor = this->getPeer("IRAdapter");
if (taskOdom) {
	this->connectServices(taskOdom);
	cout<<"connecting odometry services done --- \n";
	ok = true;
}
if (this->requires("localisation")->ready()) {
	log(RTT::Info) << "Service odometry ready ---\n" << RTT::endlog();
	ok = true;
} else {
	log(RTT::Warning) << "Service odometry Not ready!" << RTT::endlog();
	ok = false;
}

if (this->getPosition.ready() && this->getOrientation.ready()){
	log(RTT::Info) << "localisation services ready ---\n" << RTT::endlog();
	ok = true;
}

else{
	log(RTT::Warning) << "localisation services not ready!" << RTT::endlog();
	ok = false;
}

if (taskDistanceSensor) {
	this->connectServices(taskDistanceSensor);
	cout<<"connecting DistanceSensor services done --- \n";
	ok = true;
}
if (this->requires("distanceSensor")->ready()) {
	log(RTT::Info) << "Service DistanceSensor ready --- \n" << RTT::endlog();
	ok = true;
} else {
	log(RTT::Warning) << "Service DistanceSensor Not ready!" << RTT::endlog();
	ok = false;
}

if (this->getSafeDistance.ready() && this->obstacleInFrontOfTheRobot.ready() && this->obstacleOnTheLeft.ready() && this->obstacleOnTheRight.ready() && this->getDistanceToWall.ready() && this->obstacleInWay.ready() ){
	log(RTT::Info) << "sensing services ready --- \n" << RTT::endlog();
	ok = true;
}
else{
	log(RTT::Warning) << "sensing services not ready!" << RTT::endlog();
	ok = false;
}
return ok;
}

void updateHook(){
geometry_msgs::Point robotPos = getPosition();
if (goalReached(robotPos, goalPosition, DIST_ERROR)){	
	cout<<"at goal :) \n";
	sleep(1);
	halt();	
	EXIT_SUCCESS;
}
else if(obstacleInWay()){
	identifyLeavePoint(direction, robotPos, goalPosition);
	cout<<"checking --- \n";
	if(leavePointFound() && researchComplete(robotPos, hitPoint, goalPosition)){
		cout<<"going to the leave point...\n";
		goToLeavePoint(leavePoint);
		halt();
		faceGoal();
	}
	else if(completeCycleAroundObstacle(robotPos, hitPoint) && !leavePointFound()){
		cout<<"no leave point found! hit point encountered again!\n";
		cout<<"===> goal unreachable \n";
		//sleep(1);
		halt();
		EXIT_FAILURE;
	}
}
else 
	motionToGoal();
}

};
}

#endif
