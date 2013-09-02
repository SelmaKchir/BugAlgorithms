#include "LaserAdapter.h"

using namespace navigation;
LaserAdapter::LaserAdapter(std::string& name): TaskDistanceSensor(name), laser_port("laser_scan"){
	  this->ports()->addPort(laser_port).doc("port connected to LaserAdapter scan sensor");
	  samples = 90;
	  nb_sensors = 5;
	  range_max = 2.0;
}

double LaserAdapter::getDistanceToWall(){
	vector<double> rays = getDistanceSensors();	
	double average = samples / nb_sensors;
	double minDistance = average * range_max;
	for(int i=0; i<nb_sensors; i++){
		if(rays[i] < minDistance)
			minDistance = rays[i];	
	}
return (minDistance/average); 
}

double LaserAdapter::getSafeDistance(){
	return 0.2;
}


bool LaserAdapter::obstacleFreeWay(){			//Checks if the way to Goal is obstacle free, returns TRUE if the way is free
double midA, midB;
	sensor_msgs::LaserScan msg;
	laser_port.read(msg);
	midA = std::accumulate(msg.ranges.begin(), msg.ranges.begin()+45, 0);  // ?? +2 if the way is free
	midB = std::accumulate(msg.ranges.begin()+45, msg.ranges.end(), 0);
	return (midA == (samples/2)* range_max && midB == (samples/2 * range_max));// if all the sensors return +2, the way is free, returns TRUE
}


vector<double> LaserAdapter::getDistanceSensors(){
	sensor_msgs::LaserScan msg;	
	vector <double> rays;
	laser_port.read(msg);
	int j = 18;
	int  nbRays = samples / nb_sensors;

	for(int i=0; i< nb_sensors; i++){
		rays.push_back(std::accumulate(msg.ranges.begin()+ j - nbRays , msg.ranges.begin()+j, 0));
		//cout<<"rays["<<i<<"]: "<<rays[i]<<"\n";
		j = j + nbRays;
	}
	return rays;
}

bool LaserAdapter::obstacleOnTheRight(){
	sensor_msgs::LaserScan msg;	
	vector<double> rays = getDistanceSensors();	
	double sensorLeft = rays[4];
	double sensorLeftFront = rays[3];
	//double sensorFront = rays[2];
	double sensorRightFront = rays[1];
	double sensorRight = rays[0];
	
	return (sensorRight + sensorRightFront < sensorLeft + sensorLeftFront);
}

bool LaserAdapter::obstacleOnTheLeft(){
	sensor_msgs::LaserScan msg;	
	vector<double> rays = getDistanceSensors();	
	double sensorLeft = rays[4];
	double sensorLeftFront = rays[3];
	//double sensorFront = rays[2];
	double sensorRightFront = rays[1];
	double sensorRight = rays[0];
	
	return (sensorRight + sensorRightFront > sensorLeft + sensorLeftFront);
}


bool LaserAdapter::obstacleInFrontOfTheRobot(){
	sensor_msgs::LaserScan msg;	
	vector<double> rays = getDistanceSensors();	
	double sensorLeft = rays[4];
	double sensorLeftFront = rays[3];
	//double sensorFront = rays[2];
	double sensorRightFront = rays[1];
	double sensorRight = rays[0];
	
	return (sensorRightFront + sensorLeft + sensorLeftFront < (samples / 5) * 3);
}


bool LaserAdapter::obstacleInWay(){
	return (not obstacleFreeWay());
}

/*
bool LaserAdapter::goalIsReachable(){
	faceGoal();
	if ((obstacleInFrontOfTheRobot() && goalReached()) || obstacleFreeWay())
	  return true;
	else
	  return false;
}*/


