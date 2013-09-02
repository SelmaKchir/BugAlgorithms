#include "IRAdapter.h"

using namespace navigation;
IRAdapter::IRAdapter(std::string& name): TaskDistanceSensor(name),	
	IR_right_port("ranger_scan"),
        Laser_left_port("laser_scan"){
   this->ports()->addPort(IR_right_port).doc("port connected to right infrared sensor compared to the robot axis");
   this->ports()->addPort(Laser_left_port).doc("port connected to the left laser scan sensor");
   samples_right = 90;
   samples_laser = 90;
   nb_sensors = 5;
   laser_range_max = 2.0;
   IR_range_max = 2.0;
}


double IRAdapter::getDistanceToWall(){
	vector<double> rays = getDistanceSensors();	
	double distIR = rays[0];
	double minDist = distIR;
	double average = samples_laser / nb_sensors;
	for(int i=1; i<=nb_sensors; i++){
	double distPerRay = rays[i] / average;
		if(distPerRay < minDist)
			minDist = distPerRay;	
	}
cout<<"min dist: "<<minDist<<"\n";
return minDist; 


}

double IRAdapter::getSafeDistance(){
	return 0.2;
}


bool IRAdapter::obstacleFreeWay(){			//Checks if the way to Goal is obstacle free, returns TRUE if the way is free
double midA, midB;
	sensor_msgs::Range IR_msg;
	sensor_msgs::LaserScan laser_msg;
	IR_right_port.read(IR_msg);
	Laser_left_port.read(laser_msg);
	midA = std::accumulate(laser_msg.ranges.begin(), laser_msg.ranges.begin()+90, 0);  // ?? +2 if the way is free
	return (midA == (samples_laser * laser_range_max) && IR_msg.range ==  IR_range_max);// if all the sensors return +2, the way is free, returns TRUE
}

vector<double> IRAdapter::getDistanceSensors(){
	sensor_msgs::Range IR_msg;
	sensor_msgs::LaserScan msg;	
	vector <double> rays;
	Laser_left_port.read(msg);
	IR_right_port.read(IR_msg);	
	int j = 18;
	int  nbRays = samples_laser / nb_sensors;
        rays.push_back(IR_msg.range);
	for(int i=1; i<= nb_sensors; i++){
		rays.push_back(std::accumulate(msg.ranges.begin()+ j - nbRays , msg.ranges.begin()+j, 0));
		//cout<<"rays["<<i<<"]: "<<rays[i]<<"\n";
		j = j + nbRays;
	}
	cout<<"rays[0] "<<rays[0]<<"\n";
	cout<<"rays[1] "<<rays[1]<<"\n";
	cout<<"rays[2] "<<rays[2]<<"\n";
	cout<<"rays[3] "<<rays[3]<<"\n";
	cout<<"rays[4] "<<rays[4]<<"\n";
	cout<<"rays[5] "<<rays[5]<<"\n";
	return rays;
}

bool IRAdapter::obstacleOnTheRight(){
	sensor_msgs::Range msg;	
	IR_right_port.read(msg);
	return (msg.range < IR_range_max);
}

bool IRAdapter::obstacleOnTheLeft(){
	sensor_msgs::LaserScan msg;	
	
        double midA = std::accumulate(msg.ranges.begin(), msg.ranges.begin()+samples_laser, 0); 
	return (midA == (samples_laser * laser_range_max));
}


bool IRAdapter::obstacleInFrontOfTheRobot(){
	/*sensor_msgs::IRScan msg;	
	vector<double> rays = getDistanceSensors();	
	double sensorLeft = rays[4];
	double sensorLeftFront = rays[3];
	//double sensorFront = rays[2];
	double sensorRightFront = rays[1];
	double sensorRight = rays[0];
	
	return (sensorRightFront + sensorLeft + sensorLeftFront < (samples / 5) * 3);*/
return false;
}


bool IRAdapter::obstacleInWay(){
	return (not obstacleFreeWay());
}

