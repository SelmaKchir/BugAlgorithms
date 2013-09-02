#include "OdometryAdapter.h"
using namespace navigation;
OdometryAdapter::OdometryAdapter(std::string& name): TaskLocalisation(name), odometry_port("odom") {
    this->ports()->addPort(odometry_port).doc("position of the robot obtained by odometry");//documentation which describe the port  
}


geometry_msgs::Point OdometryAdapter::getPosition(){
   geometry_msgs::Point pos;
   nav_msgs::Odometry position_msg; 
   odometry_port.read(position_msg); //read the current position of the robot and put it in a new instance of Odometry(here position_msg)
   pos.x = position_msg.pose.pose.position.x; //get the position x of the robot
   pos.y = position_msg.pose.pose.position.y; //get the position y of the robot
   return pos; //return the current position of the robot
}

geometry_msgs::Quaternion OdometryAdapter::getOrientation(){
   geometry_msgs::Point pos;
   nav_msgs::Odometry position_msg; 
   return position_msg.pose.pose.orientation;
}
