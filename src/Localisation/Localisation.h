#ifndef __LOCALISATION_h__
#define __LOCALISATION_h__

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
class Localisation{

public: 
//----- localization method ------//
virtual geometry_msgs::Point getPosition() = 0;
virtual geometry_msgs::Quaternion getOrientation() = 0;
};
#endif
