
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#ifndef RECT_H
#define RECT_H

#include "geometry_msgs/Point.h"	//include message type
#include "geometry_msgs/Pose.h" 	//include message type
#include "geometry_msgs/PoseArray.h"
#include "boost/array.hpp"
#include "point.h"
#include <tf/transform_listener.h>

class Rect{

    /* This is a rectangle class,
       used to specify an area where
       speed gain is active */

    public:
 	Rect();
	Rect(Point p1, Point p2);
	bool isInside(Point p);
	boost::array<geometry_msgs::Pose, 6> asPoses();
	Point p1; //Links unten
	Point p2; //Rechts unten
    private:
	boost::array<geometry_msgs::Pose, 6> generatePosesPos(Point, Point);
	boost::array<geometry_msgs::Pose, 6> generatePosesNeg(Point, Point);

};

#endif // RECT_H
