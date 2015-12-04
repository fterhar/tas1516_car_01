#ifndef RECT_H
#define RECT_H

#include "geometry_msgs/Point.h"	//include message type
#include "geometry_msgs/Pose.h" 	//include message type
#include "point.h"

class Rect{

    /* This is a rectangle class,
       used to specify an area where
       speed gain is active */

    public:
 	Rect();
	Rect(Point p1, Point p2);
	bool isInside(Point p);

    private:
	Point p1; //Links unten
	Point p2; //Rechts unten
};

#endif // RECT_H
