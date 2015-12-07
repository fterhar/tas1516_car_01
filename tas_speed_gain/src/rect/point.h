
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#ifndef POINT_H
#define POINT_H

#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>

class Point{

    /* This is a point class,
       used to specify an area where
       speed gain is active */

    public:
	Point(); 
	Point(float x, float y);
	static float getAngle(Point p1, Point p2);
	float x; 
	float y; 
};

#endif // POINT_H
