/*
 * Created by:  Fynn Terhar
 * Modified by: -
 * Description: This class implements a simple point in 2D
 * 		It is used for the rectangles
 */

#ifndef POINT_H
#define POINT_H

#include <cmath>
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
