/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

#include "point.h"


Point::Point(){
    Point::x = 0, Point::y = 0;
}

Point::Point(float x, float y){
    Point::x = x, Point::y = y;
}

/* Needed for visualization of the rectangles in rviz */
float Point::getAngle(Point p1, Point p2){
   ROS_INFO("Call: getAngle: %f,%f", fabs(p1.x-p2.x), fabs(p1.x-p2.x));
   if(fabs(p1.x - p2.x) != 0){ 
      return ( atan(fabs(p1.y - p2.y) / fabs(p1.x - p2.x)) );
   } else {
      ROS_WARN("Zerodivision prevented,\np1.x:%f\tp2.x:%f\np1.y:%f\tp2.y%f",p1.x, p2.x, p1.y, p2.y);
      return 0;
   }
}
