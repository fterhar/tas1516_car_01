
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

float Point::getAngle(Point p1, Point p2){
   ROS_INFO("Call: getAngle");
   if(abs(p1.x - p2.x) != 0){ 
      return ( atan(abs(p1.y - p2.y) / abs(p1.x - p2.x)) );
   } else {
      return 0;
   }
}
