#include "rect.h"
#include "ros/ros.h"

Rect::Rect(){
    Rect::p1.x = 0;
    Rect::p1.y = 0;
    Rect::p2.x = 0;
    Rect::p2.y = 0;
}

Rect::Rect(Point p1, Point p2){
    Rect::p1.x = p1.x;
    Rect::p1.y = p1.y;
    Rect::p2.x = p2.x;
    Rect::p2.y = p2.y;
}

bool Rect::isInside(Point pt){
   if( (Rect::p1.x > Rect::p2.x) && (Rect::p1.y > Rect::p2.y) ){
	return( (pt.x > p2.x && pt.x < p1.x) && (pt.y > p2.y && pt.y < p1.y) );
   } else if ( (Rect::p1.x >  Rect::p2.x) && (Rect::p1.y <= Rect::p2.y) ){
	return( (pt.x > p2.x && pt.x < p1.x) && (pt.y > p1.y && pt.y < p2.y) );
   } else if ( (Rect::p1.x <= Rect::p2.x) && (Rect::p1.y >  Rect::p2.y) ){
	return( (pt.x > p1.x && pt.x < p2.x) && (pt.y > p2.y && pt.y < p1.y) );
   } else if ( (Rect::p1.x <= Rect::p2.x) && (Rect::p1.y <= Rect::p2.y) ){
	return( (pt.x > p1.x && pt.x < p2.x) && (pt.y > p1.y && pt.y < p2.y) );
   } else {
	ROS_INFO("[WARNING] Unhandled case in Rect::isInside!");
	return (false);
   }
}
