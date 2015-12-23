
/*
 * Created by:  Fynn Terhar
 * Modified by: -
 */

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

   if( (this->p1.x > this->p2.x) && (this->p1.y > this->p2.y) ){
	return( (pt.x > p2.x && pt.x < p1.x) && (pt.y > p2.y && pt.y < p1.y) );
   } else if ( (this->p1.x >  this->p2.x) && (this->p1.y <= this->p2.y) ){
	return( (pt.x > p2.x && pt.x < p1.x) && (pt.y > p1.y && pt.y < p2.y) );
   } else if ( (this->p1.x <= this->p2.x) && (this->p1.y >  this->p2.y) ){
	return( (pt.x > p1.x && pt.x < p2.x) && (pt.y > p2.y && pt.y < p1.y) );
   } else if ( (this->p1.x <= this->p2.x) && (this->p1.y <= this->p2.y) ){
	return( (pt.x > p1.x && pt.x < p2.x) && (pt.y > p1.y && pt.y < p2.y) );
   } else {
	ROS_WARN("Unhandled case in this->isInside!");
	return (false);
   }
}

/* This function calculates Poses that describe a rect in */
boost::array<geometry_msgs::Pose, 6> Rect::asPoses(){
    ROS_INFO("Call: asPoses");
    if( (this->p1.x > this->p2.x) && (this->p1.y > this->p2.y) ){
	//Fall 2
	return (this->generatePosesNeg(this->p1, this->p2)); 
   } else if ( (this->p1.x >  this->p2.x) && (this->p1.y <= this->p2.y) ){
	//Fall 4
	return (this->generatePosesPos(this->p1, this->p2)); 
   } else if ( (this->p1.x <= this->p2.x) && (this->p1.y >  this->p2.y) ){
	//Fall 1
	return (this->generatePosesPos(this->p2, this->p1)); 
   } else if ( (this->p1.x <= this->p2.x) && (this->p1.y <= this->p2.y) ){
	// Fall 3
	return (this->generatePosesNeg(this->p2, this->p1)); 
   } else {
	ROS_WARN("Unhandled case in this->asPoses!");
	return (this->generatePosesNeg(this->p2, this->p1));
   }
}

boost::array<geometry_msgs::Pose, 6> Rect::generatePosesNeg(Point pa, Point pb){
//2_3
   ROS_INFO("Call: generatePosesNeg"); 
   boost::array<geometry_msgs::Pose, 6> poses;

   /* Init positions to point a and point b */
   for(unsigned i = 0; i < 3; i++){
	poses[i].position.x = pa.x;  
	poses[i].position.y = pa.y;  
	poses[i].position.z = 0;
   }
   for(unsigned i = 3; i < 6; i++){
	poses[i].position.x = pb.x;  
	poses[i].position.y = pb.y;  
	poses[i].position.z = 0; 
   }

/**** Calculate angles for poses ****/

   /* Angle of middle arrow of pa */
   poses[0].orientation = \
	tf::createQuaternionMsgFromYaw(M_PI + \
				       Point::getAngle(pa, pb));
   /* Angle of x-axis of pa */
   poses[1].orientation =  \
	tf::createQuaternionMsgFromYaw(3 * M_PI/2);
  
   /* Angle of y-axis of pa */
   poses[2].orientation = \
	tf::createQuaternionMsgFromYaw(M_PI);

   /* Middle angle of pb */
   poses[3].orientation = \
	tf::createQuaternionMsgFromYaw((Point::getAngle(pa, pb))) ;

   /* y-axis of pb */
   poses[4].orientation =  \
	tf::createQuaternionMsgFromYaw(0);

   /* x-axis of pb */
   poses[5].orientation =  \
	tf::createQuaternionMsgFromYaw(M_PI/2);
 
   return poses;
}

/* Generate Poses for positive angle case.
   Given two points that make up a rectangle, this method
   calculates Poses with correct */
boost::array<geometry_msgs::Pose, 6> Rect::generatePosesPos(Point pa, Point pb){
//1_4

   ROS_INFO("Call: generatePosesPos"); 
   boost::array<geometry_msgs::Pose, 6> poses;
   
   for(unsigned i = 0; i < 3; i++){
	poses[i].position.x = pa.x;  
	poses[i].position.y = pa.y;  
	poses[i].position.z = 0;
   }
   for(unsigned i = 3; i < 6; i++){
	poses[i].position.x = pb.x;  
	poses[i].position.y = pb.y;  
	poses[i].position.z = 0; 
   }

/**** Calculate angles for poses ****/

   /* Angle of middle arrow of pa */
   poses[0].orientation =\
	tf::createQuaternionMsgFromYaw(M_PI - \
					 Point::getAngle(pa, pb)); 
   poses[1].orientation =\
	tf::createQuaternionMsgFromYaw(M_PI/2);
   poses[2].orientation = \
	tf::createQuaternionMsgFromYaw(M_PI);
   poses[3].orientation = \
	tf::createQuaternionMsgFromYaw(M_PI - \
			 Point::getAngle(pa, pb) + M_PI);
   poses[4].orientation = \
	tf::createQuaternionMsgFromYaw(0);
   poses[5].orientation = \
	tf::createQuaternionMsgFromYaw(3 * M_PI/2);


//   poses[0].orientation.w = M_PI + Point::getAngle(pa, pb); 
//   poses[1].orientation.w = M_PI;
//   poses[2].orientation.w = 2 * M_PI;
//   poses[3].orientation.w = poses[0].orientation.w + 2 * M_PI;
//   poses[4].orientation.w = 0;
//   poses[5].orientation.w = 3 * M_PI;
 
   return poses;
}
