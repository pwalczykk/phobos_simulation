#include "../include/ReverseKinematicsCalculator.hpp"
#include <math.h>

ReverseKinematicsCalculator::ReverseKinematicsCalculator(){
	
	SetLengths(50,500,250,250);
	
}

ReverseKinematicsCalculator::ReverseKinematicsCalculator(double l1, double l2, double l3, double l4){
	
	SetLengths(l1, l2, l3, l4);
	
}

void ReverseKinematicsCalculator::SetPositionOrientation(double msgx, double msgy, double msgz, double msgangle){
	
	x = msgx;
	y = msgy;
	z = msgz;
	angle = msgangle;
	
}

 int ReverseKinematicsCalculator::SetLengths(double length1, double length2, double length3, double length4){
	 
	 l1 = length1;
	 l2 = length2;
	 l3 = length3;
	 l4 = length4;
	 
 }
 
 double ReverseKinematicsCalculator::ReturnLink(int link_number){
	 
	 return link[link_number];
	 
 }
 
int ReverseKinematicsCalculator::CalculateReverseKinematics(){
	
	double xw, yw, m, n, mi, ni, lkwadrat; 
	
	link[0] = atan2(y,x);
	
	m = sqrt( pow(x,2) + pow(y,2) );
	n = z - l1;
	
	xw = m - l4 * cos(angle);
	yw = n - l4 * sin(angle);
	
	lkwadrat = pow(xw,2) + pow(yw,2);
	
	link[2] = acos( (lkwadrat - pow(l2,2) - pow(l3,2) ) / (2*l3*l4) );
	
	mi = atan2( l3 * sin(link[1]) , l2+l3 * cos(link[1]) );
	ni = atan2(yw,xw);
	
	link[1] = mi + ni;
	
	link[3] = link[1]-link[2]-angle;
	
	return 0;
	
}
 

 


