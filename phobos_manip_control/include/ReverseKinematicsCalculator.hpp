#ifndef REVERSEKINEMATICSCALCULATOR_HPP_
#define REVERSEKINEMATICSCALCULATOR_HPP_

class ReverseKinematicsCalculator
{
    double x,y,z,angle;
	double link [4];
	double l1, l2, l3, l4;
	
public:

	ReverseKinematicsCalculator();
    ReverseKinematicsCalculator(double length1, double length2, double length3, double length4);
    ~ReverseKinematicsCalculator(){};
    
    void SetPositionOrientation(double msgx, double msgy, double msgz, double msgangle);
    int SetLengths(double length1, double length2, double length3, double length4);
	int CalculateReverseKinematics();
	double ReturnLink(int link_number);
	
};

#endif
