#include "tracking/robotModels.h"
#include "vex.h"

using namespace vex;


//Tank drive has to do getY of vel for speed
//X drive + Mech drive would use vel realitive to the robot


const double headingDriftFactor = 1;
double getHeadingUnbounded(){
  double head = Inertial.angle();
  return head*headingDriftFactor;
}
double getHeading(){
  double toss;
  double r = (360)*modf(getHeadingUnbounded()/(360), &toss);
  if(r<0){r+=(360);}
  return r;
}
double getHeadingCCW(){
  return (360 - getHeading());
}