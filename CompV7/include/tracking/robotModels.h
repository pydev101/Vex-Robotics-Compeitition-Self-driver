#ifndef __ROBOTMODELS_H__
#define __ROBOTMODELS_H__

/*
Name: robotModels.h
Written By: Carson Easterling


*/

#include "robotmath.h"
#include "logger.h"
#include "navigation.h"

class Robot : public Navigator{
protected:
  Vector targetVelocity = Vector(0, 0); //Magnitude is the Pct velocity, and theta is direction of travel; Realitive to the current direction of the robot
  double targetAngularVel = 0; //Pct 

public:
  PIDGains linearGains = {0, 0, 0};
  PIDGains rotGains = {0, 0, 0};
  PIDGains reverseLinearGains = {0, 0, 0};
  PIDGains reverseRotGains = {0, 0, 0};

  Robot(Point currentPosition, double currentHeading, bool currentHeadingInDeg,
            double stopRadiusArg, double errorRadiusArg, double stopAngularRadiusArg, 
            double stopTimeArg, double errorTimeArg, double shiftRadiusArg,
            PIDGains linearGainContants, PIDGains roationalGainContants,
            PIDGains reverseLinearGainContants, PIDGains reverseRoationalGainContants): Navigator(currentPosition, currentHeading, currentHeadingInDeg, stopRadiusArg, errorRadiusArg, stopAngularRadiusArg, stopTimeArg, errorTimeArg, shiftRadiusArg){ 
    linearGains = linearGainContants;
    rotGains = roationalGainContants;
    reverseLinearGains = reverseLinearGainContants;
    reverseRotGains = reverseRoationalGainContants;
  }

  void updateTargetVelocities(){
    //Check for headingIndependence to see if it should wait and turn or if it should be based of the dot product etc; targetvel is the pid of pos error and ang vel is pid of ang error
    //Rverse PID or regular PID is based off only of percentage of difference between lastStopped pos and currentTarget
    //Build in support for transitiotng between this target and the next one if the yare not the same
    //Support for pid velocitiy, constant velocity, path velocity
    Vector error = getLocalError();
    Vector rError = getReverseLocalError();
    Vector nError = getNextLocalError();

    if(headingIndependence){
      
    }else{

    }
    
  }

  Vector getTargetVelocity(){
    return targetVelocity;
  }

  double getAngularVelocity(){
    return targetAngularVel;
  }
};


#endif