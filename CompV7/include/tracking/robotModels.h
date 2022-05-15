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

  Vector outputVelocity = Vector(0, 0);
  double outputAngularVelocity = 0;

public:
  PIDGains linearGains = {0, 0, 0};
  PIDGains rotGains = {0, 0, 0};
  PIDGains reverseLinearGains = {0, 0, 0};
  PIDGains reverseRotGains = {0, 0, 0};
  bool PIDMode = true;
  double maxDeltaV = 1000;
  double maxDeltaW = 1000;

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

  void updateTargetVelocities(double deltaT){
    //TODO Build in support for transitiotng between this target and the next one if the yare not the same
    //Support for pid velocitiy, constant velocity, path velocity

    Vector error = getLocalError();
    Vector rError = getReverseLocalError();
    Vector nError = getNextLocalError();
    double hError = getHeadError();
    double rHError = getReverseHeadError();

    Vector currentVel = getLocalVelocity();
    double currentAngularVel = getAngularVelocity();

    //Set Target Velocites
    if(PIDMode){
      //Rotation PID
      if(abs(hError) < abs(rHError)){
        targetAngularVel = hError*rotGains.p;
      }else{
        targetAngularVel = (abs(rHError)*reverseRotGains.p + reverseRotGains.i)*sign(hError);
      }


      //Tangential PID
      if(headingIndependence){
        //Magnitude of PID(e) but direction of error
        double mag = 0;
        if(error.getMagnitude() < rError.getMagnitude()){
          mag = error.getMagnitude()*linearGains.p;
        }else{
          mag = rError.getMagnitude()*reverseLinearGains.p + reverseLinearGains.i;
        }
        targetVelocity = Vector(mag, getTranslationalLocalHeading(), false);
      }else{
        //Magnitude of PID(e) but only on y axis
        if(abs(error.getY()) < abs(rError.getY())){
          targetVelocity = Vector(0, error.getY()*linearGains.p);
        }else{
          targetVelocity = Vector(0, (abs(rError.getY())*reverseLinearGains.p + reverseLinearGains.i)*sign(error.getY()));
        }
      }
    }

    //TODO might have some issues might need to do some division by deltaT or soemthign
    Vector dV = targetVelocity - currentVel;
    if(dV.getMagnitude() > maxDeltaV){
      dV = dV.getUnitVector().scale(maxDeltaV);
    }
    outputVelocity = currentVel + dV;

    double deltaOmega = targetAngularVel - currentAngularVel;
    if(deltaOmega > maxDeltaW){
      deltaOmega = maxDeltaW;
    }
    outputAngularVelocity = currentAngularVel + deltaOmega; 
  }

  //Outputs are local to the robot
  Vector getTangentVelocity(){
    return outputVelocity;
  }

  double getAngularVelocity(){
    return outputAngularVelocity;
  }
};


#endif