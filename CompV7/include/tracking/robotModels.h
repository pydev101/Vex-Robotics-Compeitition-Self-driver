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

  int mode = 0;
  Vector userDesiredVel = Vector(0, 0);

public:
  PIDGains linearGains = {0, 0, 0};
  PIDGains rotGains = {0, 0, 0};
  PIDGains reverseLinearGains = {0, 0, 0};
  PIDGains reverseRotGains = {0, 0, 0};
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

    if(mode == 0){
      //PID

      //Tangential PID
      if(headingIndependence){
        if(abs(hError) > stopAngularRadius){
          if(abs(hError) < abs(rHError)){
            targetAngularVel = hError*rotGains.p;
          }else{
            targetAngularVel = (abs(rHError)*reverseRotGains.p + reverseRotGains.i)*sign(hError);
          }
        }else{
          targetAngularVel = 0;
        }

        //Magnitude of PID(e) but direction of error
        double mag = 0;
        if(error.getMagnitude() > errorRadius){
          if(error.getMagnitude() < rError.getMagnitude()){
            mag = error.getMagnitude()*linearGains.p;
          }else{
            mag = rError.getMagnitude()*reverseLinearGains.p + reverseLinearGains.i;
          }
        }
        targetVelocity = Vector(mag, getTranslationalLocalHeading(), false);
      }else{
        if(abs(hError) > stopAngularRadius){
          targetAngularVel = hError*rotGains.p;
        }else{
          targetAngularVel = 0;
        }

        //Magnitude of PID(e) but only on y axis
        if(error.getMagnitude() > errorRadius){
          if(abs(error.getY()) < abs(rError.getY())){
            targetVelocity = Vector(0, error.getY()*linearGains.p);
          }else{
            targetVelocity = Vector(0, (abs(rError.getY())*reverseLinearGains.p + reverseLinearGains.i)*sign(error.getY()));
          }
        }else{
          targetVelocity = Vector(0, 0);
        }
      }
    }else if (mode == 1) {
      //Constant velocity
      if(error.getMagnitude() > errorRadius){
        if(error.getMagnitude() > currentVel.getMagnitude()*currentVel.getMagnitude()/(2*maxDeltaV)){
          targetVelocity = userDesiredVel;
        }else{
          if(headingIndependence){
            targetVelocity = Vector(error.getMagnitude()*linearGains.p, getTranslationalLocalHeading(), false);
          }else{
            targetVelocity = Vector(0, error.getY()*linearGains.p);
          }
        }
      }else{
        targetVelocity = Vector(0, 0);
      }
    }else if(mode == 2){
      //Path following mode

      //Same as CV execept it will adapt if their is a next target
    }


    //After target vel is set its then check for acceleration
    Vector dV = targetVelocity - currentVel;
    if(dV.getMagnitude() > maxDeltaV){
      dV = dV.getUnitVector().scale(maxDeltaV);
    }
    outputVelocity = currentVel + dV;

    double deltaOmega = targetAngularVel - currentAngularVel;
    if(abs(deltaOmega) > maxDeltaW){
      deltaOmega = maxDeltaW*sign(deltaOmega);
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

  void activatePID(){
    mode = 0;
  }
  void activateCV(Vector cv){
    userDesiredVel = cv;
    mode = 1;
  }
};


#endif