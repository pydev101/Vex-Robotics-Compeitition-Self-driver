#ifndef __ROBOTMODELS_H__
#define __ROBOTMODELS_H__

/*
Name: robotModels.h
Written By: Carson Easterling

TODO
Max Velocity
Path transitions

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
  PIDGains linearGains = {0, 0, 0}; //Pct per Unit Error
  PIDGains rotGains = {0, 0, 0};
  PIDGains reverseLinearGains = {0, 0, 0};
  PIDGains reverseRotGains = {0, 0, 0};
  double maxDeltaV = 2; //Pct
  double maxDeltaW = 2;
  double maxV = 100; //Pct
  double maxW = 100;

  Robot(Point currentPosition, double currentHeading, bool currentHeadingInDeg,
            PIDGains linearGainContants, PIDGains roationalGainContants,
            PIDGains reverseLinearGainContants, PIDGains reverseRoationalGainContants): Navigator(currentPosition, currentHeading, currentHeadingInDeg){ 
    linearGains = linearGainContants;
    rotGains = roationalGainContants;
    reverseLinearGains = reverseLinearGainContants;
    reverseRotGains = reverseRoationalGainContants;
  }

  void updateTargetVelocities(double deltaT){
    //TODO Build in support for transitiotng between this target and the next one if the yare not the same
    //TODO build in support for current vel

    Vector error = getLocalError();
    Vector rError = getReverseLocalError();
    Vector nError = getNextLocalError();
    double hError = getHeadError();
    double rHError = getReverseHeadError();

    static Vector oldTargetVel = Vector(0, 0);
    oldTargetVel = targetVelocity;

    static double oldTargetW = 0;
    oldTargetW = targetAngularVel;

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
        if(error.getMagnitude() > oldTargetVel.getMagnitude()*oldTargetVel.getMagnitude()/(2*maxDeltaV)){
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
    Vector dV = targetVelocity - oldTargetVel;
    if(dV.getMagnitude() > maxDeltaV){
      dV = dV.getUnitVector().scale(maxDeltaV);
    }
    targetVelocity = oldTargetVel + dV;
    if(targetVelocity.getMagnitude() > maxV){
      targetVelocity = targetVelocity.getUnitVector().scale(maxV);
    }
    outputVelocity = targetVelocity;

    double deltaOmega = targetAngularVel - oldTargetW;
    if(abs(deltaOmega) > maxDeltaW){
      deltaOmega = maxDeltaW*sign(deltaOmega);
    }
    targetAngularVel = deltaOmega + oldTargetW;
    if(abs(targetAngularVel) > maxW){
      targetAngularVel = sign(targetAngularVel)*maxW;
    }
    outputAngularVelocity = targetAngularVel; 
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

void setM(vex::motor m, double speed, vex::velocityUnits uni=vex::velocityUnits::pct){
  m.setVelocity(speed, uni);
  if(speed == 0){
    m.stop();
  }else{
    m.spin(vex::forward);
  }
}

void setSide(vex::motor_group m, double speed, vex::velocityUnits uni=vex::velocityUnits::pct){
  m.setVelocity(speed, uni);
  if(speed == 0){
    m.stop();
  }else{
    m.spin(vex::forward);
  }
}

class TankDrive : public Robot{
  protected:
    vex::motor_group leftSide;
    vex::motor_group rightSide;
  
  public:
    TankDrive(vex::motor_group& leftSideArg, vex::motor_group& rightSideArg, Point currentPosition, double currentHeading, bool currentHeadingInDeg,
              PIDGains linearGainContants, PIDGains roationalGainContants,
              PIDGains reverseLinearGainContants, PIDGains reverseRoationalGainContants) : Robot(currentPosition, currentHeading, currentHeadingInDeg,
              linearGainContants, roationalGainContants,
              reverseLinearGainContants, reverseRoationalGainContants){
      leftSide = leftSideArg;
      rightSide = rightSideArg;
    }

    void updateMotors(){
      double speed = getTangentVelocity().getMagnitude();
      double omega = getAngularVelocity();

      double left = speed - omega;
      double right = speed + omega;

      if(abs(right) > maxV){
        left = left - (abs(right) - maxV)*sign(left);
        right = maxV*sign(right);
      }
      if(abs(left) > maxV){
        right = right - (abs(left) - maxV)*sign(left);
        left = maxV*sign(left);
      }

      setSide(leftSide, left);
      setSide(rightSide, right);
    }
};

class XDrive : public Robot{

};

#endif