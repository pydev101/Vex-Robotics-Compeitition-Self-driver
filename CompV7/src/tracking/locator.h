#ifndef __LOCATOR_H__
#define __LOCATOR_H__

/*
Name: locator.h
Written By: Carson Easterling
*/

#include "robotmath.h"
#include "logger.h"

class Locator{
public:
  Point currentPos = Point(0, 0); //units
  double currentHead = 0; //rad
  Vector tangentialVelocity = Vector(0, 0);
  double angularVelocity = 0;
  Vector tangentialAcceleration = Vector(0, 0);
  double angularAcceleration = 0;

  int trackingMode = 0;
  Point targetPoint = Point(0, 0);
  double targetHeading = 0;

  //TODO: Target location, markers, pre-planned paths, path optimization, stop status, return tangential error vector and angulaur error, tracking modes
  //Robot handles from errors to wheels; Call updateLocation, set the pos/heading if needed, Call updateTracking
  //Might want to write robot in abstract motor classes in order to make sure vex u motors can be implemented without issue

  Locator(Point currentPosition, double currentHeading){
    currentPos = currentPosition;
    currentHead = currentHeading;
  }


  //Intended to be used to replace tracking data with info from inertial or gps sensors after calling the updateLocation function with the deltas but before calling the updateLocation function
  void setHead(double newHead, bool headingInDegrees){
    if(headingInDegrees){
      newHead = degToRad(newHead);
    }
    currentHead = newHead;
  }
  void setCurrentPosition(Point p){
    currentPos = p;
  }
  void setCurrentPosition(double x, double y){
    currentPos = Point(x, y);
  }


  void updateLocation(Vector deltaPos, double deltaHeading, double deltaT, bool headingInDegrees=false){
    if(headingInDegrees){
      deltaHeading = degToRad(deltaHeading);
    }

    currentPos = deltaPos + currentPos;
    currentHead = currentHead + deltaHeading;

    Vector newV = deltaPos.scale(1/deltaT);
    tangentialAcceleration = (newV - tangentialVelocity).scale(1/deltaT);
    tangentialVelocity = newV;
    
    double newOmega = deltaHeading / deltaT;
    angularAcceleration = (newOmega - angularVelocity)/deltaT;
    angularVelocity = newOmega;
  }

  void updateTracking(){

  }
};

#endif