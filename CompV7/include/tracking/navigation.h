#ifndef __LOCATOR_H__
#define __LOCATOR_H__

/*
Name: navigation.h
Written By: Carson Easterling

TODO
Waypoints
Pre-planned paths
Bezier paths or path will go through a set of defined points


*/

#include "robotmath.h"
#include "logger.h"

typedef struct {
  Point p; //Units
  double head; //Radians
} positionSet;

bool operator==(const positionSet& a, const positionSet& b){
  if((a.p.x == b.p.x) && (a.p.y == b.p.y) && (a.head == b.head)){
    return true;
  }else{
    return false;
  }
}
bool operator!=(const positionSet& a, const positionSet& b){
  return !(a == b);
}

class PositionList{
private:
  positionSet* data = (positionSet*)malloc(0); //Array that stores points
  int size = 0; //Size of the array
public:
  //Frees data when pointer is no longer used; Ensures safe memory usage
  ~PositionList(){
    free(data);
  }

  //Clears all data from the list
  void clear(){
    size = 0;
    free(data);
    data = (positionSet*)malloc(0);
  }

  //Adds a point to the end of the list
  void append(positionSet p){
    size++;
    data = (positionSet*)realloc(data, sizeof(positionSet) * size);
    data[size - 1] = p;
  }

  //Returns a point at an index in the list
  positionSet &operator[](size_t index) {
    return data[index];
  }

  int getSize(){
    return size;
  }
};

class Navigator{
protected:
  positionSet currentPos = {Point(0, 0), 0};
  positionSet previousPos = {Point(0, 0), 0};
  positionSet lastStoppedPos = {Point(0, 0), 0};

  Vector tangentialVelocity = Vector(0, 0); //units per second
  double angularVelocity = 0; //radians per second

  Vector tangentialAcceleration = Vector(0, 0); //units per second per second
  double angularAcceleration = 0; //radians per second per second

  
  positionSet currentTarget = {Point(0, 0), 0};
  positionSet nextTarget = {Point(0, 0), 0};
  positionSet lastTarget = {Point(0, 0), 0};

  int targetPathIndex = 0;
  PositionList targetPath;

  bool headingIndependence = false; //Determines if the heading is controlled by the tangential error direction
  bool atTarget = false;
  bool isStopped = false;
  bool atRotTarget = false;
  bool isStoppedRot = false;

public:
  double stopRadius = 1;
  double stopTime = 0.2;
  double errorRadius = 1;
  double errorTime = 0.2;
  double shiftRadius = 12;
  double stopAngularRadius = degToRad(1);
  bool forward = true;

  Navigator(Point currentPosition, double currentHeading, bool currentHeadingInDeg){
    if(currentHeadingInDeg){
      currentHeading = degToRad(currentHeading);
    }
    currentHeading = normalizeAngle(currentHeading);

    currentPos = {currentPosition, currentHeading};
    currentTarget = currentPos;
    lastTarget = currentPos;
    nextTarget = currentPos;
    previousPos = currentPos;
    lastStoppedPos = currentPos;
  }

  void setHead(double newHead, bool headingInDegrees){
    if(headingInDegrees){
      newHead = degToRad(newHead);
    }
    newHead = normalizeAngle(newHead);
    currentPos.head = newHead;
  }
  void setCurrentPosition(Point p){
    currentPos.p = p;
  }
  void setCurrentPosition(double x, double y){
    currentPos.p = Point(x, y);
  }

  //Updates Position data based on global transformations
  void shiftLocationGlobal(Vector deltaPos, double deltaHeading, bool headingInDegrees=false){
    if(headingInDegrees){
      deltaHeading = degToRad(deltaHeading);
    }

    currentPos.p = deltaPos + currentPos.p;
    currentPos.head = normalizeAngle(currentPos.head + deltaHeading);
  }

  //Updates Position data based on local transformations
  void shiftLocationLocal(Vector deltaFwd, double deltaHeading, bool headingInDegrees=false){
    Vector deltaPos = Vector(deltaFwd.getY(), currentPos.head) + Vector(deltaFwd.getX(), currentPos.head - PI/2);
    shiftLocationGlobal(deltaPos, deltaHeading, headingInDegrees);
  }

  //Updates the targeting system and error vectors
  void updateTracking(double deltaT){
    static double stopTimer = 0;
    static double errorTimer = 0;
    static double rotStopTimer =0;
    static double rotErrorTimer = 0;

    Vector motion = Vector(previousPos.p, currentPos.p);

    Vector newV = motion.scale(1/deltaT);
    tangentialAcceleration = (newV - tangentialVelocity).scale(1/deltaT);
    tangentialVelocity = newV;


    //TODO IF NOT HEADING INDEPENDECNCE MAKE THIS BASED OFF THE DOT PRODUCT
    if(motion.getMagnitude() < stopRadius){
      stopTimer += deltaT;
      if(stopTimer > stopTime){
        lastStoppedPos.p = currentPos.p;
        isStopped = true;
      }
    }else{
      stopTimer = 0;
      isStopped = false;
    }

    Vector error = Vector(currentPos.p, currentTarget.p);
    if(targetPath.getSize() > 0){
      if(error.getMagnitude() < shiftRadius){
        if(targetPathIndex < targetPath.getSize() - 2){
          targetPathIndex++;
          lastTarget = currentTarget;
          currentTarget = targetPath[targetPathIndex];
          nextTarget = targetPath[targetPathIndex+1];
        }else{
          if(currentTarget != nextTarget){
            lastTarget = currentTarget;
            currentTarget = targetPath[targetPathIndex];
            nextTarget = currentTarget;
          }
        }
      }
    }

    if(error.getMagnitude() < errorRadius){
      errorTimer += deltaT;
      if(errorTimer > errorTime){
        atTarget = true;
      }
    }else{
      errorTimer = 0;
      atTarget = false;
    }

    double angularDifference = shortestArcToTarget(previousPos.head, currentTarget.head);

    double newOmega = angularDifference / deltaT;
    angularAcceleration = (newOmega - angularVelocity)/deltaT;
    angularVelocity = newOmega;

    if(abs(angularDifference) < stopAngularRadius){
      rotStopTimer += deltaT;
      if(rotStopTimer > stopTime){
        lastStoppedPos.head = currentPos.head;
        isStoppedRot = true;
      }
    }else{
      rotStopTimer = 0;
      isStoppedRot = false;
    }

    if(!headingIndependence){
      double targetHead = Vector(1, 0).getAngle(Vector(currentPos.p, currentTarget.p));
      if(!forward){
        targetHead = targetHead + PI;
        targetHead = normalizeAngle(targetHead);
      }
      currentTarget.head = targetHead;
    }

    double angularError = abs(shortestArcToTarget(currentPos.head, currentTarget.head));
    if(angularError < stopAngularRadius){
      rotErrorTimer += deltaT;
      if(rotErrorTimer > errorTime){
        atRotTarget = true;
      }
    }else{
      rotErrorTimer = 0;
      atRotTarget = false;
    }
    
    previousPos = currentPos;
  }

  //Targeting setters
  void setAbsTarget(double deltaX, double deltaY){
    setAbsTarget(Vector(deltaX, deltaY));
  }
  void setAbsTarget(Vector v){
    setAbsTarget(v + currentTarget.p);
  }
  void setAbsTarget(Point p){
    double targetHead = Vector(1, 0).getAngle(Vector(currentPos.p, p));
    if(!forward){
      targetHead = targetHead + PI;
      targetHead = normalizeAngle(targetHead);
    }

    headingIndependence = false;

    lastTarget = currentTarget;
    currentTarget = {p, targetHead};
    nextTarget = currentTarget;

    targetPath.clear();
    targetPathIndex = 0;
    targetPath.append({p, targetHead});

    atTarget = false;
  }

  void setAbsTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false){
    setAbsTarget(Vector(deltaX, deltaY), targetHead, inDeg);
  }
  void setAbsTarget(Vector v, double targetHead, bool inDeg=false){
    setAbsTarget(v + currentTarget.p, targetHead, inDeg);
  }
  void setAbsTarget(Point p, double targetHead, bool inDeg=false){
    headingIndependence = true;

    if(inDeg){
      targetHead = degToRad(targetHead);
    }
    targetHead = normalizeAngle(targetHead);

    lastTarget = currentTarget;
    currentTarget = {p, targetHead};
    nextTarget = currentTarget;

    targetPath.clear();
    targetPathIndex = 0;
    targetPath.append({p, targetHead});

    atTarget = false;
  }

  void setRelTarget(double deltaX, double deltaY){
    setRelTarget(Vector(deltaX, deltaY));
  }
  void setRelTarget(Vector v){
    v.getRotatedVector(currentTarget.head);
    setAbsTarget(v);
  }
  void setRelTarget(double deltaX, double deltaY, double targetHead, bool inDeg=false){
    setRelTarget(Vector(deltaX, deltaY), targetHead, inDeg);
  }
  void setRelTarget(Vector v, double targetHead, bool inDeg=false){
    v.getRotatedVector(currentTarget.head);
    setAbsTarget(v, targetHead, inDeg);
  }

  void turnTo(double head, bool inDeg=false){
    headingIndependence = true;
    if(inDeg){
      head = degToRad(head);
    }
    head = normalizeAngle(head);
    currentTarget.head = head;
  }
  void turn(double deltaHeading, bool inDeg=false){
    if(inDeg){
      deltaHeading = degToRad(deltaHeading);
    }
    turnTo(currentTarget.head + deltaHeading);
  }
  void face(Point p){
    turnTo(Vector(1, 0).getAngle(Vector(currentTarget.p, p)));
  }
  void face(Vector v){
    turnTo(Vector(1, 0).getAngle(v));
  }

  Vector translateGlobalToLocal(Vector v){
    return v.getRotatedVector(PI*0.5 - currentPos.head);
  }
  Vector translateLocalToGlobal(Vector v){
    return v.getRotatedVector(currentPos.head - PI*0.5);
  }

  Vector getRobotNormalVector(){
    return Vector(1, currentPos.head, false);
  }
  Vector getLocalError(){
    return translateGlobalToLocal(getGlobalError());
  }
  Vector getReverseLocalError(){
    return translateGlobalToLocal(getReverseGlobalError());
  }
  Vector getNextLocalError(){
    return translateGlobalToLocal(getNextGlobalError());
  }
  Vector getGlobalError(){
    return Vector(currentPos.p, currentTarget.p);
  }
  Vector getReverseGlobalError(){
    return Vector(lastStoppedPos.p, currentTarget.p);
  }
  Vector getNextGlobalError(){
    return Vector(currentTarget.p, nextTarget.p);
  }
  double getHeadError(){
    return shortestArcToTarget(currentPos.head, currentTarget.head);
  }
  double getReverseHeadError(){
    return shortestArcToTarget(lastStoppedPos.head, currentPos.head);
  }
  double getTranslationalGlobalHeading(){
    return Vector(1, 0).getAngle(getGlobalError());
  }
  double getTranslationalLocalHeading(){
    return Vector(1, 0).getAngle(getLocalError());
  }
  Vector getGlobalVelocity(){
    return tangentialVelocity;
  }
  Vector getLocalVelocity(){
    return translateGlobalToLocal(tangentialVelocity);
  }
  Vector getGlobalAcceleration(){
    return tangentialAcceleration;
  }
  Vector getLocalAcceleration(){
    return translateGlobalToLocal(tangentialAcceleration);
  }
  double getAngularVelocity(){
    return angularVelocity;
  }

  bool isMoving(){
    return !isStopped;
  }
  bool isTurning(){
    return !isStoppedRot;
  }
  bool isAtTarget(){
    return atTarget;
  }
  bool isAtRotTarget(){
    return atRotTarget;
  }
};

#endif