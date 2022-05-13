#ifndef __ROBOTMODELS_H__
#define __ROBOTMODELS_H__

/*
Name: robotModels.h
Written By: Carson Easterling


*/

#include "robotmath.h"
#include "logger.h"
#include "navigation.h"

class Robot{
protected:
  double robotDiameter;
  double UnitsPerRevolution;

public:
  Navigator nav;


};

class TankDrive{
private:
public:
  bool forward = true;

};

class XDrive {

};

class MecDrive{

};


#endif