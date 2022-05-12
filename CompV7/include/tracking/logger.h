#ifndef __LOGGER_H__
#define __LOGGER_H__

/*
Name: logger.h
Written By: Carson Easterling
*/

#include <iostream>
#include <sstream>
#include "robotmath.h"
#include <fstream>

class Log : public std::ostringstream{
public:
  const char* fileName;

  Log(const char* logName){
    fileName = logName;
    str("");
  }

  void clear(){
    str("");
  }

  void print(bool clearBuf=true){
    std::cout << str() << std::flush;
    if(clearBuf){
      clear();
    }
  }

  void save(bool clearBuf=true){
    std::ofstream file(fileName);
    if(file.is_open()){
      file.clear();
      file << str() << std::flush;
      file.close();
    }
    if(clearBuf){
      clear();
    }
  }

  void append(bool clearBuf=true){
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()){
      file << str() << std::flush;
      file.close();

      if(clearBuf){
        clear();
      }
    }
  }
};

#endif