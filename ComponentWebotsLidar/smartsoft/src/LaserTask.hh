//--------------------------------------------------------------------------
// Code generated by the SmartSoft MDSD Toolchain
// The SmartSoft Toolchain has been developed by:
//
// Service Robotics Research Center
// University of Applied Sciences Ulm
// Prittwitzstr. 10
// 89075 Ulm (Germany)
//
// Information about the SmartSoft MDSD Toolchain is available at:
// www.servicerobotik-ulm.de
//
// This file is generated once. Modify this file to your needs.
// If you want the toolchain to re-generate this file, please
// delete it before running the code generator.
//--------------------------------------------------------------------------
#ifndef _LASERTASK_HH
#define _LASERTASK_HH

#include "LaserTaskCore.hh"
#include "CommBasicObjects/CommBasePose.hh"
#include "CommBasicObjects/CommMobileLaserScan.hh"

#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>


// modify these parameters for unit consistency
#define M_TO_CM      100.0 // Conversion factor
#define UNIT_FACTOR  100.0 // Value is expressed in 0.01 degree units
#define MEASURE_UNIT 1.0   // Set the internal length unit of the scan in mm


class LaserTask  : public LaserTaskCore
{
private:
  // webots parameters
  int wbTimeStep;
  int horizontalResolution;
  webots::Robot *webotsRobot;
  webots::Lidar *webotsLidar;

  // laser parameters
  unsigned int numberValidPoints;
  CommBasicObjects::CommMobileLaserScan scan;

  // position of scanner on robot
  double _scanner_x;
  double _scanner_y;
  double _scanner_z;
  double _scanner_a; // azimuth relative to coordinate system of mounting point
  CommBasicObjects::CommBaseState base_state;


public:
  unsigned long scanCount;
  bool accquireNewScan(CommBasicObjects::CommMobileLaserScan& scan);

  LaserTask(SmartACE::SmartComponent *comp);
  virtual ~LaserTask();

  virtual int on_entry();
  virtual int on_execute();
  virtual int on_exit();
};

#endif
