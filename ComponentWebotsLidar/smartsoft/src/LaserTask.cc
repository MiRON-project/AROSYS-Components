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
#include "LaserTask.hh"
#include "ComponentWebotsLidar.hh"

#include <iostream>
#include <cstdint>
#include <climits>

#include <webots/Device.hpp>
#include <webots/Node.hpp>


LaserTask::LaserTask(SmartACE::SmartComponent *comp)
: LaserTaskCore(comp)
{
  std::cout << "constructor LaserTask\n";
}
LaserTask::~LaserTask()
{
  std::cout << "destructor LaserTask\n";
}


int LaserTask::on_entry()
{
  // do initialization procedures here, which are called once, each time the task is started
  // it is possible to return != 0 (e.g. when initialization fails) then the task is not executed further


  // assign this controller to the correct robot in Webots
  char *robotName = std::getenv("WEBOTS_ROBOT_NAME");
  if (!robotName) {
    std::cout  << "WEBOTS_ROBOT_NAME not defined" << std::endl;
    FILE *f = fopen("robotName.txt", "rb");
    if (!f) {
      std::cout  << "'robotName.txt' file not found." << std::endl;
      return -1;
    }
    char name[256];
    int ret = fscanf(f, "%[^\n]", name);
    if (ret == 0) {
      std::cout  << "First line of the 'robotName.txt' file is empty." << std::endl;
      return -1;
    }
    char environment[256] = "WEBOTS_ROBOT_NAME=";
    putenv(strcat(environment, name));
  }

  // create Robot Instance
  webotsRobot = new webots::Robot();

  // get timestep from the world
  webotsTimeStep = webotsRobot->getBasicTimeStep();

  // connect to the sensor from Webots
  LidarFound = false;
  std::string lidarName;
  webots::Device *webotsDevice = NULL;

  for(int i=0; i<webotsRobot->getNumberOfDevices(); i++) {
    webotsDevice = webotsRobot->getDeviceByIndex(i);
    if (webotsDevice->getNodeType() == webots::Node::LIDAR) {
      LidarFound = true;
      lidarName = webotsDevice->getName();
      std::cout<<"Device #"<<i<<" called "<<webotsDevice->getName()<<" is a Lidar."<<std::endl;
      break;
    }
  }

  if(LidarFound){
  	// enable the lidar
    webotsLidar = webotsRobot->getLidar(lidarName);
    webotsLidar->enable(webotsTimeStep);
    webotsLidar->enablePointCloud();

    // set Webots sensor's properties to SmartMDSD model
    // useful doc: http://servicerobotik-ulm.de/drupal/doxygen/components_commrep/classCommBasicObjects_1_1CommMobileLaserScan.html
    scanCount = 0;
    horizontalResolution = webotsLidar->getHorizontalResolution();
    numberValidPoints = webotsLidar->getNumberOfPoints();
    scan.set_scan_size(numberValidPoints);
    scan.set_scan_update_count(scanCount);
    scan.set_scan_integer_field_of_view(-horizontalResolution*UNIT_FACTOR/2.0, horizontalResolution*UNIT_FACTOR);
    // Pay attention to limits as min/max_distance variables are short type (max value is 65535)
    if (webotsLidar->getMaxRange()*M_TO_MM > SHORT_LIMIT) {
    	std::cout  << "The lidar range is bigger than 65.535 meters and will be set to 65 meters." << std::endl;
    	scan.set_max_distance(65*M_TO_MM);
    }
    else
    	scan.set_max_distance(webotsLidar->getMaxRange()*M_TO_MM);
		scan.set_min_distance(webotsLidar->getMinRange()*M_TO_MM);
    scan.set_scan_length_unit(MEASURE_UNIT);
  }
  else
  		std::cout  << "No lidar found, no data is sent." << std::endl;

  return 0;
}


int LaserTask::on_execute()
{
  // this method is called from an outside loop,
  // hence, NEVER use an infinite loop (like "while(1)") here inside!!!
  // also do not use blocking calls which do not result from smartsoft kernel


  // controller code that is in "while loop" if run from Simulator should be inside "if statement" below,
  // otherwise the values will not be updated
  if (webotsRobot->step(webotsTimeStep) != -1) {
    // Some variables are set but not implemented now
    double basePosX = 0.0;
    double basePosY = 0.0;
    double basePosZ = 0.0;
    double basePosAzim = 0.0;
    double basePosElev = 0.0;
    double basePosRoll = 0.0;

    // get base state from port
    Smart::StatusCode baseStatus = this->baseStateServiceInGetUpdate(baseState);

    // check if the transmission worked
    if (baseStatus != Smart::SMART_OK)
      std::cerr << "Error: receiving base state: " << baseStatus << std::endl;
    else
      std::cout << "LaserScan received" << std::endl;

    basePosX = baseState.get_base_position().get_x(1.0);
    basePosY = baseState.get_base_position().get_y(1.0);
    basePosZ = baseState.get_base_position().get_z(1.0);
    basePosAzim = baseState.get_base_position().get_base_azimuth();
    basePosElev = baseState.get_base_position().get_base_elevation();
    basePosRoll = baseState.get_base_position().get_base_roll();

    // print data to debug
    //std::cout << " " << std::endl;
    //std::cout << "basePosX " << basePosX << std::endl;
    //std::cout << "basePosY " << basePosY << std::endl;
    //std::cout << "basePosZ " << basePosZ << std::endl;
    //std::cout << "basePosAzim " << basePosAzim << std::endl;
    //std::cout << "basePosElev " << basePosElev << std::endl;
    //std::cout << "basePosRoll " << basePosRoll << std::endl;

    if(LidarFound){
			// time settings and update scan count
			timeval _receiveTime;
			gettimeofday(&_receiveTime, 0);
			scan.set_scan_time_stamp(CommBasicObjects::CommTimeStamp(_receiveTime));
			scan.set_scan_update_count(scanCount);

			// get sensor's values from Webots side
			const float *rangeImageVector;
			rangeImageVector = (const float *)(void *)webotsLidar->getRangeImage(); // in m

			// pass sensor's values to SmartMDSD side
			for(unsigned int i=0; i<numberValidPoints; ++i) {
				// Pay attention to
				//   o limits as min/max_distance variables are short type (max value is 65535)
				//   o same remark for the distance (max value is 65535)
				//   o Webots array for lidar value is inverted with the one in Smartsoft
				unsigned int dist = (unsigned int)(rangeImageVector[numberValidPoints-1-i]*M_TO_MM);
				scan.set_scan_index(i, i);
				scan.set_scan_integer_distance(i, dist); // in mm
				// Print distance to debug
				//if (i%6==0)
					//std::cout << "["<<i<<"] " << dist << std::endl;
			}
			scan.set_scan_valid(true);
    }
    else
    	scan.set_scan_valid(false);
  }
  else
    return -1;

  // send out laser scan through port
  laserServiceOutPut(scan);
  ++scanCount;
  scan.set_scan_valid(false);

  // it is possible to return != 0 (e.g. when the task detects errors), then the outer loop breaks and the task stops
  return 0;
}


int LaserTask::on_exit()
{
  delete webotsRobot;

  // use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
  return 0;
}
