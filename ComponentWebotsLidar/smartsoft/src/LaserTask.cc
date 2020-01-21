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

  /*
  _default_base_position.set_x(COMP->getGlobalState().getBase_manipulator().getX());
  _default_base_position.set_y(COMP->getGlobalState().getBase_manipulator().getY());
  _default_base_position.set_z(COMP->getGlobalState().getBase_manipulator().getZ());
  _default_base_position.set_base_azimuth(COMP->getGlobalState().getBase_manipulator().getBase_a());
  //_default_base_position.set_steer_alpha(COMP->getGlobalState().getBase_manipulator().getSteer_a());
  //_default_base_position.set_turret_alpha(COMP->getGlobalState().getBase_manipulator().getTurret_a());

  // set scan id counter to 0
  scan_id = 0;
  */
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
  webots::Device *webotsDevice = NULL;
  int lidarIndex = 0;
  std::string lidarName;

  for(int i=0; i<webotsRobot->getNumberOfDevices(); i++) {
    webotsDevice = webotsRobot->getDeviceByIndex(i);
    if (webotsDevice->getNodeType() == webots::Node::LIDAR) {
      std::cout<<"Device #"<<i<<" called "<<webotsDevice->getName()<<" is a Lidar."<<std::endl;
      lidarIndex = i;
      lidarName = webotsDevice->getName();
    }
  }

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
  scan.set_min_distance(webotsLidar->getMinRange()*M_TO_CM);
  scan.set_max_distance(webotsLidar->getMaxRange()*M_TO_CM);
  scan.set_scan_length_unit(MEASURE_UNIT);

  return 0;
}


int LaserTask::on_execute()
{
  // this method is called from an outside loop,
  // hence, NEVER use an infinite loop (like "while(1)") here inside!!!
  // also do not use blocking calls which do not result from smartsoft kernel

	Smart::StatusCode laser_status;
	Smart::StatusCode base_status;

  // controller code that is in "while loop" if run from Simulator should be inside "if statement" below,
  // otherwise the values will not be updated
  if (webotsRobot->step(webotsTimeStep) != -1) {

  	/* Not working
    CommBasicObjects::CommBaseState currentBaseState;
  	CommBasicObjects::CommBasePose basePose;
  	CommBasicObjects::CommBaseVelocity baseVelocity;
  	COMP->LaserMutex.acquire();
    currentBaseState = COMP->baseState;
  	basePose = COMP->baseState.get_base_position();
		baseVelocity = COMP->baseState.get_base_velocity();
		std::cout << "" << std::endl;
    std::cout << "[LID] baseState : " << currentBaseState << std::endl;
		std::cout << "[LID] basePose  : " << basePose<< std::endl;
		std::cout << "[LID] basePoseX : " << basePose.get_x()<< std::endl;
		std::cout << "[LID] basePoseY : " << basePose.get_y()<< std::endl;
		std::cout << "[LID] basePoseZ : " << basePose.get_z()<< std::endl;
		std::cout << "[LID] baseVelo  : " << baseVelocity<< std::endl;
		// Do stuff with if necessary
		COMP->LaserMutex.release();
		*/

		// TODO: Otherwise this should be tested

  	double basePosX = 0.0;
  	double basePosY = 0.0;
  	double basePosZ = 0.0;
  	//double basePosAzim = 0.0;
  	//double basePosElev = 0.0;
  	//double basePosRoll = 0.0;
  	//double baseAbsoluteVelocity = 0.0;

  	// get base state from port
  	//base_status = COMP->baseStateServiceIn->getUpdate(base_state);
  	base_status = this->baseStateServiceInGetUpdate(base_state);

		// check if the transmission worked
		if (base_status != Smart::SMART_OK)
			std::cerr << "Error: receiving base state: " << base_status << std::endl;
		else
			std::cout << "LaserScan received" << std::endl;

		//scan.set_base_state(base_state);
		basePosX = base_state.get_base_position().get_x(1.0);
		basePosY = base_state.get_base_position().get_y(1.0);
		basePosZ = base_state.get_base_position().get_z(1.0);
		//basePosAzim = base_state.get_base_position().get_base_azimuth();
		//basePosElev = base_state.get_base_position().get_base_elevation();
		//basePosRoll = base_state.get_base_position().get_base_roll();

  	std::cout << " " << std::endl;
  	std::cout << "[LID] pos_x : " << basePosX << std::endl;
  	std::cout << "[LID] pos_y : " << basePosY << std::endl;
  	std::cout << "[LID] pos_z : " << basePosZ << std::endl;



    // time settings and update scan count
    timeval _receiveTime;
    gettimeofday(&_receiveTime, 0);
    scan.set_scan_time_stamp(CommBasicObjects::CommTimeStamp(_receiveTime));
    scan.set_scan_update_count(scanCount);

    // get sensor's values from Webots side
    const float *rangeImageVector;
    rangeImageVector = (const float *)(void *)webotsLidar->getRangeImage(); // in m

    // pass sensor's values to SmartMDSD side
    for(unsigned int i=0; i<numberValidPoints; ++i)
    {
      // it is in cm due to LaserScanPoint structure definition
      unsigned int dist = (unsigned int)(rangeImageVector[i]*M_TO_CM);
      scan.set_scan_index(i, i);
      scan.set_scan_integer_distance(i, dist); // in cm
    }
    scan.set_scan_valid(true);
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
