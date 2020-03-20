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
#include "LidarTask.hh"
#include "ComponentWebotsRobot.hh"

#include <iostream>

LidarTask::LidarTask(SmartACE::SmartComponent *comp) 
:	LidarTaskCore(comp),
	scanCount(0),
	_lidar(NULL)
{
	std::cout << "constructor LidarTask\n";
}
LidarTask::~LidarTask() 
{
	delete _lidar;
	std::cout << "destructor LidarTask\n";
}

int LidarTask::on_entry()
{
	if (!COMP->_supervisor || 
		!COMP->getGlobalState().getLidar_properties().getEnable())
    	return -1;
	
	COMP->mRobotMutex.acquire();

	for (int i = 0; i < COMP->_supervisor->getNumberOfDevices(); i++)
	{
		auto webotsDevice = COMP->_supervisor->getDeviceByIndex(i);
		if (webotsDevice->getNodeType() == webots::Node::LIDAR)
		{
			_lidar = dynamic_cast<webots::Lidar*>(webotsDevice);
			_lidar->enable(computeLidarUpdate());
			break;
		}
	}
	if (_lidar == NULL)
	{
		std::cerr << "You enable a Lidar, but robot has not such device!" 
		<< std::endl;
		return -1;
	}

	_lidar->enablePointCloud();
	horizontalResolution = _lidar->getHorizontalResolution();
	numberValidPoints = _lidar->getNumberOfPoints();
	scan.set_scan_size(numberValidPoints);
	scan.set_scan_update_count(scanCount);
	scan.set_scan_integer_field_of_view(
		- horizontalResolution * 100.0 / 2.0, 
		horizontalResolution * 100.0);
	scan.set_max_distance((int)
		std::min(65535.0, _lidar->getMaxRange() * 1000.0));
	scan.set_min_distance(_lidar->getMinRange() * 1000.0);
	scan.set_scan_length_unit(1);
	
	COMP->mRobotMutex.release();

	return 0;
}
int LidarTask::on_execute()
{
	if (!COMP->_supervisor || !_lidar)
		return -1;

	COMP->mRobotMutex.acquire();
	
	auto rangeImageVector = _lidar->getRangeImage();
	if (rangeImageVector)
	{
		++scanCount;
		timeval _receiveTime;
		::gettimeofday(&_receiveTime, 0);
		scan.set_scan_time_stamp(CommBasicObjects::CommTimeStamp(_receiveTime));
		scan.set_scan_update_count(scanCount);

		for (unsigned int i = 0; i < numberValidPoints; ++i)
		{
			unsigned int dist = (unsigned int)
				(rangeImageVector[numberValidPoints - 1 - i] * 1000.0);
			scan.set_scan_index(i, i);
			scan.set_scan_integer_distance(i, dist);
		}
		scan.set_scan_valid(true);
	}
	else
		scan.set_scan_valid(false);

	laserServiceOutPut(scan);

	COMP->mRobotMutex.release();
	
	return 0;
}
int LidarTask::on_exit()
{
	return 0;
}

int LidarTask::computeLidarUpdate() const
{
	double simulation_step = COMP->_supervisor->getBasicTimeStep();
	double lidar_step = 1000.0 / 
		COMP->connections.lidarTask.periodicActFreq;
	int duration = (int) (
		ceil(lidar_step / simulation_step) * simulation_step);
	std::cout << "Lidar duration is: " <<  duration << "\n";
	return duration;
}
