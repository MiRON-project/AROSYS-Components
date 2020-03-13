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
#include "BatteryEventTask.hh"
#include "ComponentWebotsRobot.hh"
#include "CommBasicObjects/CommBatteryLevel.hh"

#include <iostream>

BatteryEventTask::BatteryEventTask(SmartACE::SmartComponent *comp) 
:	BatteryEventTaskCore(comp)
{
	std::cout << "constructor BatteryEventTask\n";
}
BatteryEventTask::~BatteryEventTask() 
{
	std::cout << "destructor BatteryEventTask\n";
}



int BatteryEventTask::on_entry()
{
	battery_level = COMP->getGlobalState().getBattery_properties().getBattery_level_init();
	last_sample_time = std::chrono::system_clock::now();
	return 0;
}
int BatteryEventTask::on_execute()
{
	COMP->mRobotMutex.acquire();

	CommBasicObjects::CommBatteryLevel bt_level;

	auto current_sample_time = std::chrono::system_clock::now();
	auto milliseconds = (double) std::chrono::duration_cast<std::chrono::milliseconds>(current_sample_time - last_sample_time).count();
	auto seconds = milliseconds / 1000.0;
	last_sample_time = current_sample_time;

	if(battery_level <= 0)
	{
		std::cout << "Battery Level is: " << battery_level << std::endl;
		bt_level.setChargeLevel(0);
	}
	else
	{
		if(abs(COMP->mVX) > 0 || abs(COMP->mVY) > 0 || abs(COMP->mOmega) > 0)
			battery_level = std::max(0.0, battery_level- seconds * COMP->getGlobalState().getBattery_properties().getMotor_consumption());

		battery_level = std::max(0.0, battery_level- seconds * COMP->getGlobalState().getBattery_properties().getCpu_consumption());
		std::cout << "Battery Level is: " << battery_level << std::endl;
		bt_level.setChargeLevel((int)battery_level);
	}
	COMP->batteryPushServiceOut->put(bt_level);
	COMP->mRobotMutex.release();

	return 0;
}
int BatteryEventTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}
