// Copyright 1996-2020 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ComponentWebotsRobotCore.hh"

#include <fstream>

// constructor
ComponentWebotsRobotCore::ComponentWebotsRobotCore() :
  _supervisor(NULL),
  _imu(NULL),
  _gps(NULL)
{
  

  std::ifstream file_input("configuration.json");
  if (!file_input.is_open())
  {
    std::cerr << "Can't open 'configuration.json' file." << std::endl;
    return;
  }

  Json::Reader reader;
  if (!reader.parse(file_input, mConfiguration))
  {
    std::cerr << "Invalid 'configuration.json' file." << std::endl;
    return;
  }

  // assign this controller to the correct robot in Webots
  char *robotName = std::getenv("WEBOTS_ROBOT_NAME");
  if (!robotName)
  {
    if (!mConfiguration.isMember("name") || !mConfiguration["name"].isString())
    {
      std::cerr << "Missing or invalid 'name' key in 'configuration.json' file." << std::endl;
      return;
    }
    char environment[256] = "WEBOTS_ROBOT_NAME=";
    putenv(strcat(environment, mConfiguration["name"].asCString()));
  }

  // create Robot instance
  _supervisor = make_shared<webots::Supervisor>();
  checkSupervisor();
  initDevices();
  getNavigationMotors();
  battery_out = false;
}

ComponentWebotsRobotCore::~ComponentWebotsRobotCore()
{
  delete _gps;
  delete _imu;
  for (auto m : motors)
		delete m;
	motors.clear();
  for (auto& m : navigation_motors)
    delete m.second;
}

void ComponentWebotsRobotCore::checkSupervisor()
{
  auto root = _supervisor->getRoot();
  if (!root)
  {
    has_supervisor = false;
    return;
  }
  has_supervisor = true;
}

void ComponentWebotsRobotCore::initDevices()
{
  _gps = NULL;
  _imu = NULL;
  motors.clear();

  for (int i = 0; i < _supervisor->getNumberOfDevices(); i++)
  {
    auto webotsDevice = _supervisor->getDeviceByIndex(i);
    if (webotsDevice->getNodeType() == webots::Node::GPS)
      _gps = dynamic_cast<webots::GPS*>(webotsDevice);

    else if (webotsDevice->getNodeType() == webots::Node::INERTIAL_UNIT)
      _imu = dynamic_cast<webots::InertialUnit*>(webotsDevice);

    else if (webotsDevice->getNodeType() == webots::Node::LINEAR_MOTOR ||
			webotsDevice->getNodeType() == webots::Node::ROTATIONAL_MOTOR)
			motors.push_back(dynamic_cast<webots::Motor*>(webotsDevice));
  }
}

void ComponentWebotsRobotCore::getNavigationMotors()
{
  if (mConfiguration.isMember("navigationVelocity") && 
    mConfiguration["navigationVelocity"].isObject())
  {
    const Json::Value velocityConfiguration = 
      mConfiguration["navigationVelocity"];
    const Json::Value::Members motorNames = 
      velocityConfiguration.getMemberNames();
    
    for (int i = 0; i < motorNames.size(); ++i)
    {
      webots::Motor *motor = _supervisor->getMotor(motorNames[i]);
      if (motor)
      {
        motor->setPosition(INFINITY);
        motor->setVelocity(0);
        navigation_motors[motorNames[i]] = motor;
      }
    }
  }
  else
  {
    std::cerr << "Missing or invalid 'navigationVelocity' key in" << 
      "'configuration.json' file." << std::endl;
  }
}