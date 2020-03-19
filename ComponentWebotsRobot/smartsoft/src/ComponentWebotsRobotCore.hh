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

#ifndef _COMPONENTWEBOTSROBOTCORE_HH
#define _COMPONENTWEBOTSROBOTCORE_HH

#include "aceSmartSoft.hh"

#include <iostream>
#include <memory.h>

#include <jsoncpp/json/json.h>

#include <webots/Robot.hpp>
#include "webots/Supervisor.hpp"
#include "webots/Node.hpp"
#include "webots/Field.hpp"
#include "webots/Device.hpp"
#include "webots/GPS.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Motor.hpp"

using namespace std;

class ComponentWebotsRobotCore
{
private:
  void checkSupervisor();
  void initDevices();
  void getNavigationMotors();

public:
  ComponentWebotsRobotCore();
  ~ComponentWebotsRobotCore();

  SmartACE::SmartMutex mRobotMutex;
  double mVX;
  double mVY;
  double mOmega;
  Json::Value mConfiguration;
  bool has_supervisor;
  bool battery_out;

  // Webots supervisor or Robot
  shared_ptr<webots::Supervisor> _supervisor;

  // webots devices
  webots::GPS* _gps;
  webots::InertialUnit* _imu;
  std::vector<webots::Motor*> motors;
  std::map<std::string, webots::Motor *> navigation_motors;


};

#endif
