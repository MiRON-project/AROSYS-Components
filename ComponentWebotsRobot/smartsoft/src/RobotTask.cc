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

#include "RobotTask.hh"

#include "ComponentWebotsRobot.hh"

#include <iostream>

void differentialWheelVelocityController(webots::Motor *left_wheel, 
  webots::Motor *right_wheel, std::array<double, 2> vel)
{
  double speed = vel[0];
  double omega = vel[1];
  double max_vel = std::min(left_wheel->getMaxVelocity(), 
    right_wheel->getMaxVelocity());
  
  double right_wheel_coef = (omega + 2 * speed) / (2 * max_vel);
  double left_wheel_coef = (2 * speed - omega) / (2 * max_vel);

  double left_wheel_vel = left_wheel_coef * max_vel;
  double right_wheel_vel = right_wheel_coef * max_vel;
  if (abs(left_wheel_vel) > max_vel)
    left_wheel_vel = ((left_wheel_vel > 0) - (left_wheel_vel < 0)) * 
      max_vel;
  if (abs(right_wheel_vel) > max_vel)
    right_wheel_vel = ((right_wheel_vel > 0) - (right_wheel_vel < 0)) * 
      max_vel;
  
  left_wheel->setVelocity(left_wheel_vel);
  right_wheel->setVelocity(right_wheel_vel);
}

RobotTask::RobotTask(SmartACE::SmartComponent *comp) :
  RobotTaskCore(comp),
  mThread(),
  mThreadRunning(false),
  mWebotsShouldQuit(false),
  robot_duration(computeWebotsControlDuration()),
  left_wheel(NULL),
  right_wheel(NULL)
{
  for (std::map<std::string, webots::Motor *>::iterator it = 
        COMP->navigation_motors.begin();
      it != COMP->navigation_motors.end(); ++it)
  {
    const std::string name = it->first;
    const Json::Value coefficients = COMP->mConfiguration["navigationVelocity"][name];
    if (!coefficients.isArray() || coefficients.size() != 3 || 
        !coefficients[0].isDouble() || !coefficients[1].isDouble() ||
        !coefficients[2].isDouble())
    {
      std::cerr << "Wrong value for the 'navigationVelocity." << name << 
        "' key, the value should be a array of 3 doubles." << std::endl;
      break;
    }

    if (coefficients[2].asDouble() > 0)
      right_wheel = it->second;
    if (coefficients[2].asDouble() < 0)
      left_wheel = it->second;
  }
  std::cout << "constructor RobotTask\n";
}
RobotTask::~RobotTask()
{
  delete left_wheel;
  delete right_wheel;
  std::cout << "destructor RobotTask\n";
}

int RobotTask::on_entry()
{
  if (!COMP->_supervisor)
    return -1;

  COMP->mRobotMutex.acquire();

  if (COMP->_gps)
    COMP->_gps->enable(robot_duration);
  if (COMP->_imu)
    COMP->_imu->enable(robot_duration);

  COMP->mRobotMutex.release();

  return 0;
}

int RobotTask::on_execute()
{
  if (mWebotsShouldQuit)
    return -1;

  if (mThreadRunning || !COMP->_supervisor)
    return 0;

  double speed = 0.0;
  double omega = 0.0;
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  
  COMP->mRobotMutex.acquire();

  CommBasicObjects::CommBaseState baseState = setBaseStateServiceOut();
  baseStateServiceOutPut(baseState);

  // Calculate wheel velocities using differential wheels
  if (left_wheel && right_wheel)
    differentialWheelVelocityController(left_wheel, right_wheel, 
      {COMP->mVX, COMP->mOmega});

  // start robot step thread
  mThreadRunning = true;
  if (mThread.joinable())
    mThread.join();
  mThread = std::thread(&RobotTask::runStep, this);

  // release
  COMP->mRobotMutex.release();

  return 0;
}

int RobotTask::on_exit()
{
  return 0;
}

void RobotTask::runStep()
{
  mWebotsShouldQuit = COMP->_supervisor->step(robot_duration) 
    == -1.0;
  mThreadRunning = false;
}

int RobotTask::computeWebotsControlDuration() const
{
  double simulation_step = COMP->_supervisor->getBasicTimeStep();
  double robot_control_step = 1000.0 / 
    COMP->connections.robotTask.periodicActFreq;
  int duration = (int) (
    ceil(robot_control_step / simulation_step) * simulation_step);
  std::cout << "Robot duration is: " << duration << "\n";
  return duration;
}

CommBasicObjects::CommBaseState RobotTask::setBaseStateServiceOut() const
{
  CommBasicObjects::CommBaseState baseState;
  CommBasicObjects::CommBasePose basePosition;

  // set GPS values for port BaseStateServiceOut
  if (COMP->_gps)
  {
    auto GPS_value = COMP->_gps->getValues();
    basePosition.set_x(GPS_value[2], 1.0);
    basePosition.set_y(GPS_value[0], 1.0);
    basePosition.set_z(GPS_value[1], 1.0);
    baseState.set_base_position(basePosition);
  }
  else
  {
    basePosition.set_x(0.0, 1.0);
    basePosition.set_y(0.0, 1.0);
    basePosition.set_z(0.0, 1.0);
    baseState.set_base_position(basePosition);
  }

  // Set IMU values for port BaseStateServiceOut
  // Webots use the NED convention, see https://cyberbotics.com/doc/reference/inertialunit
  // Smartsoft use ???, see ???
  // ROS use ENU convention, https://www.ros.org/reps/rep-0103.html
  // Be aware of this in your calculation
  if (COMP->_imu)
  {
    auto inertialInutValue = COMP->_imu->getRollPitchYaw();
    basePosition.set_base_roll(inertialInutValue[0]);
    basePosition.set_base_azimuth(inertialInutValue[2]);
    basePosition.set_base_elevation(inertialInutValue[1]);
    baseState.set_base_position(basePosition);
  }
  else
  {
    basePosition.set_base_roll(0.0);
    basePosition.set_base_azimuth(0.0);
    basePosition.set_base_elevation(0.0);
    baseState.set_base_position(basePosition);
  }
  return baseState;
}