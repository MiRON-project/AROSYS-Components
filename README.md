# RobMoSys

## Components
The following components are new components for RobMoSys in order to be able to use them with the [Webots' simulator](https://cyberbotics.com).

#### ComponentWebots
This component allows the user to use Webots simulator instead of Gazebo, PlayerStage, etc. There exists a pre-requirement to use Webots. You have to execute this command once in a Terminal and restart the computer/Virtual Machine. Don't forget to adapt the path if necessary.
`echo -e "export WEBOTS_HOME=/home/smartsoft/SOFTWARE/webots" >> ~/.profile`
* **Ports and usage**
There is no port for this component. Just add it in your system to use Webot simulator.

#### ComponentWebotsLidar
This component configures lidar's sensors in SmartSoft according to Webots prototype sensors and provides the sensor's data.
* `input:` a sensor (such as a lidar)
* `output:` velocity commands (linear velocity and turnrate)

#### ComponentWebotsPioneer3DX
This component configures in SmartSoft the Pioneer 3-DX robot from Adept according to Webots prototype robot.

#### ComponentWebotsRobotino3
This component configures in SmartSoft the Robotino 3 robot from Festo according to Webots prototype robot.

#### ComponentWebotsTiago
This component configures in SmartSoft all versions of TIAGo's  robot from Pal Robotics according to Webots prototype robot.



## System
The following systems are examples to be able to test new components. It is important to mentionned that the lidar range has to be under 65.535 meter otherwise it will be set at 65 meter because the variable type to store the value is `short`. This comes from source files generated by SmartMDSD. Maintainers will be alerted to this problem so that it can be improved.

#### SystemLaserObstacleAvoidTiagoWebotsSimulator
This system is a copy of the `SystemLaserObstacleAvoidTiagoGazeboSimulator` in order to show that the new components developed to use Webots simulator are working since the robot has the same behavior as expected.

#### SystemWebotsPioneer3DXNavigation
This system shows a complete example of communication between a [lidar sensor](https://cyberbotics.com/doc/guide/lidar-sensors) (`ComponentWebotsLidar`) used by a [Pioneer 3-DX](https://cyberbotics.com/doc/guide/pioneer-3dx) robot (`ComponentWebotsPioneer3DX`) to navigate in a square arena filled with obstacles. It works with the new components developped to use Webots.

#### SystemWebotsRobotino3Navigation
This system shows a complete example of communication between a [lidar sensor](https://cyberbotics.com/doc/guide/lidar-sensors) (`ComponentWebotsLidar`) used by a [Robotino 3](https://cyberbotics.com/doc/guide/robotino3) robot (`ComponentWebotsPioneer3DX`) to navigate in a square arena filled with obstacles. It works with the new components developped to use Webots.
