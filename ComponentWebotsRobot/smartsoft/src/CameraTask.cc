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
#include "CameraTask.hh"
#include "ComponentWebotsRobot.hh"

#include <iostream>

CameraTask::CameraTask(SmartACE::SmartComponent *comp) 
:	CameraTaskCore(comp),
	image_counter(0),
	_camera(NULL)
{
	std::cout << "constructor CameraTask\n";
}
CameraTask::~CameraTask() 
{
	delete _camera;
	std::cout << "destructor CameraTask\n";
}



int CameraTask::on_entry()
{
	if (!COMP->_supervisor || 
		!COMP->getParameters().getCamera_properties().getEnable())
		return -1;

	COMP->mRobotMutex.acquire();

	for (int i = 0; i < COMP->_supervisor->getNumberOfDevices(); i++)
	{
		auto webotsDevice = COMP->_supervisor->getDeviceByIndex(i);

		if (webotsDevice->getNodeType() == webots::Node::CAMERA)
		{
			_camera = dynamic_cast<webots::Camera*>(webotsDevice);
			_camera->enable(computeCameraUpdate());
			if (COMP->getParameters().getCamera_properties().
				getEnable_recognition())
				_camera->recognitionEnable(computeCameraUpdate());
			break;
		}
	}
	if (_camera == NULL)
	{
		std::cerr << "You enable a Camera, but robot has not such device!" 
			<< std::endl;
		return -1;
	}

	// release
	COMP->mRobotMutex.release();

	return 0;
}
int CameraTask::on_execute()
{
	if (!_camera || !COMP->_supervisor)
		return -1;

	COMP->mRobotMutex.acquire();

	auto image = _camera->getImage();
	if (image)
	{
		unsigned int cols = _camera->getWidth();
		unsigned int rows = _camera->getHeight();
		++image_counter;
		comm_video_image.setSeq_count(image_counter);
		comm_video_image.set_parameters(cols, rows, 
			DomainVision::FormatType::RGB32);
		comm_video_image.set_data(image);
		comm_video_image.setIs_valid(true);
		rgbd_image.setColor_image(comm_video_image);
		rgbd_image.setSeq_count(image_counter);
		rgbd_image.setIs_valid(true);

		if (COMP->getParameters().getCamera_properties().
				getEnable_recognition())
			recognition();
	}

	else
		rgbd_image.setIs_valid(false);

	// send out image through port
	COMP->rGBImagePushServiceOut->put(rgbd_image.getColor_image());

	COMP->mRobotMutex.release();
	return 0;
}
int CameraTask::on_exit()
{
	// use this method to clean-up resources which are initialized in on_entry() and needs to be freed before the on_execute() can be called again
	return 0;
}

int CameraTask::computeCameraUpdate() const
{
	double simulation_step = COMP->_supervisor->getBasicTimeStep();
	double camera_step = 1000.0 / 
		COMP->connections.cameraTask.periodicActFreq;
	int duration = (int) (
		ceil(camera_step / simulation_step) * simulation_step);
	std::cout << "Camera duration is: " <<  duration << "\n";
	return duration;
}

void CameraTask::recognition()
{
	CommObjectRecognitionObjects::CommObjectRecognitionObjectProperties 
		obj_properties;
	int number_of_objects = _camera->getRecognitionNumberOfObjects();
	printf("\nRecognized %d objects.\n", number_of_objects);

	auto objects = _camera->getRecognitionObjects();
	for (int i = 0; i < number_of_objects; ++i) 
	{
		printf("Model of object %d: %s\n", i, objects[i].model);
		obj_properties.setObject_type(objects[i].model);

		printf("Id of object %d: %d\n", i, objects[i].id);
		obj_properties.setObject_id(objects[i].id);

		printf("Relative position of object %d: %lf %lf %lf\n", i, 
			objects[i].position[0], objects[i].position[1], 
			objects[i].position[2]);
		printf("Relative orientation of object %d: %lf %lf %lf %lf\n", i, 
			objects[i].orientation[0], objects[i].orientation[1],
			objects[i].orientation[2], objects[i].orientation[3]);
		Quaternion quat(objects[i].orientation[0], objects[i].orientation[1],
			objects[i].orientation[2], objects[i].orientation[3]);
		auto euler_angles = ToEulerAngles(quat);
		CommBasicObjects::CommPose3d obj_pose(objects[i].position[0],
			objects[i].position[1], objects[i].position[2], euler_angles.yaw, 
			euler_angles.pitch, euler_angles.roll);
		obj_properties.setPose(obj_pose);
		
		printf("Size of object %d: %lf %lf\n", i, objects[i].size[0], 
			objects[i].size[1]);
		obj_properties.set_dimension(objects[i].size[0], objects[i].size[1], 
			0);
		
		printf("Position of the object %d on the camera image: %d %d\n", i, 
			objects[i].position_on_image[0], objects[i].position_on_image[1]);
		printf("Size of the object %d on the camera image: %d %d\n", i, 
			objects[i].size_on_image[0], objects[i].size_on_image[1]);
		for (int j = 0; j < objects[i].number_of_colors; ++j)
			printf("- Color %d/%d: %lf %lf %lf\n", j + 1, 
				objects[i].number_of_colors, objects[i].colors[3 * j],
				objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
		obj_properties.setIs_valid(true);
		COMP->objectPushServiceOut->put(obj_properties);
	}
}