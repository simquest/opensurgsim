// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/OpenNI/OpenNIScaffold.h"

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <OpenNI.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/OpenNI/OpenNIDevice.h"
#include "SurgSim/Framework/Barrier.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"


namespace SurgSim
{
namespace Devices
{

struct OpenNIScaffold::DeviceData
{
	explicit DeviceData(OpenNIDevice* device) :
		deviceObject(device)
	{
	}

	/// OpenNI device
	openni::Device camera;
	/// OpenNI Depth Video Stream
	openni::VideoStream depthStream;
	/// OpenNI RGB Image Video Stream
	openni::VideoStream colorStream;
	/// The corresponding device object.
	OpenNIDevice* const deviceObject;
};

struct OpenNIScaffold::StateData
{
	/// The list of known devices.
	std::list<std::unique_ptr<DeviceData>> activeDevices;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;
};

OpenNIScaffold::OpenNIScaffold() :
	SurgSim::Framework::BasicThread("OpenNI Scaffold"),
	m_state(new StateData),
	m_logger(SurgSim::Framework::Logger::getLogger("OpenNI"))
{
	setRate(100);
}

OpenNIScaffold::~OpenNIScaffold()
{
	openni::OpenNI::shutdown();
}

bool OpenNIScaffold::registerDevice(OpenNIDevice* device)
{
	bool success = true;
	if (!isRunning())
	{
		std::shared_ptr<SurgSim::Framework::Barrier> barrier = std::make_shared<SurgSim::Framework::Barrier>(2);
		start(barrier);
		barrier->wait(true); // Wait for initialize
		barrier->wait(true); // Wait for startup
		success = isInitialized();
	}

	if (success)
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		const std::string deviceName = device->getName();
		auto sameName = [&deviceName](const std::unique_ptr<DeviceData>& info)
		{
			return info->deviceObject->getName() == deviceName;
		};
		auto found = std::find_if(m_state->activeDevices.cbegin(), m_state->activeDevices.cend(), sameName);

		if (found == m_state->activeDevices.end())
		{
			std::unique_ptr<DeviceData> info(new DeviceData(device));
			success = doRegisterDevice(info.get());
			if (success)
			{
				SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Registered";
				m_state->activeDevices.emplace_back(std::move(info));
			}
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device when the same name, '" <<
				device->getName() << "', is already present!";
			success = false;
		}
	}

	return success;
}

bool OpenNIScaffold::doRegisterDevice(DeviceData* info)
{
	openni::Status status;

	status = info->camera.open(openni::ANY_DEVICE);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Could not connect to a camera: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->depthStream.create(info->camera, openni::SENSOR_DEPTH);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Could not find depth stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->colorStream.create(info->camera, openni::SENSOR_COLOR);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Could find color image stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->depthStream.start();
	if (status != openni::STATUS_OK || !info->depthStream.isValid())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Unable to start depth stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->colorStream.start();
	if (status != openni::STATUS_OK || !info->colorStream.isValid())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Unable to start color image stream: " << openni::OpenNI::getExtendedError();
		return false;
	}
	return true;
}

bool OpenNIScaffold::unregisterDevice(const OpenNIDevice* device)
{
	bool success = true;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		auto sameDevice = [device](const std::unique_ptr<DeviceData>& info)
		{
			return info->deviceObject == device;
		};
		auto info = std::find_if(m_state->activeDevices.begin(), m_state->activeDevices.end(), sameDevice);
		if (info != m_state->activeDevices.end())
		{
			success = doUnregisterDevice((*info).get());
			if (success)
			{
				SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Unregistered";
				m_state->activeDevices.erase(info);
			}
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Attempted to release a non-registered device named '" <<
				device->getName() << ".";
			success = false;
		}
	}

	if (success && isRunning() && m_state->activeDevices.size() == 0)
	{
		stop();
	}

	return success;
}

bool OpenNIScaffold::doUnregisterDevice(DeviceData* info)
{
	info->depthStream.destroy();
	info->colorStream.destroy();
	info->camera.close();
	return true;
}

bool OpenNIScaffold::doInitialize()
{
	openni::Status status = openni::OpenNI::initialize();
	bool success = (status == openni::STATUS_OK);
	SURGSIM_LOG_IF(!success, m_logger, SEVERE) << "Initialize failed: " << openni::OpenNI::getExtendedError();
	return success;
}

bool OpenNIScaffold::doStartUp()
{
	return true;
}

bool OpenNIScaffold::doUpdate(double dt)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	typedef SurgSim::DataStructures::DataGroup::ImageType ImageType;

	for (auto info = m_state->activeDevices.begin();  info != m_state->activeDevices.end();  ++info)
	{
		SurgSim::DataStructures::DataGroup& inputData = (*info)->deviceObject->getInputData();

		if ((*info)->depthStream.readFrame(&depthFrame) == openni::STATUS_OK)
		{
			size_t width = depthFrame.getWidth();
			size_t height = depthFrame.getHeight();
			typedef Eigen::Map<const Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>> DepthDataType;
			DepthDataType depthData(reinterpret_cast<const uint16_t*>(depthFrame.getData()), width, height);

			ImageType depth(width, height, 1);
			depth.getAsVector() = depthData.cast<float>() * (1.0f / 1000.0f); // convert milimeters to meters
			inputData.images().set("depth", std::move(depth));

			ImageType depth_xyz(width, height, 3);
			auto x = depth_xyz.getChannel(0);
			auto y = depth_xyz.getChannel(1);
			auto z = depth_xyz.getChannel(2);
			for (size_t i = 0; i < width; i++)
			{
				for (size_t j = 0; j < height; j++)
				{
					openni::CoordinateConverter::convertDepthToWorld((*info)->depthStream, i, j, depthData(i, j),
							&x(i, j), &y(i, j), &z(i, j));
				}
			}
			depth_xyz.getAsVector() *= (1.0f / 1000.0f); // convert milimeters to meters
			inputData.images().set("depth_xyz", std::move(depth_xyz));
		}
		else
		{
			inputData.images().reset("depth");
			inputData.images().reset("depth_xyz");
		}

		if ((*info)->colorStream.readFrame(&colorFrame) == openni::STATUS_OK)
		{
			ImageType image(colorFrame.getWidth(), colorFrame.getHeight(), 3,
					reinterpret_cast<const uint8_t*>(colorFrame.getData()));
			image.getAsVector() *= (1.0f / 255.0f); // OpenNI returns colors between 0 and 255, scale values to 0..1
			inputData.images().set("color", std::move(image));
		}
		else
		{
			inputData.images().reset("color");
		}

		(*info)->deviceObject->pushInput();
	}
	return true;
}

std::shared_ptr<OpenNIScaffold> OpenNIScaffold::getOrCreateSharedInstance()
{
	static auto creator = []()
	{
		return std::shared_ptr<OpenNIScaffold>(new OpenNIScaffold);
	};
	static SurgSim::Framework::SharedInstance<OpenNIScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

SurgSim::DataStructures::DataGroup OpenNIScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addImage("color");
	builder.addImage("depth");
	builder.addImage("depth_xyz");
	return builder.createData();
}


};  // namespace Devices
};  // namespace SurgSim
