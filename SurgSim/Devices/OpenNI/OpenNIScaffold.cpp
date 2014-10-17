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
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"


namespace SurgSim
{
namespace Device
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
	openni::Status status = openni::OpenNI::initialize();
	bool success = (status == openni::STATUS_OK);
	SURGSIM_LOG_IF(!success, m_logger, DEBUG) << "Initialize failed: " << openni::OpenNI::getExtendedError();
}

OpenNIScaffold::~OpenNIScaffold()
{
	openni::OpenNI::shutdown();
}

bool OpenNIScaffold::registerDevice(OpenNIDevice* device)
{
	bool success = true;

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

	if (success && !isRunning())
	{
		start();
	}

	return success;
}

bool OpenNIScaffold::doRegisterDevice(DeviceData* info)
{
	openni::Status status;

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);
	SURGSIM_LOG_INFO(m_logger) << "Found " << deviceList.getSize() << " devices";

	status = info->camera.open(openni::ANY_DEVICE);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Could not connect to a camera: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->depthStream.create(info->camera, openni::SENSOR_DEPTH);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Could not find depth stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->colorStream.create(info->camera, openni::SENSOR_COLOR);
	if (status != openni::STATUS_OK)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Could find color image stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->depthStream.start();
	if (status != openni::STATUS_OK || !info->depthStream.isValid())
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Unable to start depth stream: " << openni::OpenNI::getExtendedError();
		return false;
	}

	status = info->colorStream.start();
	if (status != openni::STATUS_OK || !info->colorStream.isValid())
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Unable to start color image stream: " << openni::OpenNI::getExtendedError();
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
	return true;
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
		bool newImages = false;
		SurgSim::DataStructures::DataGroup& inputData = (*info)->deviceObject->getInputData();

		if ((*info)->depthStream.readFrame(&depthFrame) == openni::STATUS_OK)
		{
			newImages = true;
			ImageType image(depthFrame.getWidth(), depthFrame.getHeight(), 1,
					reinterpret_cast<const unsigned short*>(depthFrame.getData()));
			image.getAsVector() *= (1.0f / 1000.0f); // OpenNI2 returns mm, convert to meters
			inputData.images().set("depth", std::move(image));
		}

		if ((*info)->colorStream.readFrame(&colorFrame) == openni::STATUS_OK)
		{
			newImages = true;
			ImageType image(colorFrame.getWidth(), colorFrame.getHeight(), 3,
					reinterpret_cast<const unsigned char*>(colorFrame.getData()));
			image.getAsVector() *= (1.0 / 256.0); // OpenNI returns colors between 0 and 255, scale values to 0..1
			inputData.images().set("color", std::move(image));
		}

		if (newImages)
		{
			(*info)->deviceObject->pushInput();
		}
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
	return builder.createData();
}


};  // namespace Device
};  // namespace SurgSim
