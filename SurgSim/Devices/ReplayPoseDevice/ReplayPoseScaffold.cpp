// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/ReplayPoseDevice/ReplayPoseScaffold.h"

#include <Eigen/Core>

#include <fstream>

#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/ReplayPoseDevice/ReplayPoseDevice.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"

namespace
{
/// Missing istream "operator >>" for Eigen matrix type
/// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=622
template<typename Derived>
std::istream& operator >>(std::istream& s, Eigen::MatrixBase<Derived>& m)
{
	for (int i = 0; i < m.rows(); ++i)
	{
		for (int j = 0; j < m.cols(); j++)
		{
			s >> m(i, j);
		}
	}
	return s;
}
}

namespace SurgSim
{
namespace Devices
{

class ReplayPoseDevice;

ReplayPoseScaffold::ReplayPoseScaffold() :
	Framework::BasicThread("ReplayPoseScaffold"),
	m_logger(Framework::Logger::getLogger("Devices/ReplayPoseScaffold"))
{
	SURGSIM_LOG_DEBUG(m_logger) << "ReplayPose scaffold created.";
}

ReplayPoseScaffold::~ReplayPoseScaffold()
{
	if (m_device != nullptr)
	{
		unregisterDevice();
	}
	SURGSIM_LOG_DEBUG(m_logger) << "ReplayPose scaffold destroyed.";
}

std::shared_ptr<ReplayPoseScaffold> ReplayPoseScaffold::getOrCreateSharedInstance()
{
	static Framework::SharedInstance<ReplayPoseScaffold> sharedInstance;
	return sharedInstance.get();
}

bool ReplayPoseScaffold::doInitialize() { return true; }

bool ReplayPoseScaffold::doStartUp() { return true; }

bool ReplayPoseScaffold::doUpdate(double dt)
{
	SURGSIM_ASSERT(m_device != nullptr) << "DeviceData not properly allocated";

	return updateDevice(m_device.get());
}

struct ReplayPoseScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be managed by this scaffold
	explicit DeviceData(ReplayPoseDevice* device) : deviceObject(device), m_timestamp(0), m_index(0)
	{
		loadFile(deviceObject->getFileName());
		m_timer.start();
	}

	/// Device object managed by this scaffold.
	ReplayPoseDevice* const deviceObject;

	/// Timer to keep track of the time stamp and keep the replay in sync with the recording
	Framework::Timer m_timer;

	/// Time stamp for the device (which time stamp are we reproducing?)
	double m_timestamp;

	/// Series of poses through time loaded from the input file
	std::vector<std::pair<double, Math::RigidTransform3d>> m_motion;

	/// Index of the latest pose used
	size_t m_index;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;

	void loadFile(const std::string& fileName)
	{
		auto logger = Framework::Logger::getLogger("Devices/ReplayPoseScaffold");

		std::ifstream inputFile;

		inputFile.open(fileName, std::ios::in);

		if (!inputFile.is_open())
		{
			SURGSIM_LOG_WARNING(logger) << "Could not find or open the file " << fileName <<
				"; Replay will use Identity pose";
		}

		while (!inputFile.eof())
		{
			double timeStamp;
			Math::RigidTransform3d pose;
			inputFile >> timeStamp >> pose.matrix();
			m_motion.emplace_back(timeStamp, pose);
		}
		SURGSIM_LOG_INFO(logger) << "Loaded " << m_motion.size() << " timestamps";
		if (m_motion.size() >= 1)
		{
			SURGSIM_LOG_INFO(logger) << "The loaded timestamps cover a range of " <<
				m_motion[m_motion.size() - 1].first - m_motion[0].first << " second(s)";
		}
		SURGSIM_LOG_IF(m_motion.size() == 0, logger, WARNING) <<
			"No poses could be properly loaded, Identity pose will be used";

		inputFile.close();
	}
};

bool ReplayPoseScaffold::registerDevice(ReplayPoseDevice* device)
{
	SURGSIM_ASSERT(m_device == nullptr) << "Can't register two ReplayPoseDevice.";

	m_device.reset(new ReplayPoseScaffold::DeviceData(device));
	if (m_device == nullptr)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Failed to create a DeviceData";
		return false;
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Registered device " << device->getName();

	bool success = true;
	if (!isRunning())
	{
		std::shared_ptr<Framework::Barrier> barrier = std::make_shared<Framework::Barrier>(2);
		start(barrier);
		barrier->wait(true); // Wait for initialize
		barrier->wait(true); // Wait for startup
		success = isInitialized();
	}

	return success;
}

bool ReplayPoseScaffold::unregisterDevice()
{
	// #threadsafety After unregistering, another thread could be in the process of registering.
	if (isRunning())
	{
		stop();
	}

	m_device.reset();
	if (m_device == nullptr)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Unregistered device";
		return true;
	}
	SURGSIM_LOG_DEBUG(m_logger) << "There is no device to unregister.";

	return false;
}

bool ReplayPoseScaffold::updateDevice(ReplayPoseScaffold::DeviceData* info)
{
	DataStructures::DataGroup& inputData = m_device->deviceObject->getInputData();

	info->m_timer.markFrame();
	info->m_timestamp += info->m_timer.getLastFramePeriod();

	Math::RigidTransform3d pose = getPoseAtTimeStamp(info);
	inputData.poses().set(DataStructures::Names::POSE, pose);

	m_device->deviceObject->pushInput();

	return true;
}

Math::RigidTransform3d ReplayPoseScaffold::getPoseAtTimeStamp(ReplayPoseScaffold::DeviceData* info)
{
	if (info->m_motion.size() == 0)
	{
		return Math::RigidTransform3d::Identity();
	}

	if (info->m_motion.size() == 1 || (info->m_index == 0 && info->m_timestamp <= info->m_motion[0].first))
	{
		return info->m_motion[0].second;
	}

	while (info->m_motion.size() > info->m_index && info->m_timestamp > info->m_motion[info->m_index].first)
	{
		info->m_index++;
	}

	// Requesting a timestamp over the higher limit recorded in the file
	if (info->m_motion.size() <= info->m_index)
	{
		return info->m_motion[info->m_motion.size() - 1].second;
	}

	// Are we requesting exactly the timestamp we just indexed
	if (info->m_timestamp == info->m_motion[info->m_index].first)
	{
		return info->m_motion[info->m_index].second;
	}

	// Otherwise, we need to interpolate between the index and the previous index
	double range = info->m_motion[info->m_index].first - info->m_motion[info->m_index - 1].first;
	double time = (info->m_timestamp - info->m_motion[info->m_index - 1].first) / range;
	return Math::interpolate(info->m_motion[info->m_index - 1].second, info->m_motion[info->m_index].second, time);
}

DataStructures::DataGroup ReplayPoseScaffold::buildDeviceInputData()
{
	DataStructures::DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	return builder.createData();
}

};  // namespace Devices
};  // namespace SurgSim
