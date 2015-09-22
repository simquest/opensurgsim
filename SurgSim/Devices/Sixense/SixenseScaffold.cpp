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

#include "SurgSim/Devices/Sixense/SixenseScaffold.h"

#include <vector>
#include <list>
#include <memory>
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <sixense.h>

#include "SurgSim/Devices/Sixense/SixenseDevice.h"
#include "SurgSim/Devices/Sixense/SixenseThread.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector3f;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;


namespace SurgSim
{
namespace Devices
{


struct SixenseScaffold::DeviceData
{
public:
	/// Initialize the data.
	DeviceData(int baseIndex, int controllerIndex, SixenseDevice* device) :
		deviceBaseIndex(baseIndex),
		deviceControllerIndex(controllerIndex),
		deviceObject(device)
	{
	}

	/// The index of the Sixense base unit for this device.
	const int deviceBaseIndex;
	/// The index of the Sixense controller for this device.
	const int deviceControllerIndex;
	/// The corresponding device object.
	SixenseDevice* const deviceObject;

private:
	// prohibit copy construction and assignment
	DeviceData(const DeviceData&);
	DeviceData& operator=(const DeviceData&);
};

struct SixenseScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// Processing thread.
	std::unique_ptr<SixenseThread> thread;

	/// The list of known devices.
	std::list<std::unique_ptr<SixenseScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// prohibit copy construction and assignment
	StateData(const StateData&);
	StateData& operator=(const StateData&);
};


SixenseScaffold::SixenseScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Sixense/Hydra device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Sixense/Hydra: Shared scaffold created.";
}


SixenseScaffold::~SixenseScaffold()
{
	// The thread needs to be torn down while NOT holding the mutex, to avoid deadlock.
	if (m_state->thread)
	{
		destroyThread();
	}

	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Sixense/Hydra: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Sixense/Hydra: Shared scaffold destroyed.";
}


bool SixenseScaffold::registerDevice(SixenseDevice* device)
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->isApiInitialized)
		{
			if (! initializeSdk())
			{
				return false;
			}
		}
	}

	boost::chrono::steady_clock::time_point tick = boost::chrono::steady_clock::now();
	int numUsedDevicesSeen = 0;
	bool fatalError = false;

	bool deviceFound = findUnusedDeviceAndRegister(device, &numUsedDevicesSeen, &fatalError);
	if (! deviceFound && ! fatalError && (numUsedDevicesSeen == 0) && (m_startupDelayMilliseconds > 0))
	{
		// Unfortunately, right after sixenseInit() the library has not yet completed its device discovery!
		// That means that calls to sixenseIsBaseConnected(), sixenseIsControllerEnabled(), etc. will return
		// *false* even if the base/controller is actually present.
		//
		// One way to deal with that would be to dynamically handle any (or no) connected devices, the way the
		// example code does.  But we'd like to report missing devices to the calling code as soon as possible.
		//
		// Another is to simply sleep and retry a few times here.  Ugh.
		// --advornik 2012-08-03
		boost::chrono::steady_clock::time_point retryEnd =
			tick + boost::chrono::milliseconds(m_startupDelayMilliseconds);
		while (true)
		{
			tick += boost::chrono::milliseconds(m_startupRetryIntervalMilliseconds);
			boost::this_thread::sleep_until(tick);
			tick = boost::chrono::steady_clock::now();  // finding a device may take > 100ms, so fix up the time.

			deviceFound = findUnusedDeviceAndRegister(device, &numUsedDevicesSeen, &fatalError);
			if (deviceFound || fatalError || (numUsedDevicesSeen > 0) || (tick >= retryEnd))
			{
				break;
			}
		}
	}

	if (! deviceFound)
	{
		if (fatalError)
		{
			// error information was hopefully already logged
			SURGSIM_LOG_DEBUG(m_logger) << "Sixense/Hydra: Registering device failed due to earlier fatal error.";
		}
		else if (numUsedDevicesSeen > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Sixense/Hydra: Failed to find any unused controllers!";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Sixense/Hydra: Failed to find any devices." <<
				"  Are the base and controllers plugged in?";
		}
		return false;
	}

	return true;
}


bool SixenseScaffold::unregisterDevice(const SixenseDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (! found)
	{
		SURGSIM_LOG_WARNING(m_logger) << "Sixense/Hydra: Attempted to release a non-registered device.";
	}
	return found;
}

bool SixenseScaffold::runInputFrame()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
	{
		// We don't call it->deviceObject->pullOutput() because we don't use any output at all.
		if (updateDevice(**it))
		{
			(*it)->deviceObject->pushInput();
		}
	}

	return true;
}

bool SixenseScaffold::updateDevice(const SixenseScaffold::DeviceData& info)
{
	//const SurgSim::DataStructures::DataGroup& outputData = info.deviceObject->getOutputData();

	int status = sixenseSetActiveBase(info.deviceBaseIndex);
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Sixense/Hydra: Could not activate base unit #" << info.deviceBaseIndex <<
			" to read device '" << info.deviceObject->getName() << "'! (status = " << status << ")";
		return false;
	}

	sixenseControllerData data;
	status = sixenseGetNewestData(info.deviceControllerIndex, &data);
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Sixense/Hydra: Could not get data from controller #" <<
			info.deviceBaseIndex << "," << info.deviceControllerIndex << " for device '" <<
			info.deviceObject->getName() << "'! (status = " << status << ")";
		return false;
	}

	// Use Eigen::Map to make the raw API output values look like Eigen data types
	Eigen::Map<Vector3f> position(data.pos);
	Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::ColMajor>> orientation(&(data.rot_mat[0][0]));

	RigidTransform3d pose;
	pose.makeAffine();
	pose.linear() = orientation.cast<double>();
	pose.translation() = position.cast<double>() * 0.001;  // convert from millimeters to meters!

	// TODO(bert): this code should cache the access indices.
	SurgSim::DataStructures::DataGroup& inputData = info.deviceObject->getInputData();
	inputData.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	inputData.scalars().set("trigger", data.trigger);
	inputData.scalars().set("joystickX", data.joystick_x);
	inputData.scalars().set("joystickY", data.joystick_y);
	inputData.booleans().set("buttonTrigger", (data.trigger > 0));
	inputData.booleans().set("buttonBumper", (data.buttons & SIXENSE_BUTTON_BUMPER) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_1, (data.buttons & SIXENSE_BUTTON_1) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_2, (data.buttons & SIXENSE_BUTTON_2) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_3, (data.buttons & SIXENSE_BUTTON_3) != 0);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_4, (data.buttons & SIXENSE_BUTTON_4) != 0);
	inputData.booleans().set("buttonStart", (data.buttons & SIXENSE_BUTTON_START) != 0);
	inputData.booleans().set("buttonJoystick", (data.buttons & SIXENSE_BUTTON_JOYSTICK) != 0);

	return true;
}

bool SixenseScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	int status = sixenseInit();
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Sixense/Hydra: Could not initialize the Sixense library (status = " <<
			status << ")";
		return false;
	}

	m_state->isApiInitialized = true;
	return true;
}

bool SixenseScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	int status = sixenseExit();
	if (status != SIXENSE_SUCCESS)
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Sixense/Hydra: Could not shut down the Sixense library (status = " <<
			status << ")";
		return false;
	}

	m_state->isApiInitialized = false;
	return true;
}

bool SixenseScaffold::findUnusedDeviceAndRegister(SixenseDevice* device, int* numUsedDevicesSeen, bool* fatalError)
{
	*numUsedDevicesSeen = 0;
	*fatalError = false;

	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "Sixense/Hydra: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Sixense/Hydra: Tried to register a device when the same name is" <<
			" already present!";
		*fatalError = true;
		return false;
	}

	const int maxNumBases = sixenseGetMaxBases();

	for (int b = 0;  b < maxNumBases;  ++b)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Sixense/Hydra: scanning base #" << b << " (of total " << maxNumBases << ")";
		if (! sixenseIsBaseConnected(b))
		{
			continue;
		}

		int status = sixenseSetActiveBase(b);
		if (status != SIXENSE_SUCCESS)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Sixense/Hydra: Could not activate connected base #" << b <<
				" (status = " << status << ")";
			continue;
		}

		const int maxNumControllers = sixenseGetMaxControllers();
		for (int c = 0;  c < maxNumControllers;  ++c)
		{
			if (! sixenseIsControllerEnabled(c))
			{
				continue;
			}

			sixenseControllerData data;
			status = sixenseGetNewestData(c, &data);
			if (status != SIXENSE_SUCCESS)
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Sixense/Hydra: Could not get data from enabled controller #" <<
					b << "," << c << " (status = " << status << ")";
				continue;
			}

			SURGSIM_LOG_DEBUG(m_logger) << "Sixense/Hydra: scanning controller #" << b << "," << c <<
				" (of total " << maxNumControllers << ")";

			if (registerIfUnused(b, c, device, numUsedDevicesSeen))
			{
				return true;
			}
		}
	}

	return false;
}

bool SixenseScaffold::registerIfUnused(int baseIndex, int controllerIndex, SixenseDevice* device,
									  int* numUsedDevicesSeen)
{
	// Check existing devices.
	auto sameIndices = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[baseIndex, controllerIndex](const std::unique_ptr<DeviceData>& info)
			{ return ((info->deviceBaseIndex == baseIndex) && (info->deviceControllerIndex == controllerIndex)); });
	if (sameIndices != m_state->activeDeviceList.end())
	{
		// We found an existing device for this controller.
		++(*numUsedDevicesSeen);
		return false;
	}

	// The controller is not yet in use.

	if (! m_state->thread)
	{
		createThread();
	}

	// Construct the object, start its thread, then move it to the list.
	// Note that since Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers, storing a
	// list of unique_ptr results in nicer code than storing a list of DeviceData values directly.
	std::unique_ptr<DeviceData> info(new DeviceData(baseIndex, controllerIndex, device));
	m_state->activeDeviceList.emplace_back(std::move(info));

	return true;
}

bool SixenseScaffold::createThread()
{
	SURGSIM_ASSERT(! m_state->thread);

	std::unique_ptr<SixenseThread> thread(new SixenseThread(this));
	thread->start();
	m_state->thread = std::move(thread);

	return true;
}

bool SixenseScaffold::destroyThread()
{
	SURGSIM_ASSERT(m_state->thread);

	std::unique_ptr<SixenseThread> thread = std::move(m_state->thread);
	thread->stop();
	thread.release();

	return true;
}

SurgSim::DataStructures::DataGroup SixenseScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addScalar("trigger");
	builder.addScalar("joystickX");
	builder.addScalar("joystickY");
	builder.addBoolean("buttonTrigger");
	builder.addBoolean("buttonBumper");
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_2);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_3);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_4);
	builder.addBoolean("buttonStart");
	builder.addBoolean("buttonJoystick");
	return builder.createData();
}

std::shared_ptr<SixenseScaffold> SixenseScaffold::getOrCreateSharedInstance()
{
	// Using an explicit creation function gets around problems with accessing the private constructor.
	static auto creator =
		[]() { return std::shared_ptr<SixenseScaffold>(new SixenseScaffold); };  // NOLINT(readability/braces)
	static SurgSim::Framework::SharedInstance<SixenseScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

void SixenseScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel SixenseScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

int SixenseScaffold::m_startupDelayMilliseconds = 6000;
int SixenseScaffold::m_startupRetryIntervalMilliseconds = 100;



};  // namespace Devices
};  // namespace SurgSim
