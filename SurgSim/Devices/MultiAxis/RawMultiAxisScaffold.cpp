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

#include "SurgSim/Devices/MultiAxis/RawMultiAxisScaffold.h"

#include <vector>
#include <list>
#include <array>
#include <memory>
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include "SurgSim/Devices/MultiAxis/RawMultiAxisDevice.h"
#include "SurgSim/Devices/MultiAxis/RawMultiAxisThread.h"
#include "SurgSim/Devices/MultiAxis/GetSystemError.h"
#include "SurgSim/Devices/MultiAxis/SystemInputDeviceHandle.h"
#include "SurgSim/Devices/MultiAxis/CreateInputDeviceHandle.h"
#include "SurgSim/Devices/MultiAxis/BitSetBuffer.h"
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

using SurgSim::Device::Internal::getSystemErrorCode;
using SurgSim::Device::Internal::getSystemErrorText;


namespace SurgSim
{
namespace Device
{


struct RawMultiAxisScaffold::DeviceData
{
public:
	/// Initialize the data.
	DeviceData(const std::string& path, RawMultiAxisDevice* device, std::unique_ptr<SystemInputDeviceHandle>&& handle) :
		devicePath(path),
		deviceObject(device),
		thread(),
		deviceHandle(std::move(handle)),
		axisStates(initialAxisStates()),
		buttonStates(initialButtonStates()),
		coordinateSystemRotation(defaultCoordinateSystemRotation()),
		positionScale(device->getPositionScale()),
		orientationScale(device->getOrientationScale()),
		useAxisDominance(device->isUsingAxisDominance())
	{
	}

	// Initialize by moving the data from another object.
	// Needed because Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers.
	DeviceData(DeviceData&& other) :
		devicePath(std::move(other.devicePath)),
		deviceObject(std::move(other.deviceObject)),
		thread(std::move(other.thread)),
		deviceHandle(std::move(other.deviceHandle)),
		axisStates(std::move(other.axisStates)),
		buttonStates(std::move(other.buttonStates)),
		coordinateSystemRotation(std::move(other.coordinateSystemRotation)),
		positionScale(std::move(other.positionScale)),
		orientationScale(std::move(other.orientationScale)),
		useAxisDominance(std::move(other.useAxisDominance))
	{
	}

	~DeviceData()
	{
	}

	// Returns the default coordinate system rotation matrix.
	static SurgSim::Math::Matrix33d defaultCoordinateSystemRotation()
	{
		SurgSim::Math::Matrix33d coordinateSystemRotation;
		// Make +Y point up (3DConnexion devices use +Z up)
		coordinateSystemRotation <<
			1, 0, 0,
			0, 0, -1,
			0, 1, 0;
		return coordinateSystemRotation;
	}

	static SystemInputDeviceHandle::AxisStates initialAxisStates()
	{
		SystemInputDeviceHandle::AxisStates states;
		states.fill(0);
		return states;
	}

	static SystemInputDeviceHandle::ButtonStates initialButtonStates()
	{
		SystemInputDeviceHandle::ButtonStates states;
		states.fill(false);
		return states;
	}

	/// The system device path corresponding to this device.
	const std::string devicePath;
	/// The corresponding device object.
	RawMultiAxisDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<RawMultiAxisThread> thread;
	/// Device handle to read from.
	std::unique_ptr<SystemInputDeviceHandle> deviceHandle;
	/// Persistent axis states.
	SystemInputDeviceHandle::AxisStates axisStates;
	/// Persistent button states.
	SystemInputDeviceHandle::ButtonStates buttonStates;
	/// The rotation of the coordinate system (used to reorient, e.g. point +Y up)
	SurgSim::Math::Matrix33d coordinateSystemRotation;
	/// Scale factor for the position axes.
	double positionScale;
	/// Scale factor for the orientation axes.
	double orientationScale;
	/// Controls whether dominance will be enabled.
	bool useAxisDominance;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex parametersMutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

struct RawMultiAxisScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// The list of known devices.
	std::list<std::unique_ptr<RawMultiAxisScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};


RawMultiAxisScaffold::RawMultiAxisScaffold() :
	m_logger(Framework::Logger::getLogger("Devices/RawMultiAxis")), m_state(new StateData)
{
	SURGSIM_LOG_DEBUG(m_logger) << "Shared scaffold created.";
}


RawMultiAxisScaffold::~RawMultiAxisScaffold()
{
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Destroying scaffold while devices are active!?!";
			for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
			{
				if ((*it)->thread)
				{
					destroyPerDeviceThread(it->get());
				}
			}
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Shared scaffold destroyed.";
}

std::shared_ptr<SurgSim::Framework::Logger> RawMultiAxisScaffold::getLogger() const
{
	return m_logger;
}


bool RawMultiAxisScaffold::registerDevice(RawMultiAxisDevice* device)
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

	int numUsedDevicesSeen = 0;
	bool deviceFound = findUnusedDeviceAndRegister(device, &numUsedDevicesSeen);
	if (! deviceFound)
	{
		if (numUsedDevicesSeen > 0)
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Failed to find any unused multi-axis controllers," <<
				" out of " << numUsedDevicesSeen << " present!";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Failed to find any multi-axis controllers." <<
				"  Is one plugged in?  Are the permissions correct?";
		}
		return false;
	}

	return true;
}


bool RawMultiAxisScaffold::unregisterDevice(const RawMultiAxisDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			if ((*matching)->thread)
			{
				destroyPerDeviceThread(matching->get());
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (! found)
	{
		SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: Attempted to release a non-registered device.";
	}
	return found;
}

void RawMultiAxisScaffold::setPositionScale(const RawMultiAxisDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->positionScale = scale;
	}
}

void RawMultiAxisScaffold::setOrientationScale(const RawMultiAxisDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->orientationScale = scale;
	}
}

void RawMultiAxisScaffold::setAxisDominance(const RawMultiAxisDevice* device, bool onOff)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->useAxisDominance = onOff;
	}
}

bool RawMultiAxisScaffold::runInputFrame(RawMultiAxisScaffold::DeviceData* info)
{
	info->deviceObject->pullOutput();
	if (! updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();
	return true;
}

bool RawMultiAxisScaffold::runAfterLastFrame(RawMultiAxisScaffold::DeviceData* info)
{
	info->deviceHandle->prepareForShutdown();
	return true;
}

static int findDominantAxis(const std::array<int, 6>& axes)
{
	int biggestAxis = 0;
	int biggestValue = abs(axes[biggestAxis]);
	for (int i = 1;  i < 6;  ++i)
	{
		int value = abs(axes[i]);
		if (value > biggestValue)
		{
			biggestValue = value;
			biggestAxis = i;
		}
	}
	// Note that if a tie occurs, the 1st entry with the biggest absolute value wins !
	return biggestAxis;
}

bool RawMultiAxisScaffold::updateDevice(RawMultiAxisScaffold::DeviceData* info)
{
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();

	boost::lock_guard<boost::mutex> lock(info->parametersMutex);

	bool ledState = false;
	if (outputData.booleans().get("led1", &ledState))
	{
		static bool firstTimeWarning = true;
		if (firstTimeWarning)
		{
			firstTimeWarning = false;
			SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: controlling LEDs is not supported yet!";
			// TODO(advornik): We should implement LED control.  But that probably doesn't mix well with blocking
			// reads here, and would either need to be done in another thread, or we'd need to poll here.
		}
	}

	bool didUpdate = false;
	if (! info->deviceHandle->updateStates(&(info->axisStates), &(info->buttonStates), &didUpdate))
	{
		// If updateStates returns false, the device is no longer usable, so we exit and stop its update loop.
		return false;
	}

	if (didUpdate)
	{
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: STATE  " <<
			std::setw(3) << info->axisStates[0] << " " <<   std::setw(3) << info->axisStates[1] << " " <<
			std::setw(3) << info->axisStates[2] << " / " << std::setw(3) << info->axisStates[3] << " " <<
			std::setw(3) << info->axisStates[4] << " " <<   std::setw(3) << info->axisStates[5] << " :" <<
			(info->buttonStates[0] ? " X" : " _") << (info->buttonStates[1] ? " X" : " _") <<
			(info->buttonStates[2] ? " X" : " _") << (info->buttonStates[3] ? " X" : " _");
	}

	// Deal with dominance and scaling.
	// It would be neat to put dominance into a filter, outside of the raw multi-axis device... but it has to
	// happen before filtering, and putting dominance AND filtering (and integrator) into components is too much.
	Vector3d position;
	Vector3d rotation;
	if (! info->useAxisDominance)
	{
		position = Vector3d(info->axisStates[0], info->axisStates[1], info->axisStates[2]) * info->positionScale;
		rotation = Vector3d(info->axisStates[3], info->axisStates[4], info->axisStates[5]) * info->orientationScale;
	}
	else
	{
		position.setZero();
		rotation.setZero();
		int index = findDominantAxis(info->axisStates);
		if (index >= 0 && index < 3)
		{
			position[index] = info->axisStates[index] * info->positionScale;
		}
		else if (index >= 3 && index < 6)
		{
			rotation[index-3] = info->axisStates[index] * info->orientationScale;
		}
	}

	// Fix up the coordinate system (3DConnexion devices use +Z up coordinates, we want +Y up).
	position = info->coordinateSystemRotation * position;
	rotation = info->coordinateSystemRotation * rotation;

	// Convert to a pose.
	Matrix33d orientation;
	double angle = rotation.norm();
	if (angle < 1e-9)
	{
		orientation.setIdentity();
	}
	else
	{
		orientation = SurgSim::Math::makeRotationMatrix(angle, Vector3d(rotation / angle));
	}

	RigidTransform3d pose;
	pose.makeAffine();
	pose.linear() = orientation;
	pose.translation() = position;

	// TODO(bert): this code should cache the access indices.
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	inputData.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_1, info->buttonStates[0]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_2, info->buttonStates[1]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_3, info->buttonStates[2]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_4, info->buttonStates[3]);

	return true;
}

bool RawMultiAxisScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = true;
	return true;
}

bool RawMultiAxisScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = false;
	return true;
}

std::unique_ptr<SystemInputDeviceHandle> RawMultiAxisScaffold::openDevice(const std::string& path)
{
	std::unique_ptr<SystemInputDeviceHandle> handle = createInputDeviceHandle(path, m_logger);
	if (! handle)
	{
		int64_t error = getSystemErrorCode();
		SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: Could not open device " << path << ": error " << error <<
			", " << getSystemErrorText(error);
	}
	return std::move(handle);
}

bool RawMultiAxisScaffold::findUnusedDeviceAndRegister(RawMultiAxisDevice* device, int* numUsedDevicesSeen)
{
	*numUsedDevicesSeen = 0;

	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "RawMultiAxis: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: Tried to register a device when the same name is" <<
			" already present!";
		return false;
	}

	const std::vector<std::string> devicePaths = enumerateInputDevicePaths(m_logger.get());

	for (auto it = devicePaths.cbegin();  it != devicePaths.cend();  ++it)
	{
		const std::string& devicePath = *it;

		// Check if this is the device we wanted.

		std::unique_ptr<SystemInputDeviceHandle> handle = openDevice(devicePath);
		if (! handle)
		{
			// message was already printed
			continue;
		}

		const std::string reportedName = handle->getDeviceName();

		int vendorId, productId;
		if (handle->getDeviceIds(&vendorId, &productId))
		{
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Examining device " << devicePath << " (" <<
				std::hex << std::setfill('0') << std::setw(4) << vendorId << ":" << std::setw(4) << productId << " " <<
				reportedName << ")";
		}
		else
		{
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Examining device " << devicePath << " (????:???? " <<
				reportedName << ")";
		}

		if (! handle->hasTranslationAndRotationAxes())
		{
			continue;
		}

		handle.reset();

		if (registerIfUnused(devicePath, device, numUsedDevicesSeen))
		{
			return true;
		}
	}

	return false;
}

bool RawMultiAxisScaffold::registerIfUnused(const std::string& path, RawMultiAxisDevice* device,
											int* numUsedDevicesSeen)
{
	// Check existing devices.
	auto sameIndices = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[path](const std::unique_ptr<DeviceData>& info) { return (info->devicePath == path); });
	if (sameIndices != m_state->activeDeviceList.end())
	{
		// We found an existing device for this controller.
		++(*numUsedDevicesSeen);
		return false;
	}

	// The controller is not yet in use.

	std::unique_ptr<SystemInputDeviceHandle> handle = openDevice(path);
	if (! handle)
	{
		return false;
	}

	// Construct the object, start its thread, then move it to the list.
	// The thread needs a device entry pointer, but the unique_ptr indirection means we have one to provide even
	// before we've put an entry in the active device array.
	std::unique_ptr<DeviceData> info(new DeviceData(path, device, std::move(handle)));

	createPerDeviceThread(info.get());
	SURGSIM_ASSERT(info->thread);
	m_state->activeDeviceList.emplace_back(std::move(info));

	return true;
}

bool RawMultiAxisScaffold::createPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(! data->thread);

	std::unique_ptr<RawMultiAxisThread> thread(new RawMultiAxisThread(this, data));
	thread->start();
	data->thread = std::move(thread);

	return true;
}

bool RawMultiAxisScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread);

	std::unique_ptr<RawMultiAxisThread> thread = std::move(data->thread);
	thread->stop();
	thread.reset();

	return true;
}

SurgSim::DataStructures::DataGroup RawMultiAxisScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_2);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_3);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_4);
	return builder.createData();
}

std::shared_ptr<RawMultiAxisScaffold> RawMultiAxisScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<RawMultiAxisScaffold> sharedInstance;
	return sharedInstance.get();
}


};  // namespace Device
};  // namespace SurgSim
