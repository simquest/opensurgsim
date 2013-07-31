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

#ifndef HID_WINDDK_XXX
#include <linux/input.h>

#include <sys/ioctl.h>
#else /* HID_WINDDK_XXX */
#undef  _WIN32_WINNT
#define _WIN32_WINNT 0x0501   // request Windows XP-compatible SDK APIs
#undef  WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN   // do not automatically include WinSock 1 and some other header files
#include <windows.h>

#include <setupapi.h>
extern "C" {  // sigh...
#include <hidsdi.h>
}
#endif /* HID_WINDDK_XXX */

#include <stdint.h>

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
#ifndef HID_WINDDK_XXX
	DeviceData(const std::string& path, RawMultiAxisDevice* device, std::unique_ptr<SystemInputDeviceHandle>&& handle,
			   const std::vector<int>& buttonCodeList) :
#else /* HID_WINDDK_XXX */
	DeviceData(const std::string& path, RawMultiAxisDevice* device, std::unique_ptr<SystemInputDeviceHandle>&& handle) :
#endif /* HID_WINDDK_XXX */
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
#ifndef HID_WINDDK_XXX
		for (size_t i = 0;  (i < buttonCodeList.size()) && (i < NUM_BUTTONS);  ++i)
		{
			buttonCodes[i] = buttonCodeList[i];
		}
		for (size_t i = buttonCodeList.size();  i < NUM_BUTTONS;  ++i)
		{
			buttonCodes[i] = -1;
		}
#endif /* not HID_WINDDK_XXX */
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
#ifndef HID_WINDDK_XXX
		buttonCodes(std::move(other.buttonCodes)),
#endif /* not HID_WINDDK_XXX */
		coordinateSystemRotation(std::move(other.coordinateSystemRotation)),
		positionScale(std::move(other.positionScale)),
		orientationScale(std::move(other.orientationScale)),
		useAxisDominance(std::move(other.useAxisDominance))
	{
	}

	~DeviceData()
	{
	}

	static const size_t NUM_BUTTONS = 4;

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

	static std::array<int, 6> initialAxisStates()
	{
		std::array<int, 6> zeros;
		zeros.fill(0);
		return zeros;
	}

	static std::array<bool, NUM_BUTTONS> initialButtonStates()
	{
		std::array<bool, NUM_BUTTONS> states;
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
	std::array<int, 6> axisStates;
	/// Persistent button states.
	std::array<bool, NUM_BUTTONS> buttonStates;
#ifndef HID_WINDDK_XXX
	/// Event library button code corresponding to each index.
	std::array<int, NUM_BUTTONS> buttonCodes;
#endif /* not HID_WINDDK_XXX */
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


RawMultiAxisScaffold::RawMultiAxisScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::createConsoleLogger("RawMultiAxis device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Shared scaffold created.";
}


RawMultiAxisScaffold::~RawMultiAxisScaffold()
{
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Shared scaffold destroyed.";
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
		else if (value == biggestValue)
		{
			return -1;  // a tie means no dominant axis
		}
	}
	return biggestAxis;
}

#ifdef HID_WINDDK_XXX
static inline int16_t signedShortData(unsigned char byte0, unsigned char byte1)
{
	return static_cast<int16_t>(static_cast<uint16_t>(byte0) | (static_cast<uint16_t>(byte1) << 8));
}
#endif /* HID_WINDDK_XXX */

bool RawMultiAxisScaffold::updateDevice(RawMultiAxisScaffold::DeviceData* info)
{
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();

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
#ifndef HID_WINDDK_XXX
	while (info->deviceHandle->hasDataToRead())
	{
		struct input_event event;
		size_t numRead;
		if (! info->deviceHandle->readBytes(&event, sizeof(event), &numRead))
		{
			int64_t error = getSystemErrorCode();
			if (error == ENODEV)
			{
				SURGSIM_LOG_SEVERE(m_logger) << "RawMultiAxis: read failed; device has been disconnected!  (ignoring)";
				return false;  // stop updating this device!
			}
			else if (error != EAGAIN)
			{
				SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: read failed with error " << error << ", " <<
					getSystemErrorText(error);
			}
		}
		else if (numRead != sizeof(event))
		{
			SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: reading produced " << numRead << " bytes (expected " <<
				sizeof(event) << ")";
		}
		else
		{
			if (event.type == EV_REL)
			{

				if (event.code >= REL_X && event.code < (REL_X+3))  // Assume that X, Y, Z are consecutive
				{
					info->axisStates[0 + (event.code - REL_X)] = event.value;
					didUpdate = true;
				}
				else if (event.code >= REL_RX && event.code < (REL_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					info->axisStates[3 + (event.code - REL_RX)] = event.value;
					didUpdate = true;
				}
			}
			else if (event.type == EV_ABS)
			{
				if (event.code >= ABS_X && event.code < (ABS_X+3))  // Assume that X, Y, Z are consecutive
				{
					info->axisStates[0 + (event.code - ABS_X)] = event.value;
					didUpdate = true;
				}
				else if (event.code >= ABS_RX && event.code < (ABS_RX+3))  // Assume that RX, RY, RZ are consecutive
				{
					info->axisStates[3 + (event.code - ABS_RX)] = event.value;
					didUpdate = true;
				}
			}
			else if (event.type == EV_KEY)
			{
				for (size_t i = 0;  i < DeviceData::NUM_BUTTONS;  ++i)
				{
					if (event.code == info->buttonCodes[i])
					{
						info->buttonStates[i] = (event.value != 0);
						didUpdate = true;
						break;
					}
				}
			}
		}
	}
#else /* HID_WINDDK_XXX */

	// We can't keep reading while data is available, because we don't know how to tell when data is available.
	// Both WaitForSingleObject() and WaitForMultipleObjects() always claim data is available for 3DConnexion device
	// file handles.  So we just do it once, blocking until we have data.
	//
	// We also can't unblock the read once we initiate it (closing the handle has no effect).

	{
		unsigned char deviceBuffer[7*128];
		size_t numRead;
		if (! info->deviceHandle->readBytes(&deviceBuffer, sizeof(deviceBuffer), &numRead))
		{
			int64_t error = getSystemErrorCode();
			SURGSIM_LOG_WARNING(m_logger) << "RawMultiAxis: read failed with error " << error << ", " <<
				getSystemErrorText(error);
		}
		else if ((numRead >= 7) && (deviceBuffer[0] == 0x01))       // Translation
		{
			info->axisStates[0] = signedShortData(deviceBuffer[1],  deviceBuffer[2]);
			info->axisStates[1] = signedShortData(deviceBuffer[3],  deviceBuffer[4]);
			info->axisStates[2] = signedShortData(deviceBuffer[5],  deviceBuffer[6]);
			didUpdate = true;

			if ((numRead >= 14) && (deviceBuffer[7] == 0x02))  // translation data may have rotation appended to it
			{
				info->axisStates[3] = signedShortData(deviceBuffer[8],  deviceBuffer[9]);
				info->axisStates[4] = signedShortData(deviceBuffer[10],  deviceBuffer[11]);
				info->axisStates[5] = signedShortData(deviceBuffer[12],  deviceBuffer[13]);
			}
		}
		else if ((numRead >= 7) && (deviceBuffer[0] == 0x02))  // Rotation
		{
			info->axisStates[3] = signedShortData(deviceBuffer[1],  deviceBuffer[2]);
			info->axisStates[4] = signedShortData(deviceBuffer[3],  deviceBuffer[4]);
			info->axisStates[5] = signedShortData(deviceBuffer[5],  deviceBuffer[6]);
			didUpdate = true;

			if ((numRead >= 14) && (deviceBuffer[7] == 0x01))  // rotation data may have translation appended to it
			{
				info->axisStates[0] = signedShortData(deviceBuffer[8],  deviceBuffer[9]);
				info->axisStates[1] = signedShortData(deviceBuffer[10],  deviceBuffer[11]);
				info->axisStates[2] = signedShortData(deviceBuffer[12],  deviceBuffer[13]);
			}
		}
		else if ((numRead >= 2) && (deviceBuffer[0] == 0x03))  // Buttons
		{
			size_t currentByte = 1;  // Byte 0 specifies the packet type; data starts at byte 1
			unsigned char currentBit = 0x01;
			for (size_t i = 0;  i < DeviceData::NUM_BUTTONS;  ++i)
			{
				info->buttonStates[i] = ((deviceBuffer[currentByte] & currentBit) != 0);
				if (currentBit < 0x80)
				{
					currentBit = currentBit << 1;
				}
				else
				{
					currentBit = 0x01;
					++currentByte;
					if (currentByte >= numRead)  // out of data?
					{
						break;
					}
				}
			}
			didUpdate = true;
		}
	}
#endif /* HID_WINDDK_XXX */

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
	inputData.poses().set("pose", pose);
	inputData.booleans().set("button1", info->buttonStates[0]);
	inputData.booleans().set("button2", info->buttonStates[1]);
	inputData.booleans().set("button3", info->buttonStates[2]);
	inputData.booleans().set("button4", info->buttonStates[3]);

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
	std::unique_ptr<SystemInputDeviceHandle> handle = SystemInputDeviceHandle::open(path);
	if (! handle)
	{
		int64_t error = getSystemErrorCode();
#ifndef HID_WINDDK_XXX
		// XXX FIXME TODO(advornik): does not belong here...
		if (error == ENOENT)
		{
			return std::move(handle);
		}
#endif /* HID_WINDDK_XXX */
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

#ifdef HID_WINDDK_XXX
	// Prepare to iterate over the attached HID devices
	GUID hidGuid;
	HidD_GetHidGuid(&hidGuid);
	HDEVINFO hidDeviceInfo = SetupDiGetClassDevs(&hidGuid, NULL, NULL,
		DIGCF_DEVICEINTERFACE | DIGCF_PRESENT | DIGCF_PROFILE);
	if (hidDeviceInfo == INVALID_HANDLE_VALUE)
	{
		DWORD error = GetLastError();
		SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: Failed to query attached HID devices;" <<
			" SetupDiGetClassDevs() failed with error " << error << ", " << getSystemErrorText(error);
		return false;
	}
#endif /* HID_WINDDK_XXX */

#ifndef HID_WINDDK_XXX
	for (int i = 0;  i < 32;  ++i)
	{
		char devicePath[128];
		snprintf(devicePath, sizeof(devicePath), "/dev/input/event%d", i);

		std::unique_ptr<SystemInputDeviceHandle> handle = openDevice(devicePath);
		if (! handle)
		{
			// message was already printed
			continue;
		}

		char reportedName[1024];
		if (ioctl(handle->get(), EVIOCGNAME(sizeof(reportedName)), reportedName) < 0)
		{
			int error = errno;
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGNAME): error " << error << ", " <<
				getSystemErrorText(error);
			snprintf(reportedName, sizeof(reportedName), "???");
		}
		else
		{
			reportedName[sizeof(reportedName)-1] = '\0';
		}
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Examining device " << devicePath << " (" << reportedName << ")";

		struct input_id reportedId;
		if (ioctl(handle->get(), EVIOCGID, &reportedId) < 0)
		{
			int error = errno;
			SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGID): error " << error << ", " <<
				getSystemErrorText(error);
			continue;
		}

		if (! deviceHasSixAxes(handle.get()))
		{
			continue;
		}

		handle.reset();

		if (registerIfUnused(devicePath, device, numUsedDevicesSeen))
		{
			return true;
		}
	}

#else /* HID_WINDDK_XXX */
	// Loop through the device list, looking for the devices we want
	int hidEnumerationIndex = 0;
	SP_DEVICE_INTERFACE_DATA deviceInterfaceData;
	deviceInterfaceData.cbSize = sizeof(deviceInterfaceData);
	SP_DEVICE_INTERFACE_DETAIL_DATA* deviceInterfaceDetail = 0;
	DWORD deviceInterfaceDetailSize = 0;

	int numMultiAxisDevices = 0;
	int numDevicesWithType  = 0;

	while (1)
	{
		// Get the next interface in the list.
		if (! SetupDiEnumDeviceInterfaces(hidDeviceInfo, NULL, &hidGuid, hidEnumerationIndex, &deviceInterfaceData))
		{
			DWORD error = GetLastError();
			if (error == ERROR_NO_MORE_ITEMS)
			{
				break;
			}
			else
			{
				SURGSIM_LOG_CRITICAL(m_logger) << "RawMultiAxis: Failed to query attached HID devices;" <<
					" SetupDiEnumDeviceInterfaces() failed with error " << error << ", " << getSystemErrorText(error);
				return false;
			}
		}

		// Increment the counter for the next pass.  Make sure not to use this value directly later in the loop!
		++hidEnumerationIndex;

		// Find out the required size.
		DWORD neededSize = 0;
		if (! SetupDiGetDeviceInterfaceDetail(hidDeviceInfo, &deviceInterfaceData, NULL, 0, &neededSize, NULL))
		{
			DWORD error = GetLastError();
			if (error != ERROR_INSUFFICIENT_BUFFER)
			{
				SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: Failed to get the required device detail size," <<
					" device will be ignored; error " << error << ", " << getSystemErrorText(error);
				continue;
			}
		}

		// Make sure we have enough memory.
		if (neededSize > deviceInterfaceDetailSize)
		{
			deviceInterfaceDetailSize = 2*neededSize;
			if (deviceInterfaceDetail)
				free(deviceInterfaceDetail);
			deviceInterfaceDetail = static_cast<SP_DEVICE_INTERFACE_DETAIL_DATA*>( malloc(deviceInterfaceDetailSize) );
		}

		// Get the device detail (which actually just means the path).
		deviceInterfaceDetail->cbSize = sizeof(*deviceInterfaceDetail);
		if (! SetupDiGetDeviceInterfaceDetail(hidDeviceInfo, &deviceInterfaceData,
			deviceInterfaceDetail, deviceInterfaceDetailSize, NULL, NULL))
		{
			DWORD error = GetLastError();
			SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: Failed to get the HID device detail," <<
					" device will be ignored; error " << error << ", " << getSystemErrorText(error);
			continue;
		}

		// Check if this is the device we wanted.
		// TODO(advornik): Implement device attribute scanning like we do on Linux.

		std::string devicePath(deviceInterfaceDetail->DevicePath);
		std::unique_ptr<SystemInputDeviceHandle> handle = openDevice(devicePath);
		if (! handle)
		{
			// message was already printed
			continue;
		}

		char reportedName[1024] = "???";  // TODO(advornik): Implement querying names?
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: Examining device " << devicePath << " (" << reportedName << ")";

		if (! deviceHasSixAxes(handle.get()))
		{
			continue;
		}

		handle.reset();

		if (registerIfUnused(devicePath, device, numUsedDevicesSeen))
		{
			return true;
		}
	}
#endif /* HID_WINDDK_XXX */

#ifdef HID_WINDDK_XXX
	// Free the detail buffer
	if (deviceInterfaceDetail)
	{
		free(deviceInterfaceDetail);
		deviceInterfaceDetail = 0;
	}
#endif /* HID_WINDDK_XXX */

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

#ifndef HID_WINDDK_XXX
	std::vector<int> buttons = getDeviceButtonsAndKeys(handle.get());
#endif /* not HID_WINDDK_XXX */

	// Construct the object, start its thread, then move it to the list.
	// The thread needs a device entry pointer, but the unique_ptr indirection means we have one to provide even
	// before we've put an entry in the active device array.
#ifndef HID_WINDDK_XXX
	std::unique_ptr<DeviceData> info(new DeviceData(path, device, std::move(handle), buttons));
#else /* HID_WINDDK_XXX */
	std::unique_ptr<DeviceData> info(new DeviceData(path, device, std::move(handle)));
#endif /* HID_WINDDK_XXX */

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
	thread.release();

	return true;
}

#ifndef HID_WINDDK_XXX
static bool deviceHasSixAbsoluteAxes(SystemInputDeviceHandle* deviceHandle,
				     std::shared_ptr<SurgSim::Framework::Logger>* logger)
{
	BitSetBuffer<ABS_CNT> buffer;
	if (ioctl(deviceHandle->get(), EVIOCGBIT(EV_ABS, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(*logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_ABS)): error " << error << ", " <<
			getSystemErrorText(error);
		return false;
	}

	if (! buffer.test(ABS_X) || ! buffer.test(ABS_Y) || ! buffer.test(ABS_Z) ||
		! buffer.test(ABS_RX) || ! buffer.test(ABS_RY) || ! buffer.test(ABS_RZ))
	{
		SURGSIM_LOG_DEBUG(*logger) << "RawMultiAxis: does not have the 6 absolute axes.";
		return false;
	}

	int numIgnoredAxes = 0;
	for (size_t i = 0;  i < ABS_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			if ((i != ABS_X) && (i != ABS_Y) && (i != ABS_Z) && (i != ABS_RX) && (i != ABS_RY) && (i != ABS_RZ))
			{
				++numIgnoredAxes;
			}
		}
	}


	if (numIgnoredAxes)
	{
		SURGSIM_LOG_INFO(*logger) << "RawMultiAxis: ignoring " << numIgnoredAxes << " additional absolute axes.";
	}

	return true;
}

static bool deviceHasSixRelativeAxes(SystemInputDeviceHandle* deviceHandle,
				     std::shared_ptr<SurgSim::Framework::Logger>* logger)
{
	BitSetBuffer<REL_CNT> buffer;
	if (ioctl(deviceHandle->get(), EVIOCGBIT(EV_REL, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(*logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_REL)): error " << error << ", " <<
			getSystemErrorText(error);
		return false;
	}

	if (! buffer.test(REL_X) || ! buffer.test(REL_Y) || ! buffer.test(REL_Z) ||
		! buffer.test(REL_RX) || ! buffer.test(REL_RY) || ! buffer.test(REL_RZ))
	{
		SURGSIM_LOG_DEBUG(*logger) << "RawMultiAxis: does not have the 6 relative axes.";
		return false;
	}

	int numIgnoredAxes = 0;
	for (size_t i = 0;  i < REL_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			if ((i != REL_X) && (i != REL_Y) && (i != REL_Z) && (i != REL_RX) && (i != REL_RY) && (i != REL_RZ))
			{
				++numIgnoredAxes;
			}
		}
	}
	if (numIgnoredAxes)
	{
		SURGSIM_LOG_INFO(*logger) << "RawMultiAxis: ignoring " << numIgnoredAxes << " additional relative axes.";
	}

	return true;
}

bool RawMultiAxisScaffold::deviceHasSixAxes(SystemInputDeviceHandle* deviceHandle)
{
  return deviceHasSixAbsoluteAxes(deviceHandle, &m_logger) || deviceHasSixRelativeAxes(deviceHandle, &m_logger);
}
#else /* HID_WINDDK_XXX */
bool RawMultiAxisScaffold::deviceHasSixAxes(SystemInputDeviceHandle* deviceHandle)
{
	//SURGSIM_ASSERT(0) << "unimpl!";//XXX

	int numIgnoredAxes = 0;//XXX
	if (numIgnoredAxes)
	{
		SURGSIM_LOG_INFO(m_logger) << "RawMultiAxis: ignoring " << numIgnoredAxes << " additional absolute axes.";
	}

	return true;
}
#endif /* HID_WINDDK_XXX */

#ifndef HID_WINDDK_XXX
std::vector<int> RawMultiAxisScaffold::getDeviceButtonsAndKeys(SystemInputDeviceHandle* deviceHandle)
{
	std::vector<int> result;
	BitSetBuffer<KEY_CNT> buffer;
	if (ioctl(deviceHandle->get(), EVIOCGBIT(EV_KEY, buffer.sizeBytes()), buffer.getPointer()) == -1)
	{
		int error = errno;
		SURGSIM_LOG_DEBUG(m_logger) << "RawMultiAxis: ioctl(EVIOCGBIT(EV_KEY)): error " << error << ", " <<
			getSystemErrorText(error);
		return result;
	}

	// Start listing buttons/keys from BTN_0; then go back and cover the earlier ones.
	for (int i = BTN_0;  i < KEY_CNT;  ++i)
	{
		if (buffer.test(i))
		{
			result.push_back(i);
		}
	}
	for (int i = 0;  i < BTN_0;  ++i)
	{
		if (buffer.test(i))
		{
			result.push_back(i);
		}
	}

	return result;
}
#endif /* not HID_WINDDK_XXX */

SurgSim::DataStructures::DataGroup RawMultiAxisScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addBoolean("button1");
	builder.addBoolean("button2");
	builder.addBoolean("button3");
	builder.addBoolean("button4");
	return builder.createData();
}

std::shared_ptr<RawMultiAxisScaffold> RawMultiAxisScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<RawMultiAxisScaffold> sharedInstance;
	return sharedInstance.get();
}

void RawMultiAxisScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel RawMultiAxisScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;


};  // namespace Device
};  // namespace SurgSim
