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


#include "SurgSim/Devices/Novint/NovintScaffold.h"

#include <algorithm>
#include <array>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>
#include <hdl/hdl.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Clock.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;


namespace
{

/// Convert a HDLError to text.  The text was cut+pasted from the comments in Novint's hdlErrors.h file.
std::string convertErrorCodeToString(HDLError errorCode)
{
	switch (errorCode)
	{
	case HDL_ERROR_STACK_OVERFLOW:
		return "Overflow of error stack";
	case HDL_ERROR_INTERNAL:
		return "HDAL internal error";
	case HDL_ERROR_INIT_FAILED:
		return "Device initialization error";
	case HDL_INIT_INI_NOT_FOUND:
		return "Could not find configuration file";
	case HDL_INIT_INI_DLL_STRING_NOT_FOUND:
		return "No DLL name in configuration file";
	case HDL_INIT_INI_MANUFACTURER_NAME_STRING_NOT_FOUND:
		return "No MANUFACTURER_NAME value in configuration file";
	case HDL_INIT_DLL_LOAD_ERROR:
		return "Could not load driver DLL";
	case HDL_INIT_DEVICE_FAILURE:
		return "Failed to initialize device";
	case HDL_INIT_DEVICE_ALREADY_INITED:
		return "Device already initialized";
	case HDL_INIT_DEVICE_NOT_CONNECTED:
		return "Requested device not connected";
	case HDL_SERVO_START_ERROR:
		return "Could not start servo thread";
	default:
		return "<unknown>";
	}
}

/// Check for HDAL errors, display them, and signal fatal errors.
/// Exactly equivalent to <code>checkForFatalError(false, message)</code>.
/// \param message An additional descriptive message.
/// \return true if there was a fatal error; false if everything is OK.
bool checkForFatalError(const char* message)
{
	HDLError errorCode = hdlGetError();
	if (errorCode == HDL_NO_ERROR)
	{
		return false;
	}

	// The HDAL maintains an error stack, so in theory there could be more than one error pending.
	// We do head recursion to get them all in the correct order, and hope we don't overrun the stack...
	bool anotherFatalError = checkForFatalError(message);

	bool isFatal = (errorCode != HDL_ERROR_STACK_OVERFLOW);

	SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getLogger("Devices/Novint")) << message << std::endl <<
		"  Error text: '" << convertErrorCodeToString(errorCode) << "'" << std::endl <<
		"  Error code: 0x" << std::hex << std::setw(4) << std::setfill('0') << errorCode << std::endl;

	return (isFatal || anotherFatalError);
}

/// Check for HDAL errors, display them, and signal fatal errors.
/// Exactly equivalent to <code>checkForFatalError(message) || previousError</code>, but less nasty to read.
/// \param previousError	True if a previous error has occurred.
/// \param message	An additional descriptive message.
/// \return	true if there was a fatal error or if previousError is true; false if everything is OK.
bool checkForFatalError(bool previousError, const char* message)
{
	bool newError = checkForFatalError(message);
	return previousError || newError;
}
}

namespace SurgSim
{
namespace Devices
{

class NovintScaffold::Handle
{
public:
	explicit Handle(const std::string& serial) :
		m_deviceHandle(HDL_INVALID_HANDLE)
	{
		HDLDeviceHandle deviceHandle = HDL_INVALID_HANDLE;
		deviceHandle = hdlInitDeviceBySerialNumber(serial.c_str());

		if (checkForFatalError("Failed to initialize"))
		{
			SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices/Novint")) << std::endl <<
				"  HDAL serial number: '" << serial << "'" << std::endl;
		}
		else if (deviceHandle == HDL_INVALID_HANDLE)
		{
			SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Devices/Novint")) <<
				"No error during initializing device with serial number '" << serial << "'" <<
				", but an invalid handle returned.";
		}
		else
		{
			m_deviceHandle = deviceHandle;
			SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Novint")) <<
				"Handle " << deviceHandle << " created. hdlInitDeviceBySerialNumber called. Serial # " << serial;
		}
	}

	~Handle()
	{
		if (isValid())
		{
			checkForFatalError("Error prior to calling hdlUninitDevice");
			SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Novint")) <<
				"Handle " << m_deviceHandle << " destructing, calling hdlUninitDevice.";
			hdlUninitDevice(m_deviceHandle);
			m_deviceHandle = HDL_INVALID_HANDLE;
			checkForFatalError("Error calling hdlUninitDevice");
			SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Novint")) <<
				"Handle " << m_deviceHandle << " destructed, hdlUninitDevice called.";
		}
	}

	bool isValid() const
	{
		return (m_deviceHandle != HDL_INVALID_HANDLE);
	}

	HDLDeviceHandle get() const
	{
		SURGSIM_ASSERT(isValid()) << "HDL device handle is not valid.";
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The HDAL device handle (or HDL_INVALID_HANDLE if not valid).
	HDLDeviceHandle m_deviceHandle;
};


class NovintScaffold::Callback
{
public:
	Callback(NovintScaffold* scaffold) : m_callbackHandle(HDL_INVALID_HANDLE)
	{
		SURGSIM_ASSERT(scaffold != nullptr) << "Callback::create needs non-nullptr scaffold.";
		m_callbackHandle = hdlCreateServoOp(run, scaffold, false);
		if (!checkForFatalError("Failed to create servoOp callback"))
		{
			if (m_callbackHandle == HDL_INVALID_HANDLE)
			{
				SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Devices/Novint")) <<
					"Servo operation created, but invalid servo operation entry handle returned." << std::endl <<
					"  Error details: unknown (HDAL returned an invalid servo operation entry handle)";
			}
			else
			{
				SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Novint")) <<
					"Callback created, hdlCreateServoOp called.";
			}
		}
	}

	~Callback()
	{
		if (isValid())
		{
			checkForFatalError("Error prior to stopping haptic callback");
			SURGSIM_LOG_DEBUG(Framework::Logger::getLogger("Devices/Novint")) <<
				"Callback destructing, calling hdlDestroyServoOp...";
			hdlDestroyServoOp(m_callbackHandle);
			SURGSIM_LOG_IF(!checkForFatalError("Error stopping haptic callback"),
						   Framework::Logger::getLogger("Devices/Novint"), DEBUG) <<
								"Callback destructing, hdlDestroyServoOp called.";
			m_callbackHandle = HDL_INVALID_HANDLE;
		}
	}

	bool isValid() const
	{
		return (m_callbackHandle != HDL_INVALID_HANDLE);
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Callback(const Callback&) /*= delete*/;
	Callback& operator=(const Callback&) /*= delete*/;

	/// The callback wrapper passed to HDAL.
	/// \param [in,out] data	The user data (in our case, the scaffold pointer).
	/// \return	HD_CALLBACK_CONTINUE to wait for the next frame, or HD_CALLBACK_DONE to terminate further calls.
	static HDLServoOpExitCode run(void* data);

	/// The haptic loop callback handle (or HDL_INVALID_HANDLE if not valid).
	HDLOpHandle m_callbackHandle;
};

HDLServoOpExitCode NovintScaffold::Callback::run(void* data)
{
	static_cast<NovintScaffold*>(data)->runHapticFrame();
	return HDL_SERVOOP_CONTINUE;
}


struct NovintScaffold::DeviceData
{
	/// Initialize the state.
	explicit DeviceData(NovintCommonDevice* device) :
		initializationName(""),
		serialNumber(""),
		deviceObject(device),
		isPositionHomed(false),
		isOrientationHomed(false),
		isDeviceHomed(false),
		isDeviceHeld(false),
		isDevice7Dof(device->is7DofDevice()),
		isDeviceRollAxisReversed(false),
		eulerAngleOffsetRoll(0.0),
		eulerAngleOffsetYaw(0.0),
		eulerAngleOffsetPitch(0.0),
		forwardPointingPoseThreshold(0.9),
		torqueScale(Vector3d::Constant(1.0)),
		positionScale(device->getPositionScale()),
		orientationScale(device->getOrientationScale()),
		jointAngles(Vector3d::Zero()),
		force(Vector3d::Zero()),
		torque(Vector4d::Zero()),
		scaledPose(RigidTransform3d::Identity())
	{
		buttonStates.fill(false);
	}

	/// The maximum number of buttons supported by any device object.
	static const size_t MAX_NUM_BUTTONS = 4;

	/// Type used to store button states.
	typedef std::array<bool, MAX_NUM_BUTTONS> ButtonStates;

	/// The HDAL device name.
	std::string initializationName;
	/// The HDAL device serial number.
	std::string serialNumber;
	/// The corresponding device object.
	NovintCommonDevice* const deviceObject;

	/// The device handle wrapper.
	std::shared_ptr<NovintScaffold::Handle> deviceHandle;

	/// The joint angles for the device orientation.
	Vector3d jointAngles;
	/// The button state read from the device.
	ButtonStates buttonStates;
	/// The homing state read from the device.
	bool isPositionHomed;
	/// The homing state read from the device.
	bool isOrientationHomed;
	/// The homing state read from the device.
	bool isDeviceHomed;
	/// The proximity state read from the device.
	bool isDeviceHeld;
	/// True if this is a 7DoF device.
	bool isDevice7Dof;
	/// True if the roll axis of a 7DoF device has reverse polarity because the device is left-handed.
	bool isDeviceRollAxisReversed;

	/// The offset added to the roll Euler angle.
	double eulerAngleOffsetRoll;
	/// The offset added to the yaw Euler angle.
	double eulerAngleOffsetYaw;
	/// The offset added to the pitch Euler angle.
	double eulerAngleOffsetPitch;
	/// The threshold to determine if the device is pointing forwards before unlocking orientation.
	double forwardPointingPoseThreshold;
	/// The scaling factors for the torque axes.
	Vector3d torqueScale;

	/// The pose value from the device, after scaling.
	RigidTransform3d scaledPose;

	/// The force value to be written to the device.
	Vector3d force;
	/// The torque value to be written to the device.
	Vector4d torque;

	/// Scale factor for the position axes.
	double positionScale;
	/// Scale factor for the orientation axes.
	double orientationScale;
	/// The mutex that protects the externally modifiable parameters.
	boost::mutex parametersMutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};


struct NovintScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// Wrapper for the haptic loop callback handle.
	std::unique_ptr<Callback> callback;

	/// The registered devices.
	std::list<std::unique_ptr<DeviceData>> registeredDevices;

	/// The map from serial number to Handle for all devices that were available when the SDK was initialized.
	std::unordered_map<std::string, std::shared_ptr<Handle>> serialToHandle;

	/// List of devices that have been unregistered and should have their forces, torques, and gravity compensation
	/// zeroed in the next update.
	std::list<std::shared_ptr<Handle>> unregisteredHandles;

	/// The map from name to serial number for all devices.
	std::map<std::string, std::string> nameToSerial;

	/// The mutex that protects the list of registered devices.
	boost::mutex mutex;

	/// Time of the initialization of the latest handle.
	Framework::Clock::time_point initializationTime;

	/// Timer to measure update rate.
	Framework::Timer timer;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};



template <typename T>
static inline T clampToRange(T value, T rangeMin, T rangeMax)
{
	if (value < rangeMin)
		return rangeMin;
	if (value > rangeMax)
		return rangeMax;
	return value;
}



NovintScaffold::NovintScaffold() :
	m_logger(Framework::Logger::getLogger("Devices/Novint")), m_state(new StateData)
{
	{
		// Drain the HDAL error stack
		HDLError errorCode = hdlGetError();
		while (errorCode != HDL_NO_ERROR)
		{
			errorCode = hdlGetError();
		}
	}
	m_state->timer.setMaxNumberOfFrames(5000);

	// The canonical HDAL approach (Programmer's Guide, section 4.7 Multiple devices) is:
	// 1) hdlInitX on all devices that will be used by this application,
	// 2) hdlStart (must be after all hdlInitX and before hdlCreateServoOp), then
	// 3) hdlCreateServoOp (starts the callback).

	createAllHandles();
	hdlStart();
	if (!checkForFatalError("Couldn't start HDAL scheduler"))
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Scheduler started, hdlStart called.";

		std::unique_ptr<Callback> callback(new Callback(this));
		if (callback->isValid())
		{
			m_state->callback = std::move(callback);
			m_state->nameToSerial = getNameMap();
			m_state->isApiInitialized = true;
			SURGSIM_LOG_DEBUG(m_logger) << "Callback scheduled; Scaffold created successfully.";
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to create a callback.";
			hdlStop();
			checkForFatalError("Couldn't stop HDAL scheduler");
		}
	}
}

NovintScaffold::~NovintScaffold()
{
	// The HDAL seems to do bad things (and the CRT complains) if we uninitialize the device too soon.
	const int MINIMUM_LIFETIME_MILLISECONDS = 500;
	Framework::Clock::time_point earliestEndTime =
		m_state->initializationTime + boost::chrono::milliseconds(MINIMUM_LIFETIME_MILLISECONDS);
	boost::this_thread::sleep_until(earliestEndTime);

	if (m_state->callback != nullptr)
	{
		m_state->callback = nullptr;
		SURGSIM_LOG_DEBUG(m_logger) << "Callback reset.";
	}

	SURGSIM_LOG_DEBUG(m_logger) << "Stopping HDAL scheduler...";
	hdlStop();
	SURGSIM_LOG_IF(!checkForFatalError("Couldn't stop HDAL scheduler"), m_logger, DEBUG) <<
		"HDAL scheduler stopped, hdlStop called.";

	boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
	if (!m_state->registeredDevices.empty())
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Destroying scaffold while there are still registered devices!?!";
		m_state->registeredDevices.clear();
	}

	destroyAllHandles();
	SURGSIM_LOG_DEBUG(m_logger) << "Scaffold destroyed.";
}

bool NovintScaffold::registerDevice(NovintCommonDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (!m_state->isApiInitialized)
	{
		SURGSIM_LOG_WARNING(m_logger) << "The HDAL did not initialize correctly so no NovintDevice can be registered.";
		return false;
	}

	// Make sure the serial number is unique.
	std::string serialNumber = "";
	if ((device->getSerialNumber(&serialNumber)) && (serialNumber != ""))
	{
		auto& sameSerialNumber = std::find_if(m_state->registeredDevices.cbegin(),
			m_state->registeredDevices.cend(),
			[&serialNumber](const std::unique_ptr<DeviceData>& info)
		{ return info->serialNumber == serialNumber; });
		if (sameSerialNumber != m_state->registeredDevices.end())
		{
			SURGSIM_LOG_CRITICAL(m_logger) << "Tried to register a device when the same serial number " <<
				serialNumber <<" is already present!";
			return false;
		}
	}

	// Make sure the initialization name is unique.
	std::string initializationName = "";
	if ((device->getInitializationName(&initializationName)) && (initializationName != ""))
	{
		auto& sameInitializationName = std::find_if(m_state->registeredDevices.cbegin(),
			m_state->registeredDevices.cend(),
			[&initializationName](const std::unique_ptr<DeviceData>& info)
		{ return info->initializationName == initializationName; });
		if (sameInitializationName != m_state->registeredDevices.end())
		{
			SURGSIM_LOG_CRITICAL(m_logger) << "Tried to register a device when the same initialization (HDAL) name " <<
				initializationName << " is already present!";
			return false;
		}
	}

	// Construct the object, start its thread, then move it to the list.
	// Note that since Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers, storing a
	// list of unique_ptr results in nicer code than storing a list of DeviceData values directly.
	std::unique_ptr<DeviceData> info(new DeviceData(device));
	info->serialNumber = serialNumber;
	info->initializationName = initializationName;

	if (!initializeDeviceState(info.get()))
	{
		return false;   // message already printed
	}
	m_state->registeredDevices.emplace_back(std::move(info));
	SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << " initialized.";

	return true;
}


bool NovintScaffold::unregisterDevice(const NovintCommonDevice* const device)
{
	bool result = false;
	std::unique_ptr<DeviceData> savedInfo;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto& matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
			[&device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->registeredDevices.end())
		{
			savedInfo = std::move(*matching);
			m_state->registeredDevices.erase(matching);
			m_state->unregisteredHandles.push_back(savedInfo->deviceHandle);
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << " finalized.";
			result = true;
			// the iterator is now invalid but that's OK
		}
	}
	SURGSIM_LOG_IF(!result, m_logger, SEVERE) << "Attempted to release a non-registered device.";
	return result;
}

std::shared_ptr<NovintScaffold::Handle>
	NovintScaffold::findHandleByInitializationName(const std::string& initializationName)
{
	std::shared_ptr<Handle> handle;
	if (initializationName == "")
	{
		// get the first available
		for (auto& it : m_state->serialToHandle)
		{
			auto& possibleHandle = it.second;
			auto& matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
				[&possibleHandle](const std::unique_ptr<DeviceData>& info)
			{
				return info->deviceHandle == possibleHandle;
			});
			if (matching == m_state->registeredDevices.end())
			{
				handle = possibleHandle;
				break;
			}
		}
		if (handle == nullptr)
		{
			SURGSIM_ASSERT(m_state->serialToHandle.size() == m_state->registeredDevices.size()) <<
				"Failed to find an un-registered device when the number of registered devices is not equal to" <<
				" the number of devices found at startup.";
			SURGSIM_LOG_SEVERE(m_logger) <<
				"Attempted to register a default device, but no more devices are available." <<
				" There were " << m_state->serialToHandle.size() << " devices available at program start.";
		}
	}
	else
	{
		if (m_state->nameToSerial.count(initializationName) > 0)
		{
			const std::string serial = m_state->nameToSerial[initializationName];
			if (m_state->serialToHandle.count(serial) > 0)
			{
				handle = m_state->serialToHandle[serial];
			}
			else
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Attempted to register a device named '" << initializationName <<
					"', which should map to serial number " << serial <<
					", but no device with that serial number is available.";
			}
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Attempted to register a device named '" << initializationName <<
				"', but that name does not map to a serial number.  Was the configuration file found?" <<
				" Does it contain the text of a YAML node (for the map from name to serial number)?  Is '" <<
				initializationName << "' a key in that map?";
		}
	}
	return handle;
}


bool NovintScaffold::initializeDeviceState(DeviceData* info)
{
	SURGSIM_ASSERT(info->deviceHandle == nullptr) << "The raw handle should be nullptr before initialization.";

	if (info->serialNumber == "")
	{
		info->deviceHandle = findHandleByInitializationName(info->initializationName);
	}
	else
	{
		if (m_state->serialToHandle.count(info->serialNumber) > 0)
		{
			info->deviceHandle = m_state->serialToHandle[info->serialNumber];
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Attempted to register a device by serial number for serial number " <<
				info->serialNumber << ", but no device with that serial number is available.";
		}
	}
	m_state->unregisteredHandles.remove(info->deviceHandle);

	bool result = info->deviceHandle != nullptr;

	if (result && info->isDevice7Dof)
	{
		hdlMakeCurrent(info->deviceHandle->get());
		checkForFatalError("Couldn't enable the handle");

		int gripStatus[2] = { 0, 0 };
		// OSG2 grips report their "handedness" in the LSB of the second raw status byte
		hdlGripGetAttributes (HDL_GRIP_STATUS, 2, gripStatus);
		if (checkForFatalError("Cannot get grip status"))
		{
			// HDL reported an error.  An error message was already logged.
			return false;
		}
		bool leftHanded = ((gripStatus[1] & 0x01) != 0);
		if (leftHanded)
		{
			SURGSIM_LOG_DEBUG(m_logger) << "'" << info->initializationName << "' is LEFT-handed.";
			info->isDeviceRollAxisReversed = true;
			info->eulerAngleOffsetRoll = 0;
			info->eulerAngleOffsetYaw = -75. * M_PI / 180.;
			info->eulerAngleOffsetPitch = -50. * M_PI / 180.;
		}
		else
		{
			SURGSIM_LOG_DEBUG(m_logger) << "'" << info->initializationName << "' is right-handed.";
			info->isDeviceRollAxisReversed = false;
			info->eulerAngleOffsetRoll = 0;
			info->eulerAngleOffsetYaw = +75. * M_PI / 180.;
			info->eulerAngleOffsetPitch = +50. * M_PI / 180.;
		}
	}
	return result;
}

bool NovintScaffold::updateDeviceOutput(DeviceData* info, bool pulledOutput)
{
	hdlMakeCurrent(info->deviceHandle->get());	// This device is now "current", and all hdlXxx calls apply to it.
	bool fatalError = checkForFatalError("hdlMakeCurrent()");

	info->force.setZero();
	info->torque.setZero();
	if (info->isDeviceHomed && pulledOutput)
	{
		bool desiredGravityCompensation = false;
		if (info->deviceObject->getOutputData().booleans().get("gravityCompensation", &desiredGravityCompensation))
		{
			setGravityCompensation(info, desiredGravityCompensation);
		}
		calculateForceAndTorque(info);
	}

	// Set the force command (in newtons).
	hdlGripSetAttributev(HDL_GRIP_FORCE, 0, info->force.data()); // 2nd arg is index; output force is always "vector #0"
	fatalError = checkForFatalError(fatalError, "hdlGripSetAttributev(HDL_GRIP_FORCE)");

	if (info->isDevice7Dof)
	{
		// Set the torque vector.  Also set the jaw squeeze torque (as 4th element of the array)-- though this is not
		// used anywhere at the moment.
		// The 2nd arg to this call is the count; we're setting 4 doubles.
		hdlGripSetAttributesd(HDL_GRIP_TORQUE, 4, info->torque.data());
		fatalError = checkForFatalError(fatalError, "hdlGripSetAttributesd(HDL_GRIP_TORQUE)");
	}
	return !fatalError;
}

bool NovintScaffold::updateDeviceInput(DeviceData* info)
{
	boost::lock_guard<boost::mutex> lock(info->parametersMutex);
	hdlMakeCurrent(info->deviceHandle->get());	// This device is now "current", and all hdlXxx calls apply to it.
	bool fatalError = checkForFatalError("hdlMakeCurrent()");

	info->buttonStates.fill(false);
	hdlGripGetAttributesb(HDL_GRIP_BUTTON, static_cast<int>(info->buttonStates.size()), info->buttonStates.data());
	fatalError = checkForFatalError(fatalError, "hdlGripGetAttributesb(HDL_GRIP_BUTTON)");

	checkDeviceHoming(info);
	if (info->isPositionHomed)
	{
		hdlGripGetAttributev(HDL_GRIP_POSITION, 0, info->scaledPose.translation().data());
		fatalError = checkForFatalError(fatalError, "hdlGripGetAttributev(HDL_GRIP_POSITION)");
		info->scaledPose.translation() *= info->positionScale;
	}

	// Get the additional 7DoF data if available.
	if (info->isDevice7Dof && info->isOrientationHomed)
	{
		// We compute the device orientation from the joint angles, for two reasons.  The first that it lets us
		// compensate for recurrent bugs in the HDAL grip code.  The second is that we'll need the joint angles in
		// order to correctly generate joint torques.
		double angles[4];
		hdlGripGetAttributesd(HDL_GRIP_ANGLE, 4, angles);
		fatalError = checkForFatalError(fatalError, "hdlGripGetAttributesd(HDL_GRIP_ANGLE)");

		// The zero values are NOT the home orientation.
		info->jointAngles[0] = angles[0] + info->eulerAngleOffsetRoll;
		info->jointAngles[1] = angles[1] + info->eulerAngleOffsetYaw;
		info->jointAngles[2] = angles[2] + info->eulerAngleOffsetPitch;

		// For the Falcon 7DoF grip, the axes are perpendicular and the joint angles are Euler angles:
		Matrix33d rotationX = makeRotationMatrix(info->jointAngles[0] * info->orientationScale,
			Vector3d(Vector3d::UnitX()));
		Matrix33d rotationY = makeRotationMatrix(info->jointAngles[1] * info->orientationScale,
			Vector3d(Vector3d::UnitY()));
		Matrix33d rotationZ = makeRotationMatrix(info->jointAngles[2] * info->orientationScale,
			Vector3d(Vector3d::UnitZ()));
		info->scaledPose.linear() = rotationY * rotationZ * rotationX;
	}

	setInputData(info);
	return !fatalError;
}

void NovintScaffold::checkDeviceHoming(DeviceData* info)
{
	unsigned int deviceStateBitmask = hdlGetState();
	info->isPositionHomed = ((deviceStateBitmask & HDAL_NOT_CALIBRATED) == 0);

	if (info->isDevice7Dof)
	{
		// The homing state is communicated using the button information.
		info->isOrientationHomed = info->buttonStates[0] && info->buttonStates[1];
		// So is the state of whether the device is currently held (proximity sensor).
		info->isDeviceHeld = info->buttonStates[2];
		// There are no ACTUAL buttons on the 7DoF Falcons, so we clear the button buffer.
		info->buttonStates.fill(false);
	}
	else
	{
		// The 3-DoF device doesn't need the orientation homing shenanigans...
		info->isOrientationHomed = true;
		info->isDeviceHomed = info->isPositionHomed;
		info->isDeviceHeld = true;  // ...I guess
	}

	if (info->isPositionHomed && info->isOrientationHomed && !info->isDeviceHomed)
	{
		// Wait until the tool is pointed forwards (i.e. perpendicular to the Falcon centerline) before proclaiming the
		// whole device homed.
		Vector3d forwardDirection = Vector3d::UnitX();
		double forwardMetric = forwardDirection.dot(info->scaledPose.linear() * forwardDirection);

		if (forwardMetric >= info->forwardPointingPoseThreshold)
		{
			// It looks like everything is ready!
			info->isDeviceHomed = true;
		}
	}
}

void NovintScaffold::calculateForceAndTorque(DeviceData* info)
{
	const DataGroup& outputData = info->deviceObject->getOutputData();
	outputData.vectors().get(DataStructures::Names::FORCE, &(info->force));

	// If the springJacobian was provided, multiply with the change in position since the output data was set,
	// to get a delta force.  This way a linearized output force is calculated at haptic update rates.
	Math::Vector6d deltaPosition;
	DataGroup::DynamicMatrixType springJacobian;
	bool havespringJacobian =
		outputData.matrices().get(DataStructures::Names::SPRING_JACOBIAN, &springJacobian);
	if (havespringJacobian)
	{
		RigidTransform3d poseForNominal = info->scaledPose;
		outputData.poses().get(DataStructures::Names::INPUT_POSE, &poseForNominal);

		Vector3d rotationVector = Vector3d::Zero();
		Math::computeRotationVector(info->scaledPose, poseForNominal, &rotationVector);

		Math::setSubVector(info->scaledPose.translation() - poseForNominal.translation(), 0, 3,
			&deltaPosition);
		Math::setSubVector(rotationVector, 1, 3, &deltaPosition);

		info->force += springJacobian.block<3,6>(0, 0) * deltaPosition;
	}

	// If the damperJacobian was provided, calculate a delta force based on the change in velocity.
	Math::Vector6d deltaVelocity;
	DataGroup::DynamicMatrixType damperJacobian;
	bool havedamperJacobian =
		outputData.matrices().get(DataStructures::Names::DAMPER_JACOBIAN, &damperJacobian);
	if (havedamperJacobian)
	{
		// TODO(ryanbeasley): consider adding a velocity filter setting to NovintDevice/DeviceData.
		Vector3d linearVelocity = Vector3d::Zero();
		Vector3d angularVelocity = Vector3d::Zero();

		Vector3d linearVelocityForNominal = linearVelocity;
		outputData.vectors().get(DataStructures::Names::INPUT_LINEAR_VELOCITY, &linearVelocityForNominal);
		Vector3d angularVelocityForNominal = angularVelocity;
		outputData.vectors().get(DataStructures::Names::INPUT_ANGULAR_VELOCITY, &angularVelocityForNominal);

		Math::setSubVector(linearVelocity - linearVelocityForNominal, 0, 3, &deltaVelocity);
		Math::setSubVector(angularVelocity - angularVelocityForNominal, 1, 3, &deltaVelocity);

		info->force += damperJacobian.block<3,6>(0, 0) * deltaVelocity;
	}

	// Calculate the torque command if applicable (and convert newton-meters to command counts).
	if (info->isDevice7Dof)
	{
		Vector3d torque = Vector3d::Zero();
		outputData.vectors().get(DataStructures::Names::TORQUE, &torque);

		if (havespringJacobian)
		{
			torque += springJacobian.block<3,6>(3, 0) * deltaPosition;
		}

		if (havedamperJacobian)
		{
			torque += damperJacobian.block<3,6>(3, 0) * deltaVelocity;
		}

		// We have the torque vector in newton-meters.  Sadly, what we need is the torque command counts FOR EACH MOTOR
		// AXIS, not for each Cartesian axis. Which means we need to go back to calculations with joint angles.
		// For the Falcon 7DoF grip, the axes are perpendicular and the joint angles are Euler angles:
		Matrix33d rotationX = makeRotationMatrix(info->jointAngles[0], Vector3d(Vector3d::UnitX()));
		Matrix33d rotationY = makeRotationMatrix(info->jointAngles[1], Vector3d(Vector3d::UnitY()));
		Matrix33d rotationZ = makeRotationMatrix(info->jointAngles[2], Vector3d(Vector3d::UnitZ()));
		// NB: the order of rotations is (rotY * rotZ * rotX), not XYZ!
		// Construct the joint axes for the CURRENT pose of the device.
		Vector3d jointAxisY = Vector3d::UnitY();
		Vector3d jointAxisZ = rotationY * Vector3d::UnitZ();
		Vector3d jointAxisX = rotationY * (rotationZ * Vector3d::UnitX());
		// To convert from Cartesian space to motor-axis space, we assemble the axes into a basis matrix and invert it.
		Matrix33d basisMatrix;
		basisMatrix.col(0) = jointAxisX;
		basisMatrix.col(1) = jointAxisY;
		basisMatrix.col(2) = jointAxisZ;
		double basisDeterminant = fabs(basisMatrix.determinant());

		// Also construct a "fake" X axis orthogonal with the other two, to be used when the pose is degenerate.
		// Note that the Y and Z axes are always perpendicular for the Falcon 7DoF, so the normalize() can't fail and
		// is basically unnecessary, but...
		Vector3d fakeAxisX  = jointAxisY.cross(jointAxisZ).normalized();
		Matrix33d fakeBasisMatrix;
		fakeBasisMatrix.col(0) = fakeAxisX;
		fakeBasisMatrix.col(1) = jointAxisY;
		fakeBasisMatrix.col(2) = jointAxisZ;

		const double mediumBasisDeterminantThreshold = 0.6;
		const double smallBasisDeterminantThreshold = 0.4;

		Matrix33d decompositionMatrix;
		if (basisDeterminant >= mediumBasisDeterminantThreshold)
		{
			// All is well!
			decompositionMatrix = basisMatrix.inverse();
		}
		else if (basisDeterminant >= smallBasisDeterminantThreshold)
		{
			// If the determinant is "medium" but not "small", the device is in a near-degenerate configuration.
			// Which axes are going to be commanded may be hugely dependent on small changes in the pose.
			// We want to gradually decrease the amount of roll torque produced near the degenerate point.
			double ratio =  ((basisDeterminant - smallBasisDeterminantThreshold) /
				(mediumBasisDeterminantThreshold - smallBasisDeterminantThreshold));
			// The computed ratio has to be 0 <= ratio < 1.  We just use linear drop-off.

			// The "fake" basis matrix replaces the X axis with a fake (so it's always invertible), but the output X
			// torque is then meaningless.
			Matrix33d fakeDecompositionMatrix = fakeBasisMatrix.inverse();
			fakeDecompositionMatrix.row(0) = Vector3d::Zero();

			decompositionMatrix = basisMatrix.inverse() * ratio + fakeDecompositionMatrix * (1.-ratio);
		}
		else
		{
			// If the determinant is small, the matrix may not be invertible.
			// The "fake" basis matrix replaces the X axis with a fake (so it's always invertible), but the output X
			// torque is then meaningless.
			decompositionMatrix = fakeBasisMatrix.inverse();
			decompositionMatrix.row(0) = Vector3d::Zero();
			// Moreover, near the degenerate position the X axis free-spins but is aligned with Y,
			// so we want to reduce Y torques as well.
			//double ratio = (basisDeterminant / smallBasisDeterminantThreshold);
			double ratio = 0;
			// The computed ratio has to be 0 <= ratio < 1.  We just use linear drop-off.
			decompositionMatrix.row(1) *= ratio;
		}
		Vector3d axisTorqueVector = decompositionMatrix * torque;

		// Unit conversion factors for the Falcon 7DoF.  THIS SHOULD BE PARAMETRIZED!
		const double axisTorqueMin = -2000;
		const double axisTorqueMax = +2000;
		// roll axis:  torque = 17.6 mNm  when command = 2000 (but flipped in left grip!)
		const double rollTorqueScale  = axisTorqueMax / 17.6e-3;
		// yaw axis:   torque = 47.96 mNm when command = 2000
		const double yawTorqueScale   = axisTorqueMax / 47.96e-3;
		// pitch axis: torque = 47.96 mNm when command = 2000
		const double pitchTorqueScale = axisTorqueMax / 47.96e-3;

		info->torque[0] = clampToRange(rollTorqueScale  * info->torqueScale.x() * axisTorqueVector.x(),
			axisTorqueMin, axisTorqueMax);
		info->torque[1] = clampToRange(yawTorqueScale   * info->torqueScale.y() * axisTorqueVector.y(),
			axisTorqueMin, axisTorqueMax);
		info->torque[2] = clampToRange(pitchTorqueScale * info->torqueScale.z() * axisTorqueVector.z(),
			axisTorqueMin, axisTorqueMax);
		info->torque[3] = 0;

		if (info->isDeviceRollAxisReversed)  // commence swearing.
		{
			info->torque[0] = -info->torque[0];
		}
	}
}

void NovintScaffold::setInputData(DeviceData* info)
{
	DataGroup& inputData = info->deviceObject->getInputData();
	inputData.poses().set(DataStructures::Names::POSE, info->scaledPose);
	inputData.booleans().set(DataStructures::Names::BUTTON_1, info->buttonStates[0]);
	inputData.booleans().set(DataStructures::Names::BUTTON_2, info->buttonStates[1]);
	inputData.booleans().set(DataStructures::Names::BUTTON_3, info->buttonStates[2]);
	inputData.booleans().set(DataStructures::Names::BUTTON_4, info->buttonStates[3]);
	inputData.booleans().set(DataStructures::Names::IS_HOMED, info->isDeviceHomed);
	inputData.booleans().set(DataStructures::Names::IS_POSITION_HOMED, info->isPositionHomed);
	inputData.booleans().set(DataStructures::Names::IS_ORIENTATION_HOMED, info->isOrientationHomed);
}

void NovintScaffold::createAllHandles()
{
	// The Scaffold does not know which devices will be initialized, so we use hdlCatalogDevices to get the
	// serial numbers for every connected device, then initialize all the connected devices, and when
	// registerDevice is called the name can be matched to a Handle created from a serial number.

	char serials[HDL_MAX_DEVICES * HDL_SERNUM_BUFFSIZE];
	const int numDevices = hdlCatalogDevices(HDL_NOT_OPEN_BY_ANY_APP, &(serials[0]), NULL);
	checkForFatalError("Failed to get catalog of devices.");

	SURGSIM_LOG_DEBUG(m_logger) << numDevices << " Novint devices available.";

	for (int i = 0; i < numDevices; ++i)
	{
		const std::string serial(&(serials[i * HDL_SERNUM_BUFFSIZE]), HDL_SERNUM_BUFFSIZE - 1);
		SURGSIM_LOG_DEBUG(m_logger) << "Found serial number " << serial << ".";

		auto handle = std::make_shared<NovintScaffold::Handle>(serial);
		if (handle->isValid())
		{
			m_state->serialToHandle[serial] = handle;
			hdlMakeCurrent(handle->get());
			checkForFatalError("Failed to make device current.");
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger) << "Failed to initialize Falcon with serial " << serial << ".";
		}
	}
	m_state->initializationTime = Framework::Clock::now();
	SURGSIM_LOG_DEBUG(m_logger) << "All device handles created.";
}

void NovintScaffold::destroyAllHandles()
{
	SURGSIM_LOG_DEBUG(m_logger) << "Destroying all Handles...";
	m_state->registeredDevices.clear();
	m_state->unregisteredHandles.clear();
	m_state->serialToHandle.clear();
	SURGSIM_LOG_DEBUG(m_logger) << "Handles destroyed.";
}

std::map<std::string, std::string> NovintScaffold::getNameMap()
{
	std::map<std::string, std::string> map;
	std::vector<std::string> paths;
	paths.push_back(".");
	Framework::ApplicationData applicationData(paths);
	std::string filePath;
	if (applicationData.tryFindFile("devices.yaml", &filePath))
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Found devices.yaml at '" << filePath << "'.";
		YAML::Node node = YAML::LoadFile(filePath);
		map = node["Novint"].as<std::map<std::string, std::string>>();
	}
	else
	{
		SURGSIM_LOG_DEBUG(m_logger) << "Failed to find devices.yaml, cannot map names to serial numbers.";
	}
	return map;
}

void NovintScaffold::runHapticFrame()
{
	m_state->timer.markFrame();
	if (m_state->timer.getCurrentNumberOfFrames() == m_state->timer.getMaxNumberOfFrames())
	{
		SURGSIM_LOG_INFO(m_logger) << std::setprecision(4)
			<< "Rate: " << m_state->timer.getAverageFrameRate() << "Hz "
			<< "(min individual frame " << 1.0 / m_state->timer.getMaxFramePeriod() << "Hz).";
		m_state->timer.setMaxNumberOfFrames(static_cast<size_t>(m_state->timer.getAverageFrameRate() * 5.0));
		m_state->timer.start();
	}
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto& it = m_state->registeredDevices.begin();  it != m_state->registeredDevices.end();  ++it)
	{
		if (updateDeviceInput((*it).get()))
		{
			(*it)->deviceObject->pushInput();
		}
	}
	for (auto& it = m_state->registeredDevices.begin();  it != m_state->registeredDevices.end();  ++it)
	{
		updateDeviceOutput(it->get(), (*it)->deviceObject->pullOutput());
	}

	bool desiredGravityCompensation = false;
	Vector3d force = Vector3d::Zero();
	Vector4d torque = Vector4d::Zero();
	for (auto& handle : m_state->unregisteredHandles)
	{
		if (handle->isValid())
		{
			hdlMakeCurrent(handle->get());

			hdlGripSetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &desiredGravityCompensation);
			checkForFatalError("Cannot set gravity compensation state on recently unregistered device.");

			hdlGripSetAttributev(HDL_GRIP_FORCE, 0, force.data());
			checkForFatalError("hdlGripSetAttributev(HDL_GRIP_FORCE)");

			hdlGripSetAttributesd(HDL_GRIP_TORQUE, 4, torque.data());
			checkForFatalError("hdlGripSetAttributesd(HDL_GRIP_TORQUE)");
		}
	}
	m_state->unregisteredHandles.clear();
}

bool NovintScaffold::getGravityCompensation(const NovintScaffold::DeviceData* info, bool* gravityCompensationState)
{
	bool state1 = true;
	hdlGripGetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &state1);
	if (checkForFatalError("Cannot get gravity compensation (#1)"))
	{
		return false;
	}

	bool state2 = false;
	hdlGripGetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &state2);
	if (checkForFatalError("Cannot get gravity compensation (#2)"))
	{
		return false;
	}

	if (state1 == true && state2 == false)
	{
		SURGSIM_LOG_WARNING(m_logger) << "getting gravity compensation state for '" << info->deviceObject->getName() <<
			"' does nothing!";
		return false;
	}
	else if (state1 != state2)
	{
		SURGSIM_LOG_WARNING(m_logger) << "getting gravity compensation state for '" << info->deviceObject->getName() <<
			"' keeps changing?!?";
		return false;
	}

	*gravityCompensationState = state1;
	return true;
}


bool NovintScaffold::enforceGravityCompensation(const NovintScaffold::DeviceData* info, bool gravityCompensationState)
{
	bool initialState;
	bool isInitialStateValid = getGravityCompensation(info, &initialState);

	const int maxAttempts = 20;
	for (int i = 0;  i < maxAttempts;  ++i)
	{
		bool state = gravityCompensationState;
		hdlGripSetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &state);
		if (checkForFatalError("Cannot set gravity compensation state"))
		{
			return false;
		}

		if (!getGravityCompensation(info, &state))
		{
			return false;
		}
		else if (state == gravityCompensationState)
		{
			// If the state has been changed, log a message.
			if (isInitialStateValid && (initialState != gravityCompensationState))
			{
				SURGSIM_LOG_INFO(m_logger) << "gravity compensation for '" << info->deviceObject->getName() <<
					"' changed to " << (gravityCompensationState? "enabled." : "disabled." );
			}
			return true;
		}
	}

	SURGSIM_LOG_WARNING(m_logger) << "failed to set gravity compensation for '" << info->deviceObject->getName() <<
		"' to " << (gravityCompensationState ? "enabled" : "disabled") << " after " << maxAttempts << " attempts";
	return false;
}


bool NovintScaffold::setGravityCompensation(const NovintScaffold::DeviceData* info, bool gravityCompensationState)
{
	bool initialState;
	bool isInitialStateValid = getGravityCompensation(info, &initialState);

	if (isInitialStateValid && (initialState == gravityCompensationState))
	{
		return true;  // no need to do anything
	}

	return enforceGravityCompensation(info, gravityCompensationState);
}

DataGroup NovintScaffold::buildDeviceInputData()
{
	DataStructures::DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	builder.addBoolean(DataStructures::Names::BUTTON_1);
	builder.addBoolean(DataStructures::Names::BUTTON_2);
	builder.addBoolean(DataStructures::Names::BUTTON_3);
	builder.addBoolean(DataStructures::Names::BUTTON_4);
	builder.addBoolean(DataStructures::Names::IS_HOMED);
	builder.addBoolean(DataStructures::Names::IS_POSITION_HOMED);
	builder.addBoolean(DataStructures::Names::IS_ORIENTATION_HOMED);
	return builder.createData();
}

void NovintScaffold::setPositionScale(const NovintCommonDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto& matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
		[&device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->registeredDevices.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->positionScale = scale;
	}
}

void NovintScaffold::setOrientationScale(const NovintCommonDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto& matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
		[&device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->registeredDevices.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->orientationScale = scale;
	}
}

std::shared_ptr<NovintScaffold> NovintScaffold::getOrCreateSharedInstance()
{
	static Framework::SharedInstance<NovintScaffold> sharedInstance;
	return sharedInstance.get();
}

};  // namespace Devices
};  // namespace SurgSim
