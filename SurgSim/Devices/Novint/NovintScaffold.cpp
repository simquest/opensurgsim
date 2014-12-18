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
#include <boost/math/special_functions/fpclassify.hpp>
#include <hdl/hdl.h>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Novint/NovintDevice.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Clock.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Framework/Timer.h"
#include "SurgSim/Math/KalmanFilter.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Framework::Clock;
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix66d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;


namespace SurgSim
{
namespace Device
{

/// Estimate angular velocity by measuring angular velocity, a 3-vector.  The state is {velocity, acceleration}.
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> AngularMeasurementMatrix;
typedef Eigen::Matrix<double, 3, 1> AngularMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> AngularObservationMatrix;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AngularStateMatrix;
typedef Eigen::Matrix<double, 6, 1> AngularStateVector;

/// Measure translational velocity by measuring translation, a 3-vector. The state is {position, velocity}.
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> LinearMeasurementMatrix;
typedef Eigen::Matrix<double, 3, 1> LinearMeasurementVector;
typedef Eigen::Matrix<double, 3, 6, Eigen::RowMajor> LinearObservationMatrix;
typedef Eigen::Matrix<double, 6, 6, Eigen::RowMajor> LinearStateMatrix;
typedef Eigen::Matrix<double, 6, 1> LinearStateVector;

class NovintScaffold::Handle
{
public:
	Handle() :
		m_deviceHandle(HDL_INVALID_HANDLE),
		m_scaffold(NovintScaffold::getInstance())
	{
	}

	Handle(const std::string& serial) :
		m_deviceHandle(HDL_INVALID_HANDLE),
		m_scaffold(NovintScaffold::getInstance())
	{
		create(serial);
	}

	~Handle()
	{
		SURGSIM_ASSERT(! isValid()) << "Expected destroy() to be called before Handle object destruction.";
	}

	bool isValid() const
	{
		return (m_deviceHandle != HDL_INVALID_HANDLE);
	}

	bool create(const std::string& serial)
	{
		SURGSIM_ASSERT(! isValid());

		HDLDeviceHandle deviceHandle = HDL_INVALID_HANDLE;
		deviceHandle = hdlInitDeviceBySerialNumber(serial.c_str());

		if (m_scaffold.checkForFatalError("Failed to initialize"))
		{
			// HDAL error message already logged
			SURGSIM_LOG_INFO(m_scaffold.getLogger()) << std::endl <<
				"  HDAL serial number: '" << serial << "'" << std::endl;
			return false;
		}
		else if (deviceHandle == HDL_INVALID_HANDLE)
		{
			SURGSIM_LOG_SEVERE(m_scaffold.getLogger()) << "Novint: Failed to initialize a device." <<
				std::endl <<
				"  Error details: unknown (HDAL returned an invalid handle)" << std::endl <<
				"  HDAL serial number: '" << serial << "'" << std::endl;
			return false;
		}

		m_deviceHandle = deviceHandle;
		return true;
	}

	bool destroy()
	{
		SURGSIM_ASSERT(isValid());

		HDLDeviceHandle deviceHandle = m_deviceHandle;
		if (deviceHandle == HDL_INVALID_HANDLE)
		{
			return false;
		}
		m_deviceHandle = HDL_INVALID_HANDLE;

		hdlUninitDevice(deviceHandle);
		m_scaffold.checkForFatalError("Couldn't disable device");
		return true;
	}

	HDLDeviceHandle get() const
	{
		SURGSIM_ASSERT(isValid());
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The HDAL device handle (or HDL_INVALID_HANDLE if not valid).
	HDLDeviceHandle m_deviceHandle;
	/// The scaffold.
	NovintScaffold& m_scaffold;
};


class NovintScaffold::Callback
{
public:
	Callback() :
		m_callbackHandle(0),
		m_haveCallback(false),
		m_scaffold(NovintScaffold::getInstance())
	{
		create();
	}

	~Callback()
	{
		if (m_haveCallback)
		{
			destroy();
		}
	}

	bool isValid() const
	{
		return m_haveCallback;
	}

	bool create()
	{
		SURGSIM_ASSERT(! m_haveCallback);

		const bool isCallbackNonblocking = false;
		m_callbackHandle = hdlCreateServoOp(run, &m_scaffold, isCallbackNonblocking);
		if (m_scaffold.checkForFatalError("Couldn't run haptic callback"))
		{
			return false;
		}
		m_haveCallback = true;
		return true;
	}

	bool destroy()
	{
		SURGSIM_ASSERT(m_haveCallback);
		hdlDestroyServoOp(m_callbackHandle);
		if (m_scaffold.checkForFatalError("Couldn't stop haptic callback"))
		{
			return false;
		}
		m_haveCallback = false;
		return true;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Callback(const Callback&) /*= delete*/;
	Callback& operator=(const Callback&) /*= delete*/;

	/// The callback wrapper passed to HDAL.
	/// \param [in,out] data	The user data (in our case, the scaffold pointer).
	/// \return	HD_CALLBACK_CONTINUE to wait for the next frame, or HD_CALLBACK_DONE to terminate further calls.
	static HDLServoOpExitCode run(void* data);

	/// The haptic loop callback handle.
	HDLOpHandle m_callbackHandle;
	/// True if the callback has been created (and not destroyed).
	bool m_haveCallback;
	/// The scaffold.
	NovintScaffold& m_scaffold;
};


struct NovintScaffold::DeviceData
{
	/// Initialize the state.
	DeviceData(const std::string& apiName, NovintCommonDevice* device) :
		initializationName(apiName),
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
		position(Vector3d::Zero()),
		linearVelocity(Vector3d::Zero()),
		angularVelocity(Vector3d::Zero()),
		jointAngles(Vector3d::Zero()),
		force(Vector3d::Zero()),
		torque(Vector4d::Zero()),
		orientationTransform(RigidTransform3d::Identity()),
		scaledPose(RigidTransform3d::Identity()),
		timer()
	{
		buttonStates.fill(false);
		timer.setMaxNumberOfFrames(1);

		LinearObservationMatrix linearObservationMatrix;
		linearObservationMatrix << LinearMeasurementMatrix::Identity(), LinearMeasurementMatrix::Zero();
		linearFilter.setObservationMatrix(linearObservationMatrix);
		linearFilter.setInitialState(LinearStateVector::Zero());
		linearFilter.setInitialStateCovariance(LinearStateMatrix::Ones() * 1000.0);
		LinearStateMatrix linearProcessNoise;
		linearProcessNoise << LinearMeasurementMatrix::Identity() * 0.1, LinearMeasurementMatrix::Zero(),
			LinearMeasurementMatrix::Zero(), LinearMeasurementMatrix::Identity() * 100.0;
		linearFilter.setProcessNoiseCovariance(linearProcessNoise);
		linearFilter.setMeasurementNoiseCovariance(LinearMeasurementMatrix::Identity() * 1.0);

		AngularObservationMatrix angularObservationMatrix;
		angularObservationMatrix << AngularMeasurementMatrix::Identity(), AngularMeasurementMatrix::Zero();
		angularFilter.setObservationMatrix(angularObservationMatrix);
		angularFilter.setInitialState(AngularStateVector::Zero());
		angularFilter.setInitialStateCovariance(AngularStateMatrix::Ones() * 1000.0);
		angularFilter.setProcessNoiseCovariance(AngularStateMatrix::Identity() * 10.0);
		angularFilter.setMeasurementNoiseCovariance(AngularMeasurementMatrix::Identity());
	}


	/// The maximum number of buttons supported by any device object.
	static const size_t MAX_NUM_BUTTONS = 4;

	/// Type used to store button states.
	typedef std::array<bool, MAX_NUM_BUTTONS> ButtonStates;


	/// The HDAL device name.
	const std::string initializationName;
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

	/// The position value from the device.
	Vector3d position;
	Vector3d linearVelocity;
	Vector3d angularVelocity;
	/// The orientation value from the device.  If the device is not 7Dof the orientation is always Identity.
	RigidTransform3d orientationTransform;
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

	SurgSim::Framework::Timer timer;
	SurgSim::Math::RigidTransform3d lastPose;

	/// The translational Kalman filter.
	SurgSim::Math::KalmanFilter linearFilter;

	/// The angular Kalman filter.
	SurgSim::Math::KalmanFilter angularFilter;

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
	std::unique_ptr<NovintScaffold::Callback> callback;

	/// The registered devices.
	std::list<std::unique_ptr<NovintScaffold::DeviceData>> registeredDevices;

	/// The map from serial number to Handle for all devices that were available when the SDK was initialized.
	std::unordered_map<std::string, std::shared_ptr<NovintScaffold::Handle>> serialToHandle;

	/// List of devices that have been unregistered and should have their forces zeroed in the next update.
	std::list<std::shared_ptr<NovintScaffold::Handle>> unregisteredHandles;

	/// The map from name to serial number for all devices.
	std::map<std::string, std::string> nameToSerial;

	/// The mutex that protects the list of registered devices.
	boost::mutex mutex;

	/// Time of the initialization of the latest handle.
	Clock::time_point initializationTime;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};


HDLServoOpExitCode NovintScaffold::Callback::run(void* data)
{
	NovintScaffold* scaffold = static_cast<NovintScaffold*>(data);
	if (! scaffold->runHapticFrame())
	{
		//...do something?...
	}

	// Should return HDL_SERVOOP_CONTINUE to wait for the next frame, or HDL_SERVOOP_EXIT to terminate the calls.
	return HDL_SERVOOP_CONTINUE;
}



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
	m_logger(SurgSim::Framework::Logger::getLogger("Devices/Novint")), m_state(new StateData)
{
	{
		// Drain the HDAL error stack
		HDLError errorCode = hdlGetError();
		while (errorCode != HDL_NO_ERROR)
		{
			errorCode = hdlGetError();
		}
	}

	SURGSIM_LOG_DEBUG(m_logger) << "Novint: Shared scaffold created.";
}


NovintScaffold::~NovintScaffold()
{
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (!m_state->registeredDevices.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Novint: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			m_state->registeredDevices.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Novint: Shared scaffold destroyed.";
}


bool NovintScaffold::registerDevice(NovintCommonDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->registeredDevices.end()) << "Novint: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->registeredDevices.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Novint: Tried to register a device when the same name is" <<
			" already present!";
		return false;
	}

	// Make sure the initialization name is unique.
	const std::string initializationName = device->getInitializationName();
	auto sameInitializationName = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(),
		[&initializationName](const std::unique_ptr<DeviceData>& info)
			{ return info->deviceObject->getInitializationName() == initializationName; });
	if (sameInitializationName != m_state->registeredDevices.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Novint: Tried to register a device when the same initialization" <<
			" (HDAL) name is already present!";
		return false;
	}

	if (!m_state->isApiInitialized)
	{
		if (!initializeSdk())
		{
			return false;
		}
	}

	// Construct the object, start its thread, then move it to the list.
	// Note that since Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers, storing a
	// list of unique_ptr results in nicer code than storing a list of DeviceData values directly.
	std::unique_ptr<DeviceData> info(new DeviceData(initializationName, device));
	if (! initializeDeviceState(info.get()))
	{
		return false;   // message already printed
	}
	m_state->registeredDevices.emplace_back(std::move(info));
	return true;
}


bool NovintScaffold::unregisterDevice(const NovintCommonDevice* const device)
{
	bool result = false;
	std::unique_ptr<DeviceData> savedInfo;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->registeredDevices.end())
		{
			result = true;
			savedInfo = std::move(*matching);
			m_state->registeredDevices.erase(matching);
			m_state->unregisteredHandles.push_back(savedInfo->deviceHandle);
			// the iterator is now invalid but that's OK
		}
	}

	SURGSIM_LOG_IF(!result, m_logger, WARNING) << "Novint: Attempted to release a non-registered device.";
	return result;
}

std::shared_ptr<NovintScaffold::Handle> NovintScaffold::findHandle(const std::string& name)
{
	std::shared_ptr<NovintScaffold::Handle> handle;
	if (name == "")
	{
		// get the first available
		for (auto it : m_state->serialToHandle)
		{
			auto possibleHandle = it.second;
			auto matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
				[possibleHandle](const std::unique_ptr<DeviceData>& info)
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
		if (m_state->nameToSerial.count(name) > 0)
		{
			const std::string serial = m_state->nameToSerial[name];
			if (m_state->serialToHandle.count(serial) > 0)
			{
				handle = m_state->serialToHandle[serial];
			}
			else
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Attempted to register a device named '" << name <<
					"', which should map to serial number " << serial <<
					", but no device with that serial number is available.";
			}
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Attempted to register a device named '" << name <<
				"', but that name does not map to a serial number.  Was the configuration file found?" << 
				" Does it contain the text of a YAML node (for the map from name to serial number)?  Is '" << name <<
				"' a key in that map?";
		}
	}
	return handle;
}


bool NovintScaffold::initializeDeviceState(DeviceData* info)
{
	SURGSIM_ASSERT(info->deviceHandle == nullptr) <<
		"The raw handle should be nullptr before initialization.";

	info->deviceHandle = findHandle(info->deviceObject->getInitializationName());
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
			info->isDeviceRollAxisReversed = true;   // sigh
			// I wish we had someplace to put these instead of hardcoding.
			info->eulerAngleOffsetRoll = 0;
			info->eulerAngleOffsetYaw = -75. * M_PI / 180.;
			info->eulerAngleOffsetPitch = -50. * M_PI / 180.;
		}
		else
		{
			SURGSIM_LOG_DEBUG(m_logger) << "'" << info->initializationName << "' is right-handed.";
			info->isDeviceRollAxisReversed = false;
			// I wish we had someplace to put these instead of hardcoding.
			info->eulerAngleOffsetRoll = 0;
			info->eulerAngleOffsetYaw = +75. * M_PI / 180.;
			info->eulerAngleOffsetPitch = +50. * M_PI / 180.;
		}
	}

	return result;
}


bool NovintScaffold::updateDevice(DeviceData* info)
{
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();

	boost::lock_guard<boost::mutex> lock(info->parametersMutex);

	// TODO(bert): this code should cache the access indices.

	hdlMakeCurrent(info->deviceHandle->get());	// This device is now "current", and all hdlXxx calls apply to it.
	bool fatalError = checkForFatalError(false, "hdlMakeCurrent()");

	// Receive the current device position (in meters), orientation transform, and button state bitmap.
	hdlGripGetAttributev(HDL_GRIP_POSITION, 0, info->position.data());
	fatalError = checkForFatalError(fatalError, "hdlGripGetAttributev(HDL_GRIP_POSITION)");

	info->buttonStates.fill(false);
	hdlGripGetAttributesb(HDL_GRIP_BUTTON, static_cast<int>(info->buttonStates.size()), info->buttonStates.data());
	fatalError = checkForFatalError(fatalError, "hdlGripGetAttributesb(HDL_GRIP_BUTTON)");

	// Get the additional 7DoF data if available.
	if (info->isDevice7Dof)
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
		Matrix33d orientation = rotationY * rotationZ * rotationX;
		// Put the result into the orientation transform
		info->orientationTransform.linear() = orientation;
	}

	checkDeviceHoming(info);

	info->force.setZero();
	info->torque.setZero();
	if (info->isDeviceHomed)
	{
		CalculateVelocity(info);


		bool desiredGravityCompensation = false;
		if (outputData.booleans().get("gravityCompensation", &desiredGravityCompensation))
		{
			setGravityCompensation(info, desiredGravityCompensation);
		}

		calculateForceAndTorque(info);
	}

	// Set the force command (in newtons).
	hdlGripSetAttributev(HDL_GRIP_FORCE, 0, info->force.data()); // 2nd arg is index; output force is always "vector #0"
	fatalError = checkForFatalError(fatalError, "hdlGripSetAttributev(HDL_GRIP_FORCE)");

	// Set the torque vector.  Also set the jaw squeeze torque (as 4th element of the array)-- though this is not used
	// anywhere at the moment.
	// The 2nd arg to this call is the count; we're setting 4 doubles.
	hdlGripSetAttributesd(HDL_GRIP_TORQUE, 4, info->torque.data());
	fatalError = checkForFatalError(fatalError, "hdlGripSetAttributesd(HDL_GRIP_TORQUE)");

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

	if (info->isPositionHomed && info->isOrientationHomed && ! info->isDeviceHomed)
	{
		// Wait until the tool is pointed forwards (i.e. perpendicular to the Falcon centerline) before proclaiming the
		// whole device homed.
		Vector3d forwardDirection = Vector3d::UnitX();
		double forwardMetric = forwardDirection.dot(info->orientationTransform.linear() * forwardDirection);

		if (forwardMetric >= info->forwardPointingPoseThreshold)
		{
			// It looks like everything is ready!
			info->isDeviceHomed = true;
		}
	}

	if (! info->isPositionHomed)
	{
		info->position.setZero();
	}
	if (! info->isOrientationHomed)
	{
		info->orientationTransform.setIdentity();
	}

	info->scaledPose.translation() = info->position * info->positionScale;
	info->scaledPose.linear() = info->orientationTransform.linear();
}

void NovintScaffold::calculateForceAndTorque(DeviceData* info)
{
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();

	// Set the DeviceData's force to the nominal force, if provided.
	Vector3d nominalForce = Vector3d::Zero();
	outputData.vectors().get(SurgSim::DataStructures::Names::FORCE, &nominalForce);
	info->force = nominalForce;

	// If the springJacobian was provided, multiply with the change in position since the output data was set,
	// to get a delta force.  This way a linearized output force is calculated at haptic update rates.
	Vector6d deltaPosition;
	SurgSim::DataStructures::DataGroup::DynamicMatrixType springJacobian;
	bool havespringJacobian =
		outputData.matrices().get(SurgSim::DataStructures::Names::SPRING_JACOBIAN, &springJacobian);
	if (havespringJacobian)
	{
		RigidTransform3d poseForNominal = info->scaledPose;
		outputData.poses().get(SurgSim::DataStructures::Names::INPUT_POSE, &poseForNominal);

		Vector3d rotationVector = Vector3d::Zero();
		SurgSim::Math::computeRotationVector(info->scaledPose, poseForNominal, &rotationVector);

		SurgSim::Math::setSubVector(info->scaledPose.translation() - poseForNominal.translation(), 0, 3,
			&deltaPosition);
		SurgSim::Math::setSubVector(rotationVector, 1, 3, &deltaPosition);

		info->force += springJacobian.block<3,6>(0, 0) * deltaPosition;
	}

	// If the damperJacobian was provided, calculate a delta force based on the change in velocity.
	Vector6d deltaVelocity;
	SurgSim::DataStructures::DataGroup::DynamicMatrixType damperJacobian;
	bool havedamperJacobian =
		outputData.matrices().get(SurgSim::DataStructures::Names::DAMPER_JACOBIAN, &damperJacobian);
	if (havedamperJacobian)
	{
		Vector3d linearVelocity = info->linearVelocity;
		Vector3d angularVelocity = info->angularVelocity;

		Vector3d linearVelocityForNominal = linearVelocity;
		outputData.vectors().get(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY, &linearVelocityForNominal);
		Vector3d angularVelocityForNominal = angularVelocity;
		outputData.vectors().get(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY, &angularVelocityForNominal);

		SurgSim::Math::setSubVector(linearVelocity - linearVelocityForNominal, 0, 3, &deltaVelocity);
		SurgSim::Math::setSubVector(angularVelocity - angularVelocityForNominal, 1, 3, &deltaVelocity);

		Vector3d forceFromAcceleration = damperJacobian.block<3,6>(0, 0) * deltaVelocity;
		info->force += damperJacobian.block<3,6>(0, 0) * deltaVelocity;
	}

	// Calculate the torque command if applicable (and convert newton-meters to command counts).
	if (info->isDevice7Dof)
	{
		Vector3d nominalTorque = Vector3d::Zero();
		outputData.vectors().get(SurgSim::DataStructures::Names::TORQUE, &nominalTorque);
		Vector3d torque = nominalTorque;

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
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();
	inputData.poses().set(SurgSim::DataStructures::Names::POSE, info->scaledPose);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_1, info->buttonStates[0]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_2, info->buttonStates[1]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_3, info->buttonStates[2]);
	inputData.booleans().set(SurgSim::DataStructures::Names::BUTTON_4, info->buttonStates[3]);
	inputData.booleans().set(SurgSim::DataStructures::Names::IS_HOMED, info->isDeviceHomed);
	inputData.booleans().set(SurgSim::DataStructures::Names::IS_POSITION_HOMED, info->isPositionHomed);
	inputData.booleans().set(SurgSim::DataStructures::Names::IS_ORIENTATION_HOMED, info->isOrientationHomed);
	inputData.vectors().set(SurgSim::DataStructures::Names::LINEAR_VELOCITY, info->linearVelocity);
	inputData.vectors().set(SurgSim::DataStructures::Names::ANGULAR_VELOCITY, info->angularVelocity);
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
	m_state->initializationTime = Clock::now();
}

void NovintScaffold::destroyAllHandles()
{
	for (auto it : m_state->serialToHandle)
	{
		it.second->destroy();
	}
}

std::map<std::string, std::string> NovintScaffold::getNameMap()
{
	std::map<std::string, std::string> map;
	std::vector<std::string> paths;
	paths.push_back(".");
	SurgSim::Framework::ApplicationData applicationData(paths);
	std::string fileName;
	if (applicationData.tryFindFile("novint.ini", &fileName))
	{
		SURGSIM_LOG_INFO(m_logger) << "Found novint.ini at '" << fileName << "'.";
		YAML::Node node = YAML::LoadFile(fileName);
		map = node["nameToSerial"].as<std::map<std::string, std::string>>();
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Failed to find novint.ini, cannot map names to serial numbers.";
	}
	return map;
}

bool NovintScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	// The canonical HDAL approach (Programmer's Guide, section 4.7 Multiple devices) is:
	// 1) hdlInitX on all devices that will be used by this application,
	// 2) hdlStart (must be after all hdlInitX and before hdlCreateServoOp), then
	// 3) hdlCreateServoOp (starts the callback).
	// According to the Programmer's Guide, it is undefined behavior to physically attach/detach devices during
	// the application.

	createAllHandles();
	m_state->nameToSerial = getNameMap();

	bool result = true;
	if (!createHapticLoop())
	{
		destroyAllHandles();
		result = false;
	}

	m_state->isApiInitialized = result;
	return result;
}


bool NovintScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	// The HDAL seems to do bad things (and the CRT complains) if we uninitialize the device too soon.
	const int MINIMUM_LIFETIME_MILLISECONDS = 500;
	Clock::time_point earliestEndTime =
		m_state->initializationTime + boost::chrono::milliseconds(MINIMUM_LIFETIME_MILLISECONDS);
	boost::this_thread::sleep_until(earliestEndTime);

	if (m_state->callback)
	{
		destroyHapticLoop();
	}

	destroyAllHandles();

	m_state->isApiInitialized = false;
	return true;
}


bool NovintScaffold::runHapticFrame()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->registeredDevices.begin();  it != m_state->registeredDevices.end();  ++it)
	{
		(*it)->deviceObject->pullOutput();
		if (updateDevice((*it).get()))
		{
			(*it)->deviceObject->pushInput();
		}
	}

	for (auto handle : m_state->unregisteredHandles)
	{
		hdlMakeCurrent(handle->get());

		bool desiredGravityCompensation = false;
		hdlGripSetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &desiredGravityCompensation);
		checkForFatalError("Cannot set gravity compensation state on recently unregistered device.");

		Vector3d force = Vector3d::Zero();
		hdlGripSetAttributev(HDL_GRIP_FORCE, 0, force.data());
		checkForFatalError("hdlGripSetAttributev(HDL_GRIP_FORCE)");

		Vector4d torque = Vector4d::Zero();
		hdlGripSetAttributesd(HDL_GRIP_TORQUE, 4, torque.data());
		checkForFatalError("hdlGripSetAttributesd(HDL_GRIP_TORQUE)");
	}
	m_state->unregisteredHandles.clear();

	return true;
}


bool NovintScaffold::createHapticLoop()
{
	SURGSIM_ASSERT(! m_state->callback);

	if (! startScheduler())
	{
		return false;
	}

	std::unique_ptr<Callback> callback(new Callback);
	if (! callback->isValid())
	{
		stopScheduler();
		return false;
	}

	m_state->callback = std::move(callback);
	return true;
}


bool NovintScaffold::destroyHapticLoop()
{
	SURGSIM_ASSERT(m_state->callback);

	checkForFatalError("Error prior to stopping haptic callback");  // NOT considered an error for return code!

	bool didDestroy = m_state->callback->destroy();
	m_state->callback.reset(nullptr);

	bool didStop = stopScheduler();

	return didDestroy && didStop;
}


bool NovintScaffold::startScheduler()
{
	hdlStart();
	if (checkForFatalError("Couldn't start the scheduler"))
	{
		return false;
	}
	return true;
}


bool NovintScaffold::stopScheduler()
{
	hdlStop();
	if (checkForFatalError("Couldn't stop the scheduler"))
	{
		return false;
	}
	return true;
}


bool NovintScaffold::getGravityCompensation(const NovintScaffold::DeviceData* info, bool* gravityCompensationState)
{
	bool state1 = true;
	hdlGripGetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &state1);
	if (checkForFatalError("Cannot get gravity compensation (#1)"))
	{
		return false;  // HDAL reported an error; an error message was already logged.
	}

	bool state2 = false;
	hdlGripGetAttributeb(HDL_GRIP_GRAVITY_COMP, 1, &state2);
	if (checkForFatalError("Cannot get gravity compensation (#2)"))
	{
		return false;  // HDAL reported an error; an error message was already logged.
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
			return false;  // HDAL reported an error; an error message was already logged.
		}

		if (! getGravityCompensation(info, &state))
		{
			return false;  // HDAL reported an error; an error message was already logged.
		}
		else if (state == gravityCompensationState)
		{
			// If the state has been changed, log a message.
			if (isInitialStateValid && (initialState != gravityCompensationState))
			{
				if (gravityCompensationState)
				{
					SURGSIM_LOG_INFO(m_logger) << "gravity compensation for '" << info->deviceObject->getName() <<
						"' changed to ENABLED.";
				}
				else
				{
					SURGSIM_LOG_INFO(m_logger) << "gravity compensation for '" << info->deviceObject->getName() <<
						"' changed to disabled.";
				}
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


static std::string convertErrorCodeToString(HDLError errorCode)
{
	// Convert a HDLError to text.  The text was cut+pasted from the comments in Novint's hdlErrors.h file.
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


bool NovintScaffold::checkForFatalError(const char* message)
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

	SURGSIM_LOG_SEVERE(m_logger) << "Novint: " << message << std::endl <<
		"  Error text: '" << convertErrorCodeToString(errorCode) << "'" << std::endl <<
		"  Error code: 0x" << std::hex << std::setw(4) << std::setfill('0') << errorCode << std::endl;

	return (isFatal || anotherFatalError);
}

SurgSim::DataStructures::DataGroup NovintScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_1);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_2);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_3);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_4);
	builder.addBoolean(SurgSim::DataStructures::Names::IS_HOMED);
	builder.addBoolean(SurgSim::DataStructures::Names::IS_POSITION_HOMED);
	builder.addBoolean(SurgSim::DataStructures::Names::IS_ORIENTATION_HOMED);
	builder.addVector(SurgSim::DataStructures::Names::LINEAR_VELOCITY);
	builder.addVector(SurgSim::DataStructures::Names::ANGULAR_VELOCITY);
	return builder.createData();
}

void NovintScaffold::setPositionScale(const NovintCommonDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->registeredDevices.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->positionScale = scale;
	}
}

void NovintScaffold::setOrientationScale(const NovintCommonDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	if (matching != m_state->registeredDevices.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->orientationScale = scale;
	}
}


NovintScaffold& NovintScaffold::getInstance()
{
	/// A SharedInstance is not appropriate, because we need to have a single point of contact with the HDAL
	/// and letting the SharedInstance destruct and be re-constructed would not follow the canonical sequence
	/// of calls to the HDAL.  That has caused the hdl calls to fail, hang, and exit the application.
	static NovintScaffold instance;
	return instance;
}

std::shared_ptr<SurgSim::Framework::Logger> NovintScaffold::getLogger() const
{
	return m_logger;
}

void NovintScaffold::CalculateVelocity(DeviceData* info)
{
	info->timer.markFrame();
	const double period = info->timer.getLastFramePeriod();
	if ((info->timer.getNumberOfClockFails() == 0) && (boost::math::isnormal(period)) && (period > 0.0))
	{
		LinearStateMatrix linearStateTransition;
		linearStateTransition <<
			LinearMeasurementMatrix::Identity(), period * LinearMeasurementMatrix::Identity(),
			LinearMeasurementMatrix::Zero(),     LinearMeasurementMatrix::Identity();

		info->linearFilter.setStateTransition(linearStateTransition);
		const LinearStateVector& linearState = info->linearFilter.update(info->scaledPose.translation());
		info->linearVelocity = linearState.segment<3>(3);

		AngularStateMatrix angularStateTransition;
		angularStateTransition <<
			AngularMeasurementMatrix::Identity(), period * AngularMeasurementMatrix::Identity(),
			AngularMeasurementMatrix::Zero(),     AngularMeasurementMatrix::Identity();
		info->angularFilter.setStateTransition(angularStateTransition);
		SurgSim::Math::Vector3d rotation;
		SurgSim::Math::computeRotationVector(info->scaledPose, info->lastPose, &rotation);
		const AngularStateVector& angularState = info->angularFilter.update(rotation / period);
		info->angularVelocity = angularState.segment<3>(0);
	}
	else
	{
		info->timer.start();

		info->linearVelocity.Zero();
		info->angularVelocity.Zero();
	}
	info->lastPose = info->scaledPose;
}


};  // namespace Device
};  // namespace SurgSim
