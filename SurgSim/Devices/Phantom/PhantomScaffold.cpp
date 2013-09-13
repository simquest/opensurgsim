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

#include "SurgSim/Devices/Phantom/PhantomScaffold.h"

#include <vector>
#include <memory>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <HD/hd.h>

#include <SurgSim/Devices/Phantom/PhantomDevice.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/SharedInstance.h>
#include <SurgSim/DataStructures/DataGroup.h>
#include <SurgSim/DataStructures/DataGroupBuilder.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


namespace SurgSim
{
namespace Device
{

	
class PhantomScaffold::Handle
{
public:
	Handle() :
		m_deviceHandle(HD_INVALID_HANDLE),
		m_scaffold(PhantomScaffold::getOrCreateSharedInstance())
	{
	}

	Handle(const std::string& deviceName, const std::string& initializationName) :
		m_deviceHandle(HD_INVALID_HANDLE),
		m_scaffold(PhantomScaffold::getOrCreateSharedInstance())
	{
		create(deviceName, initializationName);
	}

	~Handle()
	{
		SURGSIM_ASSERT(! isValid()) << "Expected destroy() to be called before Handle object destruction.";
	}

	bool isValid() const
	{
		return (m_deviceHandle != HD_INVALID_HANDLE);
	}

	bool create(const std::string& deviceName, const std::string& initializationName)
	{
		SURGSIM_ASSERT(! isValid());

		HHD deviceHandle = HD_INVALID_HANDLE;
		if (initializationName.length() > 0)
		{
			deviceHandle = hdInitDevice(initializationName.c_str());
		}
		else
		{
			deviceHandle = hdInitDevice(HD_DEFAULT_DEVICE);
		}

		if (m_scaffold->checkForFatalError("Failed to initialize"))
		{
			// HDAPI error message already logged
			SURGSIM_LOG_INFO(m_scaffold->getLogger()) << std::endl <<
				"  Device name: '" << deviceName << "'" << std::endl <<
				"  OpenHaptics device name: '" << initializationName << "'" << std::endl;
			return false;
		}
		else if (deviceHandle == HD_INVALID_HANDLE)
		{
			SURGSIM_LOG_SEVERE(m_scaffold->getLogger()) << "Phantom: Failed to initialize '" << deviceName << "'" <<
				std::endl <<
				"  Error details: unknown (HDAPI returned an invalid handle)" << std::endl <<
				"  OpenHaptics device name: '" << initializationName << "'" << std::endl;
			return false;
		}

		m_deviceHandle = deviceHandle;
		return true;
	}

	bool destroy()
	{
		SURGSIM_ASSERT(isValid());

		HHD deviceHandle = m_deviceHandle;
		if (deviceHandle == HD_INVALID_HANDLE)
		{
			return false;
		}
		m_deviceHandle = HD_INVALID_HANDLE;

		hdDisableDevice(deviceHandle);
		m_scaffold->checkForFatalError("Couldn't disable device");
		return true;
	}

	HHD get() const
	{
		SURGSIM_ASSERT(isValid());
		return m_deviceHandle;
	}

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	Handle(const Handle&) /*= delete*/;
	Handle& operator=(const Handle&) /*= delete*/;

	/// The OpenHaptics device handle (or HD_INVALID_HANDLE if not valid).
	HHD m_deviceHandle;
	/// The scaffold.
	std::shared_ptr<PhantomScaffold> m_scaffold;
};


class PhantomScaffold::Callback
{
public:
	Callback() :
		m_callbackHandle(0),
		m_haveCallback(false),
		m_scaffold(PhantomScaffold::getOrCreateSharedInstance())
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
		m_callbackHandle = hdScheduleAsynchronous(run, m_scaffold.get(), HD_DEFAULT_SCHEDULER_PRIORITY);
		if (m_scaffold->checkForFatalError("Couldn't run haptic callback"))
		{
			return false;
		}
		m_haveCallback = true;
		return true;
	}

	bool destroy()
	{
		SURGSIM_ASSERT(m_haveCallback);
		hdUnschedule(m_callbackHandle);
		if (m_scaffold->checkForFatalError("Couldn't stop haptic callback"))
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

	/// The callback wrapper passed to OpenHaptics.
	/// \param [in,out] data	The user data (in our case, the scaffold pointer).
	/// \return	HD_CALLBACK_CONTINUE to wait for the next frame, or HD_CALLBACK_DONE to terminate further calls.
	static HDCallbackCode HDCALLBACK run(void* data);

	/// The haptic loop callback handle.
	HDSchedulerHandle m_callbackHandle;
	/// True if the callback has been created (and not destroyed).
	bool m_haveCallback;
	/// The scaffold.
	std::shared_ptr<PhantomScaffold> m_scaffold;
};


struct PhantomScaffold::DeviceData
{
	/// Initialize the state.
	DeviceData(const std::string& apiName, PhantomDevice* device) :
		initializationName(apiName),
		deviceObject(device),
		positionValue(positionBuffer),
		transformValue(transformBuffer),
		forceValue(forceBuffer)
	{
		positionValue.setZero();   // also clears positionBuffer
		transformValue.setIdentity();   // also sets transformBuffer
		buttonsBuffer = 0;

		forceValue.setZero();   // also clears forceBuffer
	}

	/// The OpenHaptics device name.
	const std::string initializationName;
	/// The corresponding device object.
	PhantomDevice* const deviceObject;

	/// The device handle wrapper.
	PhantomScaffold::Handle deviceHandle;

	/// The raw position read from the device.
	double positionBuffer[3];
	/// The raw pose transform read from the device.
	double transformBuffer[16];
	/// The raw button state read from the device.
	int buttonsBuffer;

	/// The raw force to be written to the device.
	double forceBuffer[3];

	/// The position value from the device, permanently connected to positionBuffer.
	Eigen::Map<Vector3d> positionValue;
	/// The pose transform value from the device, permanently connected to transformBuffer.
	Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> transformValue;

	/// The force value to be written to the device, permanently connected to positionBuffer.
	Eigen::Map<Vector3d> forceValue;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};


struct PhantomScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// Wrapper for the haptic loop callback handle.
	std::unique_ptr<PhantomScaffold::Callback> callback;

	/// The list of known devices.
	std::list<std::unique_ptr<PhantomScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};


HDCallbackCode HDCALLBACK PhantomScaffold::Callback::run(void* data)
{
	PhantomScaffold* scaffold = static_cast<PhantomScaffold*>(data);
	if (! scaffold->runHapticFrame())
	{
		//...do something?...
	}

	// Should return HD_CALLBACK_CONTINUE to wait for the next frame, or HD_CALLBACK_DONE to terminate the calls.
	return HD_CALLBACK_CONTINUE;
}



PhantomScaffold::PhantomScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("Phantom device");
		m_logger->setThreshold(m_defaultLogLevel);
	}

	{
		// Drain the HDAPI error stack
		HDErrorInfo error = hdGetError();
		while (error.errorCode != HD_SUCCESS)
		{
			error = hdGetError();
		}
	}

	SURGSIM_LOG_DEBUG(m_logger) << "Phantom: Shared scaffold created.";
}


PhantomScaffold::~PhantomScaffold()
{
	if (m_state->callback)
	{
		destroyHapticLoop();
	}
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Phantom: Destroying scaffold while devices are active!?!";
			// do anything special with each device?
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			finalizeSdk();
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "Phantom: Shared scaffold destroyed.";
}


bool PhantomScaffold::registerDevice(PhantomDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (! m_state->isApiInitialized)
	{
		if (! initializeSdk())
		{
			return false;
		}
	}

	// Make sure the object is unique.
	auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
	SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "Phantom: Tried to register a device" <<
		" which is already present!";

	// Make sure the name is unique.
	const std::string deviceName = device->getName();
	auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == deviceName; });
	if (sameName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Phantom: Tried to register a device when the same name is" <<
			" already present!";
		return false;
	}

	// Make sure the initialization name is unique.
	const std::string initializationName = device->getInitializationName();
	auto sameInitializationName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
		[&deviceName](const std::unique_ptr<DeviceData>& info)
			{ return info->deviceObject->getInitializationName() == deviceName; });
	if (sameInitializationName != m_state->activeDeviceList.end())
	{
		SURGSIM_LOG_CRITICAL(m_logger) << "Phantom: Tried to register a device when the same initialization" <<
			" (OpenHaptics) name is already present!";
		return false;
	}

	// Construct the object, start its thread, then move it to the list.
	// Note that since Visual Studio 2010 doesn't support multi-argument emplace_back() for STL containers, storing a
	// list of unique_ptr results in nicer code than storing a list of DeviceData values directly.
	std::unique_ptr<DeviceData> info(new DeviceData(initializationName, device));
	if (! initializeDeviceState(info.get()))
	{
		return false;   // message already printed
	}
	m_state->activeDeviceList.emplace_back(std::move(info));

	if (m_state->activeDeviceList.size() == 1)
	{
		// If this is the first device, create the haptic loop as well.
		// The haptic loop should be created AFTER initializing the device, or OpenHaptics will complain.
		createHapticLoop();
	}
	return true;
}


bool PhantomScaffold::unregisterDevice(const PhantomDevice* const device)
{
	std::unique_ptr<DeviceData> savedInfo;
	bool haveOtherDevices = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		if (matching != m_state->activeDeviceList.end())
		{
			savedInfo = std::move(*matching);
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
		}
		haveOtherDevices = (m_state->activeDeviceList.size() > 0);
	}

	bool status = true;
	if (! savedInfo)
	{
		SURGSIM_LOG_WARNING(m_logger) << "Phantom: Attempted to release a non-registered device.";
		status = false;
	}
	else
	{
		// If you attempt to destroy the device while the haptic callback is active, you see lots of nasty errors
		// under OpenHaptics 3.0.  The solution seems to be to disable the haptic callback when destroying the device.
		destroyHapticLoop();

		finalizeDeviceState(savedInfo.get());
		savedInfo.reset(nullptr);

		if (haveOtherDevices)
		{
			// If there are other devices left, we need to recreate the haptic callback.
			// If there aren't, we don't need the callback... and moreover, trying to create it will fail.
			createHapticLoop();
		}
	}
	return status;
}


bool PhantomScaffold::initializeDeviceState(DeviceData* info)
{
	SURGSIM_ASSERT(! info->deviceHandle.isValid());

	if (! info->deviceHandle.create(info->deviceObject->getName(), info->deviceObject->getInitializationName()))
	{
		return false;  // message was already printed
	}

	// Enable forces.
	hdMakeCurrentDevice(info->deviceHandle.get());
	hdEnable(HD_FORCE_OUTPUT);
	checkForFatalError("Couldn't enable forces");

	return true;
}


bool PhantomScaffold::finalizeDeviceState(DeviceData* info)
{
	bool status = false;
	if (info->deviceHandle.isValid())
	{
		status = info->deviceHandle.destroy();
	}
	return status;
}


bool PhantomScaffold::updateDevice(PhantomScaffold::DeviceData* info)
{
	const SurgSim::DataStructures::DataGroup& outputData = info->deviceObject->getOutputData();
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();

	//boost::lock_guard<boost::mutex> lock(info->parametersMutex);

	// TODO(bert): this code should cache the access indices.

	SurgSim::Math::Vector3d force;
	if (outputData.vectors().get("force", &force))
	{
		info->forceValue = force;
	}
	else
	{
		info->forceValue.setZero();
	}

	hdBeginFrame(info->deviceHandle.get());

	// Receive the current device position (in millimeters!), pose transform, and button state bitmap.
	hdGetDoublev(HD_CURRENT_POSITION, info->positionBuffer);
	hdGetDoublev(HD_CURRENT_TRANSFORM, info->transformBuffer);
	hdGetIntegerv(HD_CURRENT_BUTTONS, &(info->buttonsBuffer));

	// Set the force command (in newtons).
	hdSetDoublev(HD_CURRENT_FORCE, info->forceBuffer);
	//hdSetDoublev(HD_CURRENT_TORQUE, info->torqueBuffer);

	hdEndFrame(info->deviceHandle.get());

	bool fatalError = checkForFatalError("Error in device update");

	{
		RigidTransform3d pose;
		pose.linear() = info->transformValue.block<3,3>(0,0);
		pose.translation() = info->positionValue * 0.001;  // convert from millimeters to meters!

		inputData.poses().set("pose", pose);
		inputData.booleans().set("button1", (info->buttonsBuffer & HD_DEVICE_BUTTON_1) != 0);
		inputData.booleans().set("button2", (info->buttonsBuffer & HD_DEVICE_BUTTON_2) != 0);
		inputData.booleans().set("button3", (info->buttonsBuffer & HD_DEVICE_BUTTON_3) != 0);
		inputData.booleans().set("button4", (info->buttonsBuffer & HD_DEVICE_BUTTON_4) != 0);
	}

	return !fatalError;
}


bool PhantomScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = true;
	return true;
}


bool PhantomScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	// nothing to do!

	m_state->isApiInitialized = false;
	return true;
}


bool PhantomScaffold::runHapticFrame()
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto it = m_state->activeDeviceList.begin();  it != m_state->activeDeviceList.end();  ++it)
	{
		(*it)->deviceObject->pullOutput();
		if (updateDevice((*it).get()))
		{
			(*it)->deviceObject->pushInput();
		}
	}

	return true;
}


bool PhantomScaffold::createHapticLoop()
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


bool PhantomScaffold::destroyHapticLoop()
{
	SURGSIM_ASSERT(m_state->callback);

	checkForFatalError("Error prior to stopping haptic callback");  // NOT considered an error for return code!

	bool didDestroy = m_state->callback->destroy();
	m_state->callback.reset(nullptr);

	bool didStop = stopScheduler();

	return didDestroy && didStop;
}


bool PhantomScaffold::startScheduler()
{
	hdStartScheduler();
	if (checkForFatalError("Couldn't start the scheduler"))
	{
		return false;
	}
	return true;
}


bool PhantomScaffold::stopScheduler()
{
	hdStopScheduler();
	if (checkForFatalError("Couldn't stop the scheduler"))
	{
		return false;
	}
	return true;
}


bool PhantomScaffold::checkForFatalError(const char* message)
{
	HDErrorInfo error = hdGetError();
	if (error.errorCode == HD_SUCCESS)
	{
		return false;
	}

	// The HD API maintains an error stack, so in theory there could be more than one error pending.
	// We do head recursion to get them all in the correct order, and hope we don't overrun the stack...
	bool anotherFatalError = checkForFatalError(message);

	bool isFatal = ((error.errorCode != HD_WARM_MOTORS) &&
		(error.errorCode != HD_EXCEEDED_MAX_FORCE) &&
		(error.errorCode != HD_EXCEEDED_MAX_FORCE_IMPULSE) &&
		(error.errorCode != HD_EXCEEDED_MAX_VELOCITY) &&
		(error.errorCode != HD_FORCE_ERROR));

	SURGSIM_LOG_SEVERE(m_logger) << "Phantom: " << message << std::endl <<
		"  Error text: '" << hdGetErrorString(error.errorCode) << "'" << std::endl <<
		"  Error code: 0x" << std::hex << std::setw(4) << std::setfill('0') << error.errorCode <<
		" (internal: " << std::dec << error.internalErrorCode << ")" << std::endl;

	return (isFatal || anotherFatalError);
}

SurgSim::DataStructures::DataGroup PhantomScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addBoolean("button1");
	builder.addBoolean("button2");
	builder.addBoolean("button3");
	builder.addBoolean("button4");
	return builder.createData();
}

std::shared_ptr<PhantomScaffold> PhantomScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<PhantomScaffold> sharedInstance;
	return sharedInstance.get();
}

void PhantomScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel PhantomScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;


};  // namespace Device
};  // namespace SurgSim