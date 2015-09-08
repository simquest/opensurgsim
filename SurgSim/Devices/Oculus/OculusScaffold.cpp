// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Oculus/OculusScaffold.h"

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>
#include <list>
#include <memory>

#include <OVR_Version.h>
#if 6 == OVR_MAJOR_VERSION
#include <OVR_CAPI_0_6_0.h>
#elif 5 == OVR_MAJOR_VERSION
#include <OVR_CAPI_0_5_0.h>
#endif

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Oculus/OculusDevice.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Framework::Logger;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Device
{

struct OculusScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be wrapped
	explicit DeviceData(OculusDevice* device) :
		deviceObject(device),
		handle(nullptr)
	{
#if 6 == OVR_MAJOR_VERSION
		ovrHmd_Create(0, &handle);
#elif 5 == OVR_MAJOR_VERSION
		handle = ovrHmd_Create(0);
#endif
	}

	~DeviceData()
	{
		if (handle != nullptr)
		{
			ovrHmd_Destroy(handle);
		}
	}

	/// The device object wrapped in this class.
	OculusDevice* const deviceObject;

	/// Device handle
	ovrHmd handle;
};

struct OculusScaffold::StateData
{
	/// List of registered devices.
	std::list<std::unique_ptr<OculusScaffold::DeviceData>> registeredDevices;

	/// The mutex that protects the list of registered devices.
	boost::mutex mutex;
};

OculusScaffold::OculusScaffold() :
	Framework::BasicThread("OculusScaffold"),
	m_logger(Logger::getLogger("Devices/Oculus")),
	m_state(new StateData)
{
	SURGSIM_LOG_DEBUG(m_logger) << "Oculus: Shared scaffold created.";

	// Oculus DK2 tracks Gyroscope, Accelerometer, Magnetometer at 1000Hz and
	// positional tracking is done at 60Hz.
	setRate(1000.0);

}

OculusScaffold::~OculusScaffold()
{
	ovr_Shutdown();
}

bool OculusScaffold::registerDevice(OculusDevice* device)
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
		auto found = std::find_if(m_state->registeredDevices.cbegin(), m_state->registeredDevices.cend(), sameName);

		if (found == m_state->registeredDevices.end())
		{
			std::unique_ptr<DeviceData> info(new DeviceData(device));
			success = doRegisterDevice(info.get());
			if (success)
			{
				SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Registered";
				m_state->registeredDevices.emplace_back(std::move(info));
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

bool OculusScaffold::doRegisterDevice(DeviceData* info)
{
	if (ovrHmd_Detect() < 1)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "No available Oculus device detected.";
		return false;
	}

	if (m_state->registeredDevices.size() > 0)
	{
		SURGSIM_LOG_SEVERE(m_logger)
			<< "There is one registered Oculus device already. OculusScaffold only supports one device right now";
		return false;
	}

	if (info->handle == nullptr)
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Failed to obtain a handle to Oculus Device. Is an Oculus device plugged in?";
		return false;
	}

#if 6 == OVR_MAJOR_VERSION
	if (ovrSuccess != ovrHmd_ConfigureTracking(info->handle, ovrTrackingCap_Orientation |
															 ovrTrackingCap_MagYawCorrection |
															 ovrTrackingCap_Position, 0))
#elif 5 == OVR_MAJOR_VERSION
	if (ovrTrue != ovrHmd_ConfigureTracking(info->handle, ovrTrackingCap_Orientation |
														  ovrTrackingCap_MagYawCorrection |
														  ovrTrackingCap_Position, 0))
#endif
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Failed to configure an Oculus Device. Registration failed";
		return false;
	}

	// Query the HMD for the left and right projection matrices.
	ovrFovPort defaultLeftFov = info->handle->DefaultEyeFov[ovrEyeType::ovrEye_Left];
	ovrFovPort defaultRightFov = info->handle->DefaultEyeFov[ovrEyeType::ovrEye_Right];

	float nearPlane = info->deviceObject->getNearPlane();
	float farPlane = info->deviceObject->getFarPlane();

	ovrMatrix4f leftProjection = ovrMatrix4f_Projection(defaultLeftFov, nearPlane, farPlane,
			ovrProjectionModifier::ovrProjection_RightHanded);
	ovrMatrix4f rightProjection = ovrMatrix4f_Projection(defaultRightFov, nearPlane, farPlane,
			ovrProjectionModifier::ovrProjection_RightHanded);

	DataGroup& inputData = info->deviceObject->getInputData();
	inputData.matrices().set(DataStructures::Names::LEFT_PROJECTION_MATRIX,
			Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>
			(&leftProjection.M[0][0]).cast<double>());
	inputData.matrices().set(DataStructures::Names::RIGHT_PROJECTION_MATRIX,
			Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>
			(&rightProjection.M[0][0]).cast<double>());
	info->deviceObject->pushInput();

	return true;
}

bool OculusScaffold::unregisterDevice(const OculusDevice* const device)
{
	bool result = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto match = std::find_if(m_state->registeredDevices.begin(), m_state->registeredDevices.end(),
								  [&device](const std::unique_ptr<DeviceData>& info)
		{
			return info->deviceObject == device;
		});

		if (match != m_state->registeredDevices.end())
		{
			m_state->registeredDevices.erase(match);
			// the iterator is now invalid but that's OK
			SURGSIM_LOG_INFO(m_logger) << "Device " << getName() << ": unregistered.";
			result = true;
		}
	}

	// #threadsafety After unregistering, another thread could be in the process of registering.
	if (isRunning() && m_state->registeredDevices.size() == 0)
	{
		stop();
	}

	SURGSIM_LOG_IF(!result, m_logger, SEVERE) << "Attempted to release a non-registered device.";

	return result;
}

bool OculusScaffold::doInitialize()
{
#if 6 == OVR_MAJOR_VERSION
	if (ovrSuccess != ovr_Initialize(nullptr))
#elif 5 == OVR_MAJOR_VERSION
	if (ovrTrue != ovr_Initialize(nullptr))
#endif
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Oculus SDK initialization failed.";
	}

	return true;
}

bool OculusScaffold::doStartUp()
{
	return true;
}

bool OculusScaffold::doUpdate(double dt)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	for (auto& device : m_state->registeredDevices)
	{
		DataGroup& inputData = device->deviceObject->getInputData();

		// Query the HMD for the current tracking state.
		// If time in 2nd parameter is now or earlier, no pose prediction is made.
		// Pose is reported in a right handed coordinate system, X->RIGHT, Y->UP, Z->OUT.
		// Positive rotation is counterclockwise.
		// The XZ plane is ALWAYS aligned with the ground regardless of camera orientation.
		// Default tracking origin is one meter away from the camera.
		ovrTrackingState ts = ovrHmd_GetTrackingState(device->handle, ovr_GetTimeInSeconds());
		if (ts.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
		{
			ovrPosef ovrPose = ts.HeadPose.ThePose;

			Vector3d position(ovrPose.Position.x, ovrPose.Position.y, ovrPose.Position.z);
			Quaterniond orientation(ovrPose.Orientation.w, ovrPose.Orientation.x,
									ovrPose.Orientation.y, ovrPose.Orientation.z);
			RigidTransform3d pose = makeRigidTransform(orientation, position);

			inputData.poses().set(DataStructures::Names::POSE, pose);
		}
		else
		{
			inputData.poses().reset(DataStructures::Names::POSE);
		}

		device->deviceObject->pushInput();
	}

	return true;
}

SurgSim::DataStructures::DataGroup OculusScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	builder.addMatrix(DataStructures::Names::LEFT_PROJECTION_MATRIX);
	builder.addMatrix(DataStructures::Names::RIGHT_PROJECTION_MATRIX);
	return builder.createData();
}

std::shared_ptr<OculusScaffold> OculusScaffold::getOrCreateSharedInstance()
{
	static auto creator = []()
	{
		return std::shared_ptr<OculusScaffold>(new OculusScaffold);
	};
	static Framework::SharedInstance<OculusScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

};  // namespace Device
};  // namespace SurgSim
