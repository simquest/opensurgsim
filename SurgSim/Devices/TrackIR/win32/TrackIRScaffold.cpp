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

#include "SurgSim/Devices/TrackIR/TrackIRScaffold.h"

#include <algorithm>
#include <list>
#include <memory>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <cameralibrary.h>

#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Devices/TrackIR/TrackIRThread.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Device
{

struct TrackIRScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be wrapped
	explicit DeviceData(TrackIRDevice* device, int cameraID) :
		m_deviceObject(device),
		m_thread(),
		m_vector(CameraLibrary::cModuleVector::Create()),
		m_vectorProcessor(new CameraLibrary::cModuleVectorProcessing())
	{
		CameraLibrary::CameraList list;
		list.Refresh();
		m_camera = CameraLibrary::CameraManager::X().GetCamera(list[cameraID].UID());

		SURGSIM_ASSERT(m_camera != nullptr) << "Failed to obtain a camera from CameraLibrary.";
		m_camera->SetVideoType(CameraLibrary::BitPackedPrecisionMode);

		m_vectorProcessorSettings = *(m_vectorProcessor->Settings());
		m_vectorProcessorSettings.Arrangement = CameraLibrary::cVectorSettings::VectorClip;
		m_vectorProcessorSettings.ShowPivotPoint = false;
		m_vectorProcessorSettings.ShowProcessed  = false;
		m_vectorProcessorSettings.ScaleTranslationX  = device->defaultPositionScale();
		m_vectorProcessorSettings.ScaleTranslationY  = device->defaultPositionScale();
		m_vectorProcessorSettings.ScaleTranslationZ  = device->defaultPositionScale();
		m_vectorProcessorSettings.ScaleRotationPitch = device->defaultOrientationScale();
		m_vectorProcessorSettings.ScaleRotationYaw   = device->defaultOrientationScale();
		m_vectorProcessorSettings.ScaleRotationRoll  = device->defaultOrientationScale();
		m_vectorProcessor->SetSettings(m_vectorProcessorSettings);

		//== Plug in focal length in (mm) by converting it from pixels -> mm
		m_vectorSettings = *(m_vector->Settings());
		m_vectorSettings.Arrangement = CameraLibrary::cVectorSettings::VectorClip;
		m_vectorSettings.Enabled     = true;
		m_camera->GetDistortionModel(m_lensDistortion);
		m_vectorSettings.ImagerFocalLength = m_lensDistortion.HorizontalFocalLength /
										   static_cast<double>(m_camera->PhysicalPixelWidth()) *
										   m_camera->ImagerWidth();

		m_vectorSettings.ImagerHeight = m_camera->ImagerHeight();
		m_vectorSettings.ImagerWidth  = m_camera->ImagerWidth();
		m_vectorSettings.PrincipalX   = m_camera->PhysicalPixelWidth() / 2.0;
		m_vectorSettings.PrincipalY   = m_camera->PhysicalPixelHeight() / 2.0;
		m_vectorSettings.PixelWidth   = m_camera->PhysicalPixelWidth();
		m_vectorSettings.PixelHeight  = m_camera->PhysicalPixelHeight();
		m_vector->SetSettings(m_vectorSettings);
	}

	~DeviceData()
	{
		m_camera->Release();
	}

	Core::DistortionModel m_lensDistortion;
	CameraLibrary::Camera* m_camera;
	CameraLibrary::cModuleVector* m_vector;
	CameraLibrary::cModuleVectorProcessing* m_vectorProcessor;
	CameraLibrary::cVectorProcessingSettings m_vectorProcessorSettings;
	CameraLibrary::cVectorSettings m_vectorSettings;

	/// The corresponding device object.
	SurgSim::Device::TrackIRDevice* const m_deviceObject;
	/// Processing thread.
	std::unique_ptr<SurgSim::Device::TrackIRThread> m_thread;

	/// The mutex that protects the externally modifiable parameters.
	boost::mutex m_parametersMutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	DeviceData(const DeviceData&) /*= delete*/;
	DeviceData& operator=(const DeviceData&) /*= delete*/;
};

struct TrackIRScaffold::StateData
{
public:
	/// Initialize the state.
	StateData() : isApiInitialized(false)
	{
	}

	/// True if the API has been initialized (and not finalized).
	bool isApiInitialized;

	/// The list of known devices.
	std::list<std::unique_ptr<TrackIRScaffold::DeviceData>> activeDeviceList;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;

private:
	// Prevent copy construction and copy assignment.  (VS2012 does not support "= delete" yet.)
	StateData(const StateData&) /*= delete*/;
	StateData& operator=(const StateData&) /*= delete*/;
};


TrackIRScaffold::TrackIRScaffold(std::shared_ptr<SurgSim::Framework::Logger> logger) :
	m_logger(logger),
	m_state(new StateData)
{
	if (!m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("TrackIR device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "TrackIR: Shared scaffold created.";
}


TrackIRScaffold::~TrackIRScaffold()
{
	// The following block controls the duration of the mutex being locked.
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (!m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "TrackIR: Destroying scaffold while devices are active!?!";
			for (auto it = std::begin(m_state->activeDeviceList);  it != std::end(m_state->activeDeviceList);  ++it)
			{
				stopCamera((*it).get());
				if ((*it)->m_thread)
				{
					destroyPerDeviceThread(it->get());
				}
			}
			m_state->activeDeviceList.clear();
		}

		if (m_state->isApiInitialized)
		{
			if (!finalizeSdk())
			{
				SURGSIM_LOG_SEVERE(m_logger) << "Finalizing TrackIR SDK failed.";
			}
		}
	}
	SURGSIM_LOG_DEBUG(m_logger) << "TrackIR: Shared scaffold destroyed.";
}

std::shared_ptr<SurgSim::Framework::Logger> TrackIRScaffold::getLogger() const
{
	return m_logger;
}


bool TrackIRScaffold::registerDevice(TrackIRDevice* device)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (!m_state->isApiInitialized)
	{
		if (!initializeSdk())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed to initialize TrackIR SDK in TrackIRScaffold::registerDevice(). "
										 << "Continuing without the TrackIR device.";
		}
	}

	// Only proceed when initializationSdk() is successful.
	if (m_state->isApiInitialized)
	{
		// Make sure the object is unique.
		auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->m_deviceObject == device; });
		SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "TrackIR: Tried to register a device" <<
			" which is already registered!";

		// Make sure the name is unique.
		const std::string name = device->getName();
		auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
			[&name](const std::unique_ptr<DeviceData>& info) { return info->m_deviceObject->getName() == name; });
		SURGSIM_ASSERT(sameName == m_state->activeDeviceList.end()) << "TrackIR: Tried to register a device" <<
			" when the same name is already present!";

		// This assertion overlaps with above two assertions somehow.
		SURGSIM_ASSERT(m_state->activeDeviceList.size() < 1) << "There is already a TrackIR camera exists."
			<< " TrackIRScaffold only supports one TrackIR camera right now."
			<< " Behaviors of multiple cameras are undefined.\n";

		CameraLibrary::CameraList cameraList;
		cameraList.Refresh();
		if (cameraList.Count() > static_cast<int>(m_state->activeDeviceList.size()))
		{
			int cameraID = m_state->activeDeviceList.size();
			std::unique_ptr<DeviceData> info(new DeviceData(device, cameraID));
			createPerDeviceThread(info.get());
			SURGSIM_ASSERT(info->m_thread) << "Failed to create a per-device thread for TrackIR device: " <<
				info->m_deviceObject->getName() << ", with ID number " << cameraID << ".";

			startCamera(info.get());
			m_state->activeDeviceList.emplace_back(std::move(info));
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Registration failed.  Is a TrackIR device plugged in?";
			m_state->isApiInitialized = false;
		}
	}

	return m_state->isApiInitialized;
}


bool TrackIRScaffold::unregisterDevice(const TrackIRDevice* const device)
{
	bool found = false;
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);
		auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->m_deviceObject == device; });

		if (matching != m_state->activeDeviceList.end())
		{
			stopCamera((*matching).get());
			if ((*matching)->m_thread)
			{
				destroyPerDeviceThread(matching->get());
			}
			m_state->activeDeviceList.erase(matching);
			// the iterator is now invalid but that's OK
			found = true;
		}
	}

	if (!found)
	{
		SURGSIM_LOG_WARNING(m_logger) << "TrackIR: Attempted to release a non-registered device.";
	}
	return found;
}

void TrackIRScaffold::setPositionScale(const TrackIRDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->m_deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->m_parametersMutex);
		(*matching)->m_vectorProcessorSettings.ScaleTranslationX = scale;
		(*matching)->m_vectorProcessorSettings.ScaleTranslationY = scale;
		(*matching)->m_vectorProcessorSettings.ScaleTranslationZ = scale;
	}
}

void TrackIRScaffold::setOrientationScale(const TrackIRDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->m_deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->m_parametersMutex);
		(*matching)->m_vectorProcessorSettings.ScaleRotationPitch = scale;
		(*matching)->m_vectorProcessorSettings.ScaleRotationYaw = scale;
		(*matching)->m_vectorProcessorSettings.ScaleRotationRoll = scale;
	}
}

bool TrackIRScaffold::runInputFrame(TrackIRScaffold::DeviceData* info)
{
	if (!updateDevice(info))
	{
		return false;
	}
	info->m_deviceObject->pushInput();
	return true;
}

bool TrackIRScaffold::updateDevice(TrackIRScaffold::DeviceData* info)
{
	SurgSim::DataStructures::DataGroup& inputData = info->m_deviceObject->getInputData();

	boost::lock_guard<boost::mutex> lock(info->m_parametersMutex);

	CameraLibrary::Frame *frame = info->m_camera->GetFrame();
	bool poseValid = false;
	double x, y, z, pitch, yaw, roll;
	if (frame)
	{
		info->m_vector->BeginFrame();
		for(int i = 0; i < frame->ObjectCount(); ++i)
		{
			CameraLibrary::cObject *obj = frame->Object(i);
			float xValue = obj->X();
			float yValue = obj->Y();

			Core::Undistort2DPoint(info->m_lensDistortion, xValue, yValue);
			info->m_vector->PushMarkerData(xValue, yValue, obj->Area(), obj->Width(), obj->Height());
		}
		info->m_vector->Calculate();
		info->m_vectorProcessor->PushData(info->m_vector);

		// Vector Clip uses 3 markers to identify the pose, i.e. 6DOF
		// Otherwise, the pose is considered as invalid.
		if(info->m_vectorProcessor->MarkerCount() == 3)
		{
			info->m_vectorProcessor->GetOrientation(yaw, pitch, roll); // Rotations are reported in degrees.
			info->m_vectorProcessor->GetPosition(x, y, z); // Positions are reported in millimeters.
			poseValid = true;
		}
		frame->Release();
	}

	if (true == poseValid)
	{
		// Assuming left hand coordinate with Y-axis points up.
		// pitch: rotation around X-axis
		// yaw: rotation around Y-axis
		// roll: rotation around Z-axis
		Vector3d position(x / 1000.0, y / 1000.0, z / 1000.0); // Convert millimeter to meter
		Vector3d rotation(pitch, yaw, roll);

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

		inputData.poses().set("pose", pose);
	}
	else // Invalid pose. inputData.poses().hasData("pose") will be set to 'false'.
	{
		inputData.poses().reset("pose");
	}

	return true;
}

bool TrackIRScaffold::initializeSdk()
{
	SURGSIM_ASSERT(!m_state->isApiInitialized) << "TrackIR API already initialized.";

	if (!CameraLibrary::CameraManager::X().AreCamerasInitialized())
	{
		CameraLibrary::CameraManager::X().WaitForInitialization();
	}

	if (CameraLibrary::CameraManager::X().GetCamera())
	{
		m_state->isApiInitialized = true;
	}

	return m_state->isApiInitialized;
}

bool TrackIRScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized) << "TrackIR API already finalized.";

	if (!CameraLibrary::CameraManager::X().AreCamerasShutdown())
	{
	  // Dec-17-2013-HW It's a bug in TrackIR CameraSDK that after calling CameraLibrary::CameraManager::X().Shutdown(),
	  // calls to CameraLibrary::CameraManager::X().WaitForInitialization will throw memory violation error.
		//CameraLibrary::CameraManager::X().Shutdown();
	}

	m_state->isApiInitialized = false;
	return !m_state->isApiInitialized;
}

bool TrackIRScaffold::createPerDeviceThread(DeviceData* deviceData)
{
	SURGSIM_ASSERT(!deviceData->m_thread) << "Device " << deviceData->m_deviceObject->getName()
										  << " already has a thread.";

	std::unique_ptr<TrackIRThread> thread(new TrackIRThread(this, deviceData));
	thread->start();
	deviceData->m_thread = std::move(thread);

	return true;
}

bool TrackIRScaffold::destroyPerDeviceThread(DeviceData* deviceData)
{
	SURGSIM_ASSERT(deviceData->m_thread) << "No thread attached to device " << deviceData->m_deviceObject->getName();

	std::unique_ptr<TrackIRThread> thread = std::move(deviceData->m_thread);
	thread->stop();
	thread.reset();

	return true;
}

bool TrackIRScaffold::startCamera(DeviceData* info)
{
	info->m_camera->Start();
	return info->m_camera->IsCameraRunning();
}

bool TrackIRScaffold::stopCamera(DeviceData* info)
{
	info->m_camera->Stop();
	return !(info->m_camera->IsCameraRunning());
}

SurgSim::DataStructures::DataGroup TrackIRScaffold::buildDeviceInputData()
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	return builder.createData();
}

std::shared_ptr<TrackIRScaffold> TrackIRScaffold::getOrCreateSharedInstance()
{
	static SurgSim::Framework::SharedInstance<TrackIRScaffold> sharedInstance;
	return sharedInstance.get();
}

void TrackIRScaffold::setDefaultLogLevel(SurgSim::Framework::LogLevel logLevel)
{
	m_defaultLogLevel = logLevel;
}

SurgSim::Framework::LogLevel TrackIRScaffold::m_defaultLogLevel = SurgSim::Framework::LOG_LEVEL_INFO;

};  // namespace Device
};  // namespace SurgSim
