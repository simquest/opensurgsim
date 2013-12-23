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

#include <vector>
#include <list>
#include <memory>
#include <algorithm>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <cameralibrary.h>

#include "SurgSim/Devices/TrackIR/TrackIRDevice.h"
#include "SurgSim/Devices/TrackIR/TrackIRThread.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Device
{

struct TrackIRScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be wrapped
	explicit DeviceData(TrackIRDevice* device, int cameraID) :
		deviceObject(device),
		thread(),
		vector(CameraLibrary::cModuleVector::Create()),
		vectorProcessor(new CameraLibrary::cModuleVectorProcessing())
	{
		CameraLibrary::CameraList list;
		list.Refresh();
		m_camera = CameraLibrary::CameraManager::X().GetCamera(list[cameraID].UID());

		SURGSIM_ASSERT( m_camera != nullptr );
		vectorProcessorSettings = *vectorProcessor->Settings();
		vectorProcessorSettings.Arrangement = CameraLibrary::cVectorSettings::VectorClip;
		vectorProcessorSettings.ShowPivotPoint = false;
		vectorProcessorSettings.ShowProcessed  = false;
		vectorProcessorSettings.ScaleTranslationX  = device->defaultPositionScale();
		vectorProcessorSettings.ScaleTranslationY  = device->defaultPositionScale();
		vectorProcessorSettings.ScaleTranslationZ  = device->defaultPositionScale();
		vectorProcessorSettings.ScaleRotationPitch = device->defaultOrientationScale();
		vectorProcessorSettings.ScaleRotationYaw   = device->defaultOrientationScale();
		vectorProcessorSettings.ScaleRotationRoll  = device->defaultOrientationScale();
		vectorProcessor->SetSettings(vectorProcessorSettings);

		//== Plug in focal length in (mm) by converting it from pixels -> mm
		m_camera->GetDistortionModel(lensDistortion);
		vectorSettings = *vector->Settings();
		vectorSettings.Arrangement = CameraLibrary::cVectorSettings::VectorClip;
		vectorSettings.Enabled     = true;
		vectorSettings.ImagerFocalLength = lensDistortion.HorizontalFocalLength /
										   static_cast<float>(m_camera->PhysicalPixelWidth()) *
										   m_camera->ImagerWidth();
		vectorSettings.ImagerHeight = m_camera->ImagerHeight();
		vectorSettings.ImagerWidth  = m_camera->ImagerWidth();
		vectorSettings.PrincipalX   = m_camera->PhysicalPixelWidth() / 2.0;
		vectorSettings.PrincipalY   = m_camera->PhysicalPixelHeight() / 2.0;
		vectorSettings.PixelWidth   = m_camera->PhysicalPixelWidth();
		vectorSettings.PixelHeight  = m_camera->PhysicalPixelHeight();
		vector->SetSettings(vectorSettings);
	}

	~DeviceData()
	{
		m_camera->Release();
	}

	Core::DistortionModel lensDistortion;
	CameraLibrary::cModuleVector *vector;
	CameraLibrary::cModuleVectorProcessing *vectorProcessor;
	CameraLibrary::cVectorSettings vectorSettings;
	CameraLibrary::cVectorProcessingSettings vectorProcessorSettings;

	/// The corresponding device object.
	TrackIRDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<TrackIRThread> thread;
	/// Device handle to read from.
	CameraLibrary::Camera* m_camera;

	/// The mutex that protects the externally modifiable parameters.
	boost::mutex parametersMutex;

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
	m_logger(logger), m_state(new StateData)
{
	if (! m_logger)
	{
		m_logger = SurgSim::Framework::Logger::getLogger("TrackIR device");
		m_logger->setThreshold(m_defaultLogLevel);
	}
	SURGSIM_LOG_DEBUG(m_logger) << "TrackIR: Shared scaffold created.";
}


TrackIRScaffold::~TrackIRScaffold()
{
	{
		boost::lock_guard<boost::mutex> lock(m_state->mutex);

		if (! m_state->activeDeviceList.empty())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "TrackIR: Destroying scaffold while devices are active!?!";
			for (auto it = std::begin(m_state->activeDeviceList);  it != std::end(m_state->activeDeviceList);  ++it)
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
	SURGSIM_LOG_DEBUG(m_logger) << "TrackIR: Shared scaffold destroyed.";
}

std::shared_ptr<SurgSim::Framework::Logger> TrackIRScaffold::getLogger() const
{
	return m_logger;
}


bool TrackIRScaffold::registerDevice(TrackIRDevice* device)
{
	bool result = true;
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	if (!m_state->isApiInitialized)
	{
		if (!initializeSdk())
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Failed in TrackIRScaffold::registerDevice()";
			result = false;
		}
	}

	// Only proceed when initializationSdk() is successful.
	if (true == m_state->isApiInitialized)
	{
		// Make sure the object is unique.
		auto sameObject = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "TrackIR: Tried to register a device" <<
			" which is already registered!";

		// Make sure the name is unique.
		const std::string name = device->getName();
		auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
			[&name](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == name; });
		if (sameName != m_state->activeDeviceList.end())
		{
			SURGSIM_LOG_CRITICAL(m_logger) << "TrackIR: Tried to register a device when the same name is" <<
				" already present!";
			result = false;
		}
	}

	// Only proceed when no duplicate device and device name is found.
	if (true == result)
	{
		CameraLibrary::CameraList camearList;
		camearList.Refresh();
		if (camearList.Count() > static_cast<int>(m_state->activeDeviceList.size()))
		{
			// Construct the object if one exists. Then start its thread, then move it to the list.
			int cameraID = m_state->activeDeviceList.size();
			std::unique_ptr<DeviceData> info(new DeviceData(device, cameraID));
			createPerDeviceThread(info.get());
			SURGSIM_ASSERT(info->thread);

			m_state->activeDeviceList.emplace_back(std::move(info));

			if (m_state->activeDeviceList.size() == 1)
			{
				startCamera();
			}
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Registration failed.  Is a TrackIR device plugged in?";
			result = false;
		}
	}

	return result;
}


bool TrackIRScaffold::unregisterDevice(const TrackIRDevice* const device)
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
		SURGSIM_LOG_WARNING(m_logger) << "TrackIR: Attempted to release a non-registered device.";
	}
	return found;
}

void TrackIRScaffold::setPositionScale(const TrackIRDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->vectorProcessorSettings.ScaleTranslationX = scale;
		(*matching)->vectorProcessorSettings.ScaleTranslationY = scale;
		(*matching)->vectorProcessorSettings.ScaleTranslationZ = scale;
	}
}

void TrackIRScaffold::setOrientationScale(const TrackIRDevice* device, double scale)
{
	boost::lock_guard<boost::mutex> lock(m_state->mutex);
	auto matching = std::find_if(m_state->activeDeviceList.begin(), m_state->activeDeviceList.end(),
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		(*matching)->vectorProcessorSettings.ScaleRotationPitch = scale;
		(*matching)->vectorProcessorSettings.ScaleRotationYaw= scale;
		(*matching)->vectorProcessorSettings.ScaleRotationRoll = scale;
	}
}

bool TrackIRScaffold::runInputFrame(TrackIRScaffold::DeviceData* info)
{
	if (! updateDevice(info))
	{
		return false;
	}
	info->deviceObject->pushInput();
	return true;
}

bool TrackIRScaffold::updateDevice(TrackIRScaffold::DeviceData* info)
{
	SurgSim::DataStructures::DataGroup& inputData = info->deviceObject->getInputData();

	boost::lock_guard<boost::mutex> lock(info->parametersMutex);

	CameraLibrary::Frame *frame = info->m_camera->GetFrame();
	double X = 0.0, Y = 0.0, Z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
	if(frame)
	{
		info->vector->BeginFrame();
		for(int i=0; i<frame->ObjectCount(); i++)
		{
			CameraLibrary::cObject *obj = frame->Object(i);

			float x = obj->X();
			float y = obj->Y();

			Core::Undistort2DPoint(info->lensDistortion,x,y);
			info->vector->PushMarkerData(x, y, obj->Area(), obj->Width(), obj->Height());
		}
		info->vector->Calculate();
		info->vectorProcessor->PushData(info->vector);

		// Vector Clip uses 3 markers to identify the pose, i.e. 6DOF
		// If any marker is lost by the TrackIR camera, the pose will be reset to (0,0,0) with no rotation on any axis.
		if(info->vectorProcessor->MarkerCount() > 2)
		{
			info->vectorProcessor->GetOrientation(yaw, pitch, roll);
			info->vectorProcessor->GetPosition(X, Y, Z);
		}
		frame->Release();
	}

	// Assuming left hand coordinate with Y-axis points up.
	// pitch: rotation around X-axis
	// yaw: rotation around Y-axis
	// roll: rotation around Z-axis
	Vector3d position(X, Y, Z);
	Vector3d rotation(pitch, yaw, roll); // In Degrees.

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

	return true;
}

bool TrackIRScaffold::initializeSdk()
{
	SURGSIM_ASSERT(! m_state->isApiInitialized);
	bool result = false;

	bool isShutdown = CameraLibrary::CameraManager::X().AreCamerasShutdown();
	bool initialized = CameraLibrary::CameraManager::X().AreCamerasInitialized();

	if (! initialized || isShutdown)
	{
		CameraLibrary::CameraManager::X().WaitForInitialization();
	}

	if (nullptr != CameraLibrary::CameraManager::X().GetCamera())
	{
		result = true;
		m_state->isApiInitialized = true;
	}

	return result;
}

bool TrackIRScaffold::finalizeSdk()
{
	SURGSIM_ASSERT(m_state->isApiInitialized);

	bool isShutdown = CameraLibrary::CameraManager::X().AreCamerasShutdown();
	bool isInitialized = CameraLibrary::CameraManager::X().AreCamerasInitialized();

	if (isInitialized || ! isShutdown)
	{
	  // Dec-17-2013-HW It's a bug in TrackIR CameraSDK that after calling CameraLibrary::CameraManager::X().Shutdown(),
	  // calls to CameraLibrary::CameraManager::X().WaitForInitialization will thorw memory violation error.
		//CameraLibrary::CameraManager::X().Shutdown();
	}

	m_state->isApiInitialized = false;
	return true;
}

bool TrackIRScaffold::createPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(! data->thread);

	std::unique_ptr<TrackIRThread> thread(new TrackIRThread(this, data));
	thread->start();
	data->thread = std::move(thread);

	return true;
}

bool TrackIRScaffold::destroyPerDeviceThread(DeviceData* data)
{
	SURGSIM_ASSERT(data->thread);

	std::unique_ptr<TrackIRThread> thread = std::move(data->thread);
	thread->stop();
	thread.reset();

	return true;
}

bool TrackIRScaffold::startCamera()
{
	CameraLibrary::CameraManager::X().GetCamera()->SetVideoType(CameraLibrary::BitPackedPrecisionMode);
	CameraLibrary::CameraManager::X().GetCamera()->Start();
	return CameraLibrary::CameraManager::X().GetCamera()->IsCameraRunning();
}

bool TrackIRScaffold::stopCamera()
{
	CameraLibrary::CameraManager::X().GetCamera()->Stop();
	return ! CameraLibrary::CameraManager::X().GetCamera()->IsCameraRunning();
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
