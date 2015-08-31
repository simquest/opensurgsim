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
using SurgSim::Math::makeRotationMatrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Devices
{

struct TrackIRScaffold::DeviceData
{
	/// Constructor
	/// \param device Device to be wrapped
	/// \param cameraID The camera identifier
	explicit DeviceData(TrackIRDevice* device, int cameraID) :
		deviceObject(device),
		thread(),
		vector(CameraLibrary::cModuleVector::Create()),
		vectorProcessor(new CameraLibrary::cModuleVectorProcessing())
	{
		CameraLibrary::CameraList list;
		list.Refresh();
		camera = CameraLibrary::CameraManager::X().GetCamera(list[cameraID].UID());

		SURGSIM_ASSERT(nullptr != camera) << "Failed to obtain a camera from CameraLibrary.";
		camera->SetVideoType(CameraLibrary::BitPackedPrecisionMode);

		CameraLibrary::cVectorProcessingSettings vectorProcessorSettings;
		vectorProcessorSettings = *(vectorProcessor->Settings());
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
		CameraLibrary::cVectorSettings vectorSettings;
		vectorSettings = *(vector->Settings());
		vectorSettings.Arrangement = CameraLibrary::cVectorSettings::VectorClip;
		vectorSettings.Enabled     = true;
		camera->GetDistortionModel(lensDistortion);
		vectorSettings.ImagerFocalLength = lensDistortion.HorizontalFocalLength /
										   static_cast<double>(camera->PhysicalPixelWidth()) *
										   camera->ImagerWidth();

		vectorSettings.ImagerHeight = camera->ImagerHeight();
		vectorSettings.ImagerWidth  = camera->ImagerWidth();
		vectorSettings.PrincipalX   = camera->PhysicalPixelWidth() / 2.0;
		vectorSettings.PrincipalY   = camera->PhysicalPixelHeight() / 2.0;
		vectorSettings.PixelWidth   = camera->PhysicalPixelWidth();
		vectorSettings.PixelHeight  = camera->PhysicalPixelHeight();
		vector->SetSettings(vectorSettings);
	}

	~DeviceData()
	{
		camera->Release();
	}

	Core::DistortionModel lensDistortion;
	CameraLibrary::Camera* camera;
	CameraLibrary::cModuleVector* vector;
	CameraLibrary::cModuleVectorProcessing* vectorProcessor;

	/// The corresponding device object.
	SurgSim::Devices::TrackIRDevice* const deviceObject;
	/// Processing thread.
	std::unique_ptr<SurgSim::Devices::TrackIRThread> thread;

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
				if ((*it)->thread)
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
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });
		SURGSIM_ASSERT(sameObject == m_state->activeDeviceList.end()) << "TrackIR: Tried to register a device" <<
			" which is already registered!";

		// Make sure the name is unique.
		const std::string name = device->getName();
		auto sameName = std::find_if(m_state->activeDeviceList.cbegin(), m_state->activeDeviceList.cend(),
			[&name](const std::unique_ptr<DeviceData>& info) { return info->deviceObject->getName() == name; });
		SURGSIM_ASSERT(sameName == m_state->activeDeviceList.end()) << "TrackIR: Tried to register a device" <<
			" when the same name is already present!";

		// The handling of multiple cameras could be done in different ways, each with trade-offs.
		// Instead of choosing an approach now, we assert on attempting to use more than one camera.
		SURGSIM_ASSERT(m_state->activeDeviceList.size() < 1) << "There is already an active TrackIR camera."
			<< " TrackIRScaffold only supports one TrackIR camera right now.";

		CameraLibrary::CameraList cameraList;
		cameraList.Refresh();
		if (cameraList.Count() > static_cast<int>(m_state->activeDeviceList.size()))
		{
			int cameraID = static_cast<int>(m_state->activeDeviceList.size());
			std::unique_ptr<DeviceData> info(new DeviceData(device, cameraID));
			createPerDeviceThread(info.get());
			SURGSIM_ASSERT(info->thread) << "Failed to create a per-device thread for TrackIR device: " <<
				info->deviceObject->getName() << ", with ID number " << cameraID << ".";

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
			[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

		if (matching != m_state->activeDeviceList.end())
		{
			stopCamera((*matching).get());
			if ((*matching)->thread)
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
		[device](const std::unique_ptr<DeviceData>& info) { return info->deviceObject == device; });

	if (matching != m_state->activeDeviceList.end())
	{
		boost::lock_guard<boost::mutex> lock((*matching)->parametersMutex);
		CameraLibrary::cVectorProcessingSettings* vectorProcessorSettings = (*matching)->vectorProcessor->Settings();
		vectorProcessorSettings->ScaleTranslationX = scale;
		vectorProcessorSettings->ScaleTranslationY = scale;
		vectorProcessorSettings->ScaleTranslationZ = scale;
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
		CameraLibrary::cVectorProcessingSettings* vectorProcessorSettings = (*matching)->vectorProcessor->Settings();
		vectorProcessorSettings->ScaleRotationPitch = scale;
		vectorProcessorSettings->ScaleRotationYaw   = scale;
		vectorProcessorSettings->ScaleRotationRoll  = scale;
	}
}

bool TrackIRScaffold::runInputFrame(TrackIRScaffold::DeviceData* info)
{
	if (!updateDevice(info))
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

	CameraLibrary::Frame *frame = info->camera->GetFrame();
	bool poseValid = false;
	double x, y, z, pitch, yaw, roll;
	if (frame)
	{
		info->vector->BeginFrame();
		for(int i = 0; i < frame->ObjectCount(); ++i)
		{
			CameraLibrary::cObject *obj = frame->Object(i);
			float xValue = obj->X();
			float yValue = obj->Y();

			Core::Undistort2DPoint(info->lensDistortion, xValue, yValue);
			info->vector->PushMarkerData(xValue, yValue, obj->Area(), obj->Width(), obj->Height());
		}
		info->vector->Calculate();
		info->vectorProcessor->PushData(info->vector);

		// Vector Clip uses 3 markers to identify the pose, i.e. 6DOF
		// Otherwise, the pose is considered as invalid.
		if(info->vectorProcessor->MarkerCount() == 3)
		{
			info->vectorProcessor->GetOrientation(yaw, pitch, roll); // Rotations are Euler Angles in degrees.
			info->vectorProcessor->GetPosition(x, y, z);             // Positions are reported in millimeters.
			poseValid = true;
		}
		frame->Release();
	}

	if (poseValid)
	{
		// Positions returned from CameraSDK are right-handed.
		Vector3d position(x / 1000.0, y / 1000.0, z / 1000.0); // Convert millimeter to meter

		// The angles returned from CameraSDK are Euler angles (y-x'-z'' intrinsic rotations): X=pitch, Y=yaw, Z=roll.
		// OSS use right handed coordinate system (X:Right, Y:Up, Z:Outward) and right hand rule for rotations.
		Matrix33d rotationX = makeRotationMatrix(pitch * M_PI / 180.0, Vector3d(Vector3d::UnitX()));
		Matrix33d rotationY = makeRotationMatrix(yaw   * M_PI / 180.0, Vector3d(Vector3d::UnitY()));
		Matrix33d rotationZ = makeRotationMatrix(roll * M_PI / 180.0, Vector3d(Vector3d::UnitZ()));
		Matrix33d orientation = rotationY * rotationX * rotationZ;

		RigidTransform3d pose;
		pose.linear() = orientation;
		pose.translation() = position;

		inputData.poses().set(SurgSim::DataStructures::Names::POSE, pose);
	}
	else // Invalid pose. inputData.poses().hasData("pose") will be set to 'false'.
	{
		inputData.poses().reset(SurgSim::DataStructures::Names::POSE);
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
	SURGSIM_ASSERT(!deviceData->thread) << "Device " << deviceData->deviceObject->getName() << " already has a thread.";

	std::unique_ptr<TrackIRThread> thread(new TrackIRThread(this, deviceData));
	thread->start();
	deviceData->thread = std::move(thread);

	return true;
}

bool TrackIRScaffold::destroyPerDeviceThread(DeviceData* deviceData)
{
	SURGSIM_ASSERT(deviceData->thread) << "No thread attached to device " << deviceData->deviceObject->getName();

	std::unique_ptr<TrackIRThread> thread = std::move(deviceData->thread);
	thread->stop();
	thread.reset();

	return true;
}

bool TrackIRScaffold::startCamera(DeviceData* info)
{
	info->camera->Start();
	return info->camera->IsCameraRunning();
}

bool TrackIRScaffold::stopCamera(DeviceData* info)
{
	info->camera->Stop();
	return !(info->camera->IsCameraRunning());
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

};  // namespace Devices
};  // namespace SurgSim
