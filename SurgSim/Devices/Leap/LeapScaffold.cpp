// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Devices/Leap/LeapScaffold.h"

#include <boost/thread/mutex.hpp>
#include <Leap.h>
#include <list>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;


namespace
{

RigidTransform3d makeRigidTransform(const Leap::Matrix& rotation, const Leap::Vector& translation,
		bool isRightHanded = true)
{
	Leap::Matrix matrix = rotation;
	if (!isRightHanded)
	{
		matrix.zBasis *= -1.0;
	}

	// Convert milimeters to meters
	matrix.origin = translation * 0.001f;

	SurgSim::Math::Matrix44d transform;
	matrix.toArray4x4(transform.data());
	return RigidTransform3d(transform.transpose());
}

void updateDataGroup(const Leap::Hand& hand, SurgSim::DataStructures::DataGroup* inputData)
{
	static const std::array<std::string, 5> fingerNames = {"Thumb", "IndexFinger", "MiddleFinger", "RingFinger",
		"SmallFinger"};
	static const std::array<std::string, 4> boneNames = {"Metacarpal", "Proximal", "Intermediate", "Distal"};

	inputData->poses().set("pose", makeRigidTransform(hand.basis(), hand.palmPosition(), hand.isRight()));

	std::string name;
	const Leap::FingerList fingers = hand.fingers();
	for (const Leap::Finger& finger : fingers)
	{
		for (int boneType = Leap::Bone::TYPE_PROXIMAL; boneType <= Leap::Bone::TYPE_DISTAL; ++boneType)
		{
			Leap::Bone bone = finger.bone(static_cast<Leap::Bone::Type>(boneType));
			name = fingerNames[finger.type()] + boneNames[boneType];
			inputData->poses().set(name, makeRigidTransform(bone.basis(), bone.prevJoint(), hand.isRight()));
			inputData->scalars().set(name + "Length", bone.length() / 1000.0);
			inputData->scalars().set(name + "Width", bone.width() / 1000.0);
		}
	}
}

};

namespace SurgSim
{
namespace Device
{

class LeapScaffold::Listener : public Leap::Listener
{
public:
	Listener() :
		m_scaffold(LeapScaffold::getOrCreateSharedInstance()),
		m_logger(Framework::Logger::getLogger("Leap"))
	{
	}

	void onConnect(const Leap::Controller&) override
	{
		SURGSIM_LOG_INFO(m_logger) << "Connected to Leap Motion camera";
	}

	void onDisconnect(const Leap::Controller&) override
	{
		SURGSIM_LOG_INFO(m_logger) << "Diconnected from Leap Motion camera";
	}

	void onFrame(const Leap::Controller&) override
	{
		auto scaffold = m_scaffold.lock();
		if (scaffold != nullptr)
		{
			scaffold->handleFrame();
		}
	}

private:
	std::weak_ptr<Device::LeapScaffold> m_scaffold;
	std::shared_ptr<Framework::Logger> m_logger;
};

struct LeapScaffold::DeviceData
{
	explicit DeviceData(LeapDevice* device) :
		deviceObject(device),
		handId(std::numeric_limits<int32_t>::quiet_NaN())
	{
	}

	/// The corresponding device object.
	LeapDevice* const deviceObject;

	/// A unique id for the hand, assigned by the Leap SDK
	int32_t handId;
};

struct LeapScaffold::StateData
{
	// The SDK's interface to a single Leap Motion Camera
	Leap::Controller controller;

	/// A listener that receives updates from the Leap SDK
	std::unique_ptr<Listener> listener;

	/// The list of known devices.
	std::list<std::unique_ptr<DeviceData>> activeDevices;

	/// The mutex that protects the list of known devices.
	boost::mutex mutex;
};

LeapScaffold::LeapScaffold() :
	m_state(new StateData),
	m_logger(Framework::Logger::getLogger("Leap"))
{
}

LeapScaffold::~LeapScaffold()
{
	if (m_state->listener != nullptr)
	{
		m_state->controller.removeListener(*m_state->listener);
	}
}

bool LeapScaffold::registerDevice(LeapDevice* device)
{
	bool success = true;
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	const std::string deviceName = device->getName();
	auto sameName = [&deviceName](const std::unique_ptr<DeviceData>& info)
	{
		return info->deviceObject->getName() == deviceName;
	};
	auto found = std::find_if(m_state->activeDevices.cbegin(), m_state->activeDevices.cend(), sameName);

	if (found == m_state->activeDevices.end())
	{
		std::unique_ptr<DeviceData> info(new DeviceData(device));
		success = doRegisterDevice(info.get());
		if (success)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Registered";
			m_state->activeDevices.emplace_back(std::move(info));
		}
		else
		{
			SURGSIM_LOG_SEVERE(m_logger) << "Device " << device->getName() << ": Not registered";
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Tried to register a device when the same name, '" <<
			device->getName() << "', is already present!";
		success = false;
	}

	return success;
}

bool LeapScaffold::doRegisterDevice(DeviceData* info)
{
	if (m_state->listener == nullptr)
	{
		m_state->listener = std::unique_ptr<Listener>(new Listener);
		m_state->controller.addListener(*m_state->listener);
	}

	if (info->deviceObject->isProvidingImages())
	{
		m_state->controller.setPolicy(Leap::Controller::PolicyFlag::POLICY_IMAGES);
	}

	return true;
}

bool LeapScaffold::unregisterDevice(const LeapDevice* device)
{
	bool success = true;
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	auto& devices = m_state->activeDevices;
	if (device->isProvidingImages())
	{
		auto providingImages = [](const std::unique_ptr<DeviceData>& info)
		{
			return info->deviceObject->isProvidingImages();
		};
		if (std::find_if(devices.begin(), devices.end(), providingImages) == devices.end())
		{
			m_state->controller.clearPolicy(Leap::Controller::PolicyFlag::POLICY_IMAGES);
		}
	}

	auto sameDevice = [device](const std::unique_ptr<DeviceData>& info)
	{
		return info->deviceObject == device;
	};
	auto info = std::find_if(devices.begin(), devices.end(), sameDevice);
	if (info != devices.end())
	{
		devices.erase(info);
		SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Unregistered";
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Attempted to release a non-registered device named '" <<
			device->getName() << ".";
		success = false;
	}

	return success;
}

void LeapScaffold::handleFrame()
{
	updateHandData();
	updateImageData();

	for (auto& device : m_state->activeDevices)
	{
		device->deviceObject->pushInput();
	}
}

void LeapScaffold::updateHandData()
{
	std::list<DeviceData*> unassignedDevices;
	for (auto& device : m_state->activeDevices)
	{
		unassignedDevices.push_back(device.get());
	}

	std::list<Leap::HandList::const_iterator> newHands;
	Leap::HandList hands = m_state->controller.frame().hands();
	for (auto hand = hands.begin(); hand != hands.end(); ++hand)
	{
		auto sameHandId = [hand](const std::unique_ptr<DeviceData>& info)
		{
			return info->handId == (*hand).id();
		};
		auto assignedDevice = std::find_if(m_state->activeDevices.begin(), m_state->activeDevices.end(), sameHandId);

		if (assignedDevice != m_state->activeDevices.end())
		{
			updateDataGroup(*hand, &(*assignedDevice)->deviceObject->getInputData());
			unassignedDevices.remove(assignedDevice->get());
		}
		else
		{
			newHands.push_back(hand);
		}
	}

	static auto higherConfidence = [](const Leap::HandList::const_iterator &a, const Leap::HandList::const_iterator &b)
	{
		return (*a).confidence() > (*b).confidence();
	};
	newHands.sort(higherConfidence);

	for (auto& newHand : newHands)
	{
		auto sameHandType = [newHand](DeviceData* info)
		{
			return (info->deviceObject->getHandType() == HANDTYPE_LEFT) == (*newHand).isLeft();
		};
		auto unassignedDevice = std::find_if(unassignedDevices.begin(), unassignedDevices.end(), sameHandType);

		if (unassignedDevice != unassignedDevices.end())
		{
			(*unassignedDevice)->handId = (*newHand).id();
			updateDataGroup(*newHand, &(*unassignedDevice)->deviceObject->getInputData());
			unassignedDevices.remove(*unassignedDevice);
		}
	}

	for(auto& unassignedDevice : unassignedDevices)
	{
		unassignedDevice->deviceObject->getInputData().poses().resetAll();
	}
}

void LeapScaffold::updateImageData()
{
	Leap::ImageList images = m_state->controller.frame().images();
	if (!images.isEmpty())
	{
		typedef DataStructures::DataGroup::ImageType ImageType;
		ImageType leftImage(images[0].width(), images[0].height(), 1, images[0].data());
		ImageType rightImage(images[1].width(), images[1].height(), 1, images[1].data());

		// scale values to 0..1
		leftImage.getAsVector() *= (1.0f / 255.0f);
		rightImage.getAsVector() *= (1.0f / 255.0f);

		for (auto& device : m_state->activeDevices)
		{
			if (device->deviceObject->isProvidingImages())
			{
				device->deviceObject->getInputData().images().set("left", leftImage);
				device->deviceObject->getInputData().images().set("right", rightImage);
			}
			else
			{
				device->deviceObject->getInputData().images().resetAll();
			}
		}
	}
	else
	{
		for (auto& device : m_state->activeDevices)
		{
			device->deviceObject->getInputData().images().resetAll();
		}
	}
}

std::shared_ptr<LeapScaffold> LeapScaffold::getOrCreateSharedInstance()
{
	static auto creator = []()
	{
		return std::shared_ptr<LeapScaffold>(new LeapScaffold);
	};
	static Framework::SharedInstance<LeapScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

DataStructures::DataGroup LeapScaffold::buildDeviceInputData()
{
	DataStructures::DataGroupBuilder builder;

	builder.addImage("left");
	builder.addImage("right");

	builder.addPose("pose");
	builder.addPose("ThumbProximal");
	builder.addPose("ThumbIntermediate");
	builder.addPose("ThumbDistal");
	builder.addPose("IndexFingerProximal");
	builder.addPose("IndexFingerIntermediate");
	builder.addPose("IndexFingerDistal");
	builder.addPose("MiddleFingerProximal");
	builder.addPose("MiddleFingerIntermediate");
	builder.addPose("MiddleFingerDistal");
	builder.addPose("RingFingerProximal");
	builder.addPose("RingFingerIntermediate");
	builder.addPose("RingFingerDistal");
	builder.addPose("SmallFingerProximal");
	builder.addPose("SmallFingerIntermediate");
	builder.addPose("SmallFingerDistal");

	builder.addScalar("ThumbProximalWidth");
	builder.addScalar("ThumbIntermediateWidth");
	builder.addScalar("ThumbDistalWidth");
	builder.addScalar("IndexFingerProximalWidth");
	builder.addScalar("IndexFingerIntermediateWidth");
	builder.addScalar("IndexFingerDistalWidth");
	builder.addScalar("MiddleFingerProximalWidth");
	builder.addScalar("MiddleFingerIntermediateWidth");
	builder.addScalar("MiddleFingerDistalWidth");
	builder.addScalar("RingFingerProximalWidth");
	builder.addScalar("RingFingerIntermediateWidth");
	builder.addScalar("RingFingerDistalWidth");
	builder.addScalar("SmallFingerProximalWidth");
	builder.addScalar("SmallFingerIntermediateWidth");
	builder.addScalar("SmallFingerDistalWidth");

	builder.addScalar("ThumbProximalLength");
	builder.addScalar("ThumbIntermediateLength");
	builder.addScalar("ThumbDistalLength");
	builder.addScalar("IndexFingerProximalLength");
	builder.addScalar("IndexFingerIntermediateLength");
	builder.addScalar("IndexFingerDistalLength");
	builder.addScalar("MiddleFingerProximalLength");
	builder.addScalar("MiddleFingerIntermediateLength");
	builder.addScalar("MiddleFingerDistalLength");
	builder.addScalar("RingFingerProximalLength");
	builder.addScalar("RingFingerIntermediateLength");
	builder.addScalar("RingFingerDistalLength");
	builder.addScalar("SmallFingerProximalLength");
	builder.addScalar("SmallFingerIntermediateLength");
	builder.addScalar("SmallFingerDistalLength");

	return builder.createData();
}

void LeapScaffold::setUseHmdTrackingMode(bool useHmdTrackingMode)
{
	if (useHmdTrackingMode)
	{
		m_state->controller.setPolicy(Leap::Controller::PolicyFlag::POLICY_OPTIMIZE_HMD);
	}
	else
	{
		m_state->controller.clearPolicy(Leap::Controller::PolicyFlag::POLICY_OPTIMIZE_HMD);
	}
}

bool LeapScaffold::isUsingHmdTrackingMode() const
{
	return m_state->controller.isPolicySet(Leap::Controller::PolicyFlag::POLICY_OPTIMIZE_HMD);
}

};  // namespace Device
};  // namespace SurgSim
