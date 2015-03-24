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

#include "SurgSim/Devices/Leap/LeapScaffold.h"

#include <boost/thread/mutex.hpp>
#include <Leap.h>
#include <list>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Devices/Leap/LeapDevice.h"
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
	matrix.origin = translation * 0.001;

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

	for (const Leap::Finger& finger : hand.fingers())
	{
		for (int boneType = Leap::Bone::TYPE_PROXIMAL; boneType <= Leap::Bone::TYPE_DISTAL; ++boneType)
		{
			Leap::Bone bone = finger.bone(static_cast<Leap::Bone::Type>(boneType));
			inputData->poses().set(fingerNames[finger.type()] + boneNames[boneType],
					makeRigidTransform(bone.basis(), bone.prevJoint(), hand.isRight()));
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
		m_logger(SurgSim::Framework::Logger::getLogger("Leap"))
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
	std::weak_ptr<SurgSim::Device::LeapScaffold> m_scaffold;
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
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
	m_logger(SurgSim::Framework::Logger::getLogger("Leap"))
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
	return true;
}

bool LeapScaffold::unregisterDevice(const LeapDevice* device)
{
	bool success = true;
	boost::lock_guard<boost::mutex> lock(m_state->mutex);

	auto sameDevice = [device](const std::unique_ptr<DeviceData>& info)
	{
		return info->deviceObject == device;
	};
	auto info = std::find_if(m_state->activeDevices.begin(), m_state->activeDevices.end(), sameDevice);
	if (info != m_state->activeDevices.end())
	{
		success = doUnregisterDevice((*info).get());
		if (success)
		{
			SURGSIM_LOG_INFO(m_logger) << "Device " << device->getName() << ": Unregistered";
			m_state->activeDevices.erase(info);
		}
	}
	else
	{
		SURGSIM_LOG_SEVERE(m_logger) << "Attempted to release a non-registered device named '" <<
			device->getName() << ".";
		success = false;
	}

	return success;
}

bool LeapScaffold::doUnregisterDevice(DeviceData* info)
{
	return true;
}

void LeapScaffold::handleFrame()
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
			(*assignedDevice)->deviceObject->pushInput();
			unassignedDevices.remove(assignedDevice->get());
		}
		else
		{
			newHands.push_back(hand);
		}
	}

	auto higherConfidence = [](const Leap::HandList::const_iterator &a, const Leap::HandList::const_iterator &b)
	{
		return (*a).confidence() > (*b).confidence();
	};
	newHands.sort(higherConfidence);

	for (auto& newHand : newHands)
	{
		auto sameHandType = [newHand](DeviceData* info)
		{
			return info->deviceObject->isLeftHand() == (*newHand).isLeft();
		};
		auto unassignedDevice = std::find_if(unassignedDevices.begin(), unassignedDevices.end(), sameHandType);

		if (unassignedDevice != unassignedDevices.end())
		{
			(*unassignedDevice)->handId = (*newHand).id();
			updateDataGroup(*newHand, &(*unassignedDevice)->deviceObject->getInputData());
			(*unassignedDevice)->deviceObject->pushInput();
			unassignedDevices.remove(*unassignedDevice);
		}
	}

	for(auto& unassignedDevice : unassignedDevices)
	{
		unassignedDevice->deviceObject->getInputData().resetAll();
		unassignedDevice->deviceObject->pushInput();
	}
}

std::shared_ptr<LeapScaffold> LeapScaffold::getOrCreateSharedInstance()
{
	static auto creator = []()
	{
		return std::shared_ptr<LeapScaffold>(new LeapScaffold);
	};
	static SurgSim::Framework::SharedInstance<LeapScaffold> sharedInstance(creator);
	return sharedInstance.get();
}

SurgSim::DataStructures::DataGroup LeapScaffold::buildDeviceInputData()
{
	SurgSim::DataStructures::DataGroupBuilder builder;
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
	return builder.createData();
}

};  // namespace Device
};  // namespace SurgSim
