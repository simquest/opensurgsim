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

#include "SurgSim/Devices/Leap/LeapDevice.h"

#include "SurgSim/Devices/Leap/LeapScaffold.h"

namespace SurgSim
{
namespace Device
{

LeapDevice::LeapDevice(const std::string& name) :
	SurgSim::Input::CommonDevice(name, LeapScaffold::buildDeviceInputData()),
	m_handType(HANDTYPE_RIGHT)
{
}


LeapDevice::~LeapDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

void LeapDevice::setHandType(HandType type)
{
	m_handType = type;
}

HandType LeapDevice::getHandType() const
{
	return m_handType;
}

void LeapDevice::setTrackingMode(LeapTrackingMode mode)
{
	if (isInitialized())
	{
		m_scaffold->setTrackingMode(mode);
	}
	m_trackingMode.setValue(mode);
}

LeapTrackingMode LeapDevice::getTrackingMode() const
{
	if (isInitialized())
	{
		return m_scaffold->getTrackingMode();
	}
	else
	{
		return m_trackingMode.getValue();
	}
}

bool LeapDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << "is already initialized, cannot initialize again.";
	m_scaffold = LeapScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(isInitialized()) << getName() << " initialization failed, cannot get scaffold.";

	if (m_trackingMode.hasValue())
	{
		m_scaffold->setTrackingMode(m_trackingMode.getValue());
	}

	return m_scaffold->registerDevice(this);
}

bool LeapDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << "is not initialized, cannot finalized.";
	bool success = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return success;
}

bool LeapDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


};  // namespace Device
};  // namespace SurgSim
