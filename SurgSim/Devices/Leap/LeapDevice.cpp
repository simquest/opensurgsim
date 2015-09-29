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

#include "SurgSim/Devices/Leap/LeapDevice.h"

#include <algorithm>

#include "SurgSim/Devices/Leap/LeapScaffold.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::LeapDevice, LeapDevice);

LeapDevice::LeapDevice(const std::string& name) :
	Input::CommonDevice(name, LeapScaffold::buildDeviceInputData()),
	m_handType(HANDTYPE_RIGHT),
	m_isProvidingImages(false)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(LeapDevice, bool, UseHmdTrackingMode, isUsingHmdTrackingMode,
			setUseHmdTrackingMode);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(LeapDevice, bool, ProvideImages, isProvidingImages, setProvideImages);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(LeapDevice, SurgSim::Devices::HandType, HandType, getHandType, setHandType);
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

void LeapDevice::setUseHmdTrackingMode(bool useHmdTrackingMode)
{
	if (isInitialized())
	{
		m_scaffold->setUseHmdTrackingMode(useHmdTrackingMode);
	}
	m_requestedHmdTrackingMode = useHmdTrackingMode;
}

bool LeapDevice::isUsingHmdTrackingMode() const
{
	if (isInitialized())
	{
		return m_scaffold->isUsingHmdTrackingMode();
	}
	else
	{
		return m_requestedHmdTrackingMode.getValue();
	}
}

void LeapDevice::setProvideImages(bool produceImages)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot call setProvideImages after LeapDevice is initialized";
	m_isProvidingImages = produceImages;
}

bool LeapDevice::isProvidingImages() const
{
	return m_isProvidingImages;
}

bool LeapDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << "is already initialized, cannot initialize again.";
	m_scaffold = LeapScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(isInitialized()) << getName() << " initialization failed, cannot get scaffold.";

	if (m_requestedHmdTrackingMode.hasValue())
	{
		m_scaffold->setUseHmdTrackingMode(m_requestedHmdTrackingMode.getValue());
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


};  // namespace Devices
};  // namespace SurgSim
