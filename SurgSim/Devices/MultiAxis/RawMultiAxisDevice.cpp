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

#include "SurgSim/Devices/MultiAxis/RawMultiAxisDevice.h"

#include "SurgSim/Devices/MultiAxis/RawMultiAxisScaffold.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Device
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Device::RawMultiAxisDevice, RawMultiAxisDevice);

RawMultiAxisDevice::RawMultiAxisDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, RawMultiAxisScaffold::buildDeviceInputData()),
	m_positionScale(defaultPositionScale()),
	m_orientationScale(defaultOrientationScale()),
	m_useAxisDominance(false)
{
}


RawMultiAxisDevice::~RawMultiAxisDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool RawMultiAxisDevice::initialize()
{
	SURGSIM_ASSERT(! isInitialized());
	std::shared_ptr<RawMultiAxisScaffold> scaffold = RawMultiAxisScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	if (! scaffold->registerDevice(this))
	{
		return false;
	}

	m_scaffold = std::move(scaffold);
	m_scaffold->setPositionScale(this, m_positionScale);
	m_scaffold->setOrientationScale(this, m_orientationScale);
	m_scaffold->setAxisDominance(this, m_useAxisDominance);
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}


bool RawMultiAxisDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold->getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	bool ok = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return ok;
}


bool RawMultiAxisDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


void RawMultiAxisDevice::setPositionScale(double scale)
{
	m_positionScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setPositionScale(this, m_positionScale);
	}
}


double RawMultiAxisDevice::getPositionScale() const
{
	return m_positionScale;
}


void RawMultiAxisDevice::setOrientationScale(double scale)
{
	m_orientationScale = scale;
	if (m_scaffold)
	{
		m_scaffold->setOrientationScale(this, m_orientationScale);
	}
}


double RawMultiAxisDevice::getOrientationScale() const
{
	return m_orientationScale;
}


void RawMultiAxisDevice::setAxisDominance(bool onOff)
{
	m_useAxisDominance = onOff;
	if (m_scaffold)
	{
		m_scaffold->setAxisDominance(this, m_useAxisDominance);
	}
}


bool RawMultiAxisDevice::isUsingAxisDominance() const
{
	return m_useAxisDominance;
}


};  // namespace Device
};  // namespace SurgSim
