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

#include "SurgSim/Devices/ReplayPoseDevice/ReplayPoseDevice.h"

#include "SurgSim/Devices/ReplayPoseDevice/ReplayPoseScaffold.h"
#include "SurgSim/Framework/Assert.h"

namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::ReplayPoseDevice, ReplayPoseDevice);

ReplayPoseDevice::ReplayPoseDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, ReplayPoseScaffold::buildDeviceInputData()),
	m_fileName("ReplayPoseDevice.txt")
{
}

ReplayPoseDevice::~ReplayPoseDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}

const std::string ReplayPoseDevice::getFileName() const
{
	return m_fileName;
}

void ReplayPoseDevice::setFileName(const std::string& fileName)
{
	SURGSIM_ASSERT(!isInitialized()) << "The filename can only be set before initialization";
	m_fileName = fileName;
}

double ReplayPoseDevice::getRate() const
{
	return m_rate;
}

void ReplayPoseDevice::setRate(double rate)
{
	SURGSIM_ASSERT(!isInitialized()) << "The rate can only be set before initialization";
	m_rate = rate;
}

bool ReplayPoseDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << " already initialized.";
	std::shared_ptr<ReplayPoseScaffold> scaffold = ReplayPoseScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(scaffold);

	scaffold->setRate(m_rate);

	if (!scaffold->registerDevice(this))
	{
		return false;
	}
	m_scaffold = std::move(scaffold);

	return true;
}


bool ReplayPoseDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalize.";
	bool ok = m_scaffold->unregisterDevice();
	m_scaffold.reset();
	return ok;
}

bool ReplayPoseDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

};  // namespace Devices
};  // namespace SurgSim
