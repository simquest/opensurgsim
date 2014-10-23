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

#include "SurgSim/Devices/OpenNI/OpenNIDevice.h"

#include "SurgSim/Devices/OpenNI/OpenNIScaffold.h"

namespace SurgSim
{
namespace Device
{


OpenNIDevice::OpenNIDevice(const std::string& name) :
	SurgSim::Input::CommonDevice(name, OpenNIScaffold::buildDeviceInputData())
{
}


OpenNIDevice::~OpenNIDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool OpenNIDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << "is already initialized, cannot initialize again.";
	m_scaffold = OpenNIScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(isInitialized()) << getName() << " initialization failed, cannot get scaffold.";
	return m_scaffold->registerDevice(this);
}

bool OpenNIDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << "is not initialized, cannot finalized.";
	bool success = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();
	return success;
}

bool OpenNIDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}


};  // namespace Device
};  // namespace SurgSim
