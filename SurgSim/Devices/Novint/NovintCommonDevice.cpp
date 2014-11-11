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

#include "SurgSim/Devices/Novint/NovintCommonDevice.h"

#include <iostream>
#include <iomanip>

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Devices/Novint/NovintScaffold.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


namespace SurgSim
{
namespace Device
{


NovintCommonDevice::NovintCommonDevice(const std::string& uniqueName, const std::string& initializationName) :
	SurgSim::Input::CommonDevice(uniqueName, NovintScaffold::buildDeviceInputData()),
	m_scaffold(NovintScaffold::getInstance()), m_initialized(false),
	m_initializationName(initializationName), m_positionScale(1.0), m_orientationScale(1.0)
{
}


NovintCommonDevice::~NovintCommonDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


std::string NovintCommonDevice::getInitializationName() const
{
	return m_initializationName;
}


bool NovintCommonDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized());

	if (!m_scaffold.registerDevice(this))
	{
		return false;
	}
	m_initialized = true;

	SURGSIM_LOG_INFO(m_scaffold.getLogger()) << "Device " << getName() << ": " << "Initialized.";
	return true;
}


bool NovintCommonDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized());
	SURGSIM_LOG_INFO(m_scaffold.getLogger()) << "Device " << getName() << ": " << "Finalizing.";
	bool result = m_scaffold.unregisterDevice(this);
	m_initialized = !result;
	return result;
}


bool NovintCommonDevice::isInitialized() const
{
	return m_initialized;
}

void NovintCommonDevice::setPositionScale(double scale)
{
	m_positionScale = scale;
	if (isInitialized())
	{
		m_scaffold.setPositionScale(this, m_positionScale);
	}
}

double NovintCommonDevice::getPositionScale() const
{
	return m_positionScale;
}

void NovintCommonDevice::setOrientationScale(double scale)
{
	m_orientationScale = scale;
	if (isInitialized())
	{
		m_scaffold.setOrientationScale(this, m_orientationScale);
	}
}

double NovintCommonDevice::getOrientationScale() const
{
	return m_orientationScale;
}

bool NovintCommonDevice::is7DofDevice() const
{
	return false;
}

};  // namespace Device
};  // namespace SurgSim
