// This filrgSim project.
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

#include <boost/thread/locks.hpp>

#include "SurgSim/Devices/DeviceFilters/ForceScale.h"

#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Devices
{

ForceScale::ForceScale(const std::string& name) :
	DeviceFilter(name),
	m_forceScale(1.0),
	m_torqueScale(1.0)
{
}

void ForceScale::filterOutput(const std::string& device, const DataGroup& dataToFilter, DataGroup* result)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);

	*result = dataToFilter;

	Vector3d force;
	if (dataToFilter.vectors().get(DataStructures::Names::FORCE, &force))
	{
		force *= m_forceScale;
		result->vectors().set(DataStructures::Names::FORCE, force);
	}

	Vector3d torque;
	if (dataToFilter.vectors().get(DataStructures::Names::TORQUE, &torque))
	{
		torque *= m_torqueScale;
		result->vectors().set(DataStructures::Names::TORQUE, torque);
	}

	// Scale the spring's contribution to the force and torque. First three rows calculate force, last three for torque.
	DataGroup::DynamicMatrixType springJacobian;
	if (dataToFilter.matrices().get(DataStructures::Names::SPRING_JACOBIAN, &springJacobian))
	{
		springJacobian.block<3,6>(0, 0) *= m_forceScale;
		springJacobian.block<3,6>(3, 0) *= m_torqueScale;
		result->matrices().set(DataStructures::Names::SPRING_JACOBIAN, springJacobian);
	}

	// Scale the damper's contribution to the force and torque. First three rows calculate force, last three for torque.
	DataGroup::DynamicMatrixType damperJacobian;
	if (dataToFilter.matrices().get(DataStructures::Names::DAMPER_JACOBIAN, &damperJacobian))
	{
		damperJacobian.block<3,6>(0, 0) *= m_forceScale;
		damperJacobian.block<3,6>(3, 0) *= m_torqueScale;
		result->matrices().set(DataStructures::Names::DAMPER_JACOBIAN, damperJacobian);
	}
}

void ForceScale::setForceScale(double forceScale)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_forceScale = forceScale;
}

void ForceScale::setTorqueScale(double torqueScale)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);
	m_torqueScale = torqueScale;
}

};  // namespace Devices
};  // namespace SurgSim
