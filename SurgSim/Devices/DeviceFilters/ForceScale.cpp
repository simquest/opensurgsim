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
	CommonDevice(name),
	m_forceScale(1.0),
	m_torqueScale(1.0),
	m_forceIndex(-1),
	m_torqueIndex(-1),
	m_springJacobianIndex(-1),
	m_damperJacobianIndex(-1),
	m_cachedOutputIndices(false)
{
}

ForceScale::~ForceScale()
{
	finalize();
}

bool ForceScale::initialize()
{
	return true;
}

bool ForceScale::finalize()
{
	return true;
}

void ForceScale::initializeInput(const std::string& device, const DataGroup& inputData)
{
	getInputData() = inputData;
}

void ForceScale::handleInput(const std::string& device, const DataGroup& inputData)
{
	getInputData() = inputData;
	pushInput();
}

bool ForceScale::requestOutput(const std::string& device, DataGroup* outputData)
{
	bool state = pullOutput();
	if (state)
	{
		if (!m_cachedOutputIndices)
		{
			const DataGroup& initialOutputData = getOutputData();
			m_forceIndex = initialOutputData.vectors().getIndex(SurgSim::DataStructures::Names::FORCE);
			m_torqueIndex = initialOutputData.vectors().getIndex(SurgSim::DataStructures::Names::TORQUE);
			m_springJacobianIndex =
				initialOutputData.matrices().getIndex(SurgSim::DataStructures::Names::SPRING_JACOBIAN);
			m_damperJacobianIndex =
				initialOutputData.matrices().getIndex(SurgSim::DataStructures::Names::DAMPER_JACOBIAN);
			m_cachedOutputIndices = true;
		}
		outputFilter(getOutputData(), outputData);
	}
	return state;
}

void ForceScale::outputFilter(const DataGroup& dataToFilter, DataGroup* result)
{
	boost::lock_guard<boost::mutex> lock(m_mutex);

	*result = dataToFilter;  // Pass on all the data entries.

	Vector3d force = Vector3d::Zero();
	if (dataToFilter.vectors().get(m_forceIndex, &force))
	{
		force *= m_forceScale;
		result->vectors().set(m_forceIndex, force);
	}

	Vector3d torque = Vector3d::Zero();
	if (dataToFilter.vectors().get(m_torqueIndex, &torque))
	{
		torque *= m_torqueScale;
		result->vectors().set(m_torqueIndex, torque);
	}

	// Scale the spring's contribution to the force and torque. First three rows calculate force, last three for torque.
	SurgSim::DataStructures::DataGroup::DynamicMatrixType springJacobian;
	if (dataToFilter.matrices().get(m_springJacobianIndex, &springJacobian))
	{
		springJacobian.block<3,6>(0, 0) *= m_forceScale;
		springJacobian.block<3,6>(3, 0) *= m_torqueScale;
		result->matrices().set(m_springJacobianIndex, springJacobian);
	}

	// Scale the damper's contribution to the force and torque. First three rows calculate force, last three for torque.
	SurgSim::DataStructures::DataGroup::DynamicMatrixType damperJacobian;
	if (dataToFilter.matrices().get(m_damperJacobianIndex, &damperJacobian))
	{
		damperJacobian.block<3,6>(0, 0) *= m_forceScale;
		damperJacobian.block<3,6>(3, 0) *= m_torqueScale;
		result->matrices().set(m_damperJacobianIndex, damperJacobian);
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
