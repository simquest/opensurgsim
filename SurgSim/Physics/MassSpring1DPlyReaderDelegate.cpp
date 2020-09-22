// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/MassSpring1DPlyReaderDelegate.h"

#include "SurgSim/Math/Valid.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"

namespace SurgSim
{
namespace Physics
{

MassSpring1DPlyReaderDelegate::MassSpring1DPlyReaderDelegate()
{
}

MassSpring1DPlyReaderDelegate::MassSpring1DPlyReaderDelegate(MassSpringRepresentation* massSpring) :
	m_massSpring(massSpring)
{
	SURGSIM_ASSERT(massSpring != nullptr) << "The MassSpringRepresentation cannot be null.";
}

std::string MassSpring1DPlyReaderDelegate::getElementName() const
{
	return "1d_spring";
}

void MassSpring1DPlyReaderDelegate::endParseFile()
{
	m_massSpring->init1D(m_masses, m_nodeBoundaryConditions, m_springs);
}

void MassSpring1DPlyReaderDelegate::processMass(const std::string& elementName)
{
	MassSpringRepresentation::MassElement massElement;
	massElement.position = Math::Vector3d(m_massData.x, m_massData.y, m_massData.z);
	massElement.mass = m_massData.mass;
	m_masses.push_back(massElement);
}

void MassSpring1DPlyReaderDelegate::processSpringElement(const std::string& elementName)
{
	SURGSIM_ASSERT(m_springData.massCount == 2) << "Cannot process SpringElement with "
		<< m_springData.massCount << " endpoints (massCount).";
	MassSpringRepresentation::SpringElement spring;
	spring.nodes.resize(m_springData.massCount);
	std::copy(m_springData.indices, m_springData.indices + m_springData.massCount, spring.nodes.data());
	spring.stiffness = m_springData.stiffness;
	spring.damping = m_springData.damping;
	m_springs.push_back(spring);
}

void MassSpring1DPlyReaderDelegate::processBoundaryCondition(const std::string& elementName)
{
	m_nodeBoundaryConditions.push_back(m_boundaryConditionData);
}

} // namespace Physics
} // namespace SurgSim
