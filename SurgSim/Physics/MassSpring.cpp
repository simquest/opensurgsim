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

#include "SurgSim/Physics/MassSpring.h"

#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/MassSpringPlyReaderDelegate.h"
#include "SurgSim/Physics/Spring.h"

namespace SurgSim
{
namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Asset, SurgSim::Physics::MassSpring, MassSpring)

MassSpring::MassSpring()
{
}

void MassSpring::addMass(const std::shared_ptr<Mass> mass)
{
	m_masses.push_back(mass);
}

void MassSpring::addSpring(const std::shared_ptr<Spring> spring)
{
	m_springs.push_back(spring);
}

size_t MassSpring::getNumMasses() const
{
	return m_masses.size();
}

size_t MassSpring::getNumSprings() const
{
	return m_springs.size();
}

const std::vector<std::shared_ptr<Mass>>& MassSpring::getMasses() const
{
	return m_masses;
}

const std::vector<std::shared_ptr<Spring>>& MassSpring::getSprings() const
{
	return m_springs;
}

std::shared_ptr<Mass> MassSpring::getMass(size_t nodeId)
{
	SURGSIM_ASSERT(nodeId < getNumMasses()) << "Invalid node id to request a mass from";
	return m_masses[nodeId];
}

std::shared_ptr<Spring> MassSpring::getSpring(size_t springId)
{
	SURGSIM_ASSERT(springId < getNumSprings()) << "Invalid spring id";
	return m_springs[springId];
}


size_t MassSpring::addBoundaryCondition(size_t boundaryCondition)
{
	m_boundaryConditions.push_back(boundaryCondition);
	return m_boundaryConditions.size() - 1;
}

const std::vector<size_t>& MassSpring::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

std::vector<size_t>& MassSpring::getBoundaryConditions()
{
	return m_boundaryConditions;
}

size_t MassSpring::getBoundaryCondition(size_t id) const
{
	return m_boundaryConditions[id];
}

size_t MassSpring::addElement(const std::vector<size_t>& nodeIds)
{
	SURGSIM_ASSERT((m_nodeIds.size() == 0) || (nodeIds.size() == getNumNodesPerElement())) <<
		"Cannot add an element with " << nodeIds.size() << " nodes to a MassSpring that already has an element with " <<
		getNumNodesPerElement() << " nodes.";
	m_nodeIds.push_back(nodeIds);
	return m_nodeIds.size();
}

const std::vector<size_t>& MassSpring::getNodeIds(size_t index) const
{
	return m_nodeIds[index];
}

size_t MassSpring::getNumElements() const
{
	return m_nodeIds.size();
}

size_t MassSpring::getNumNodesPerElement() const
{
	SURGSIM_ASSERT(m_nodeIds.size() > 0) <<
		"Cannot get the number of nodes per element of a MassSpring before adding elements.";
	return m_nodeIds[0].size();
}

bool MassSpring::doLoad(const std::string & filePath)
{
	SurgSim::DataStructures::PlyReader reader(filePath);
	if (!reader.isValid())
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "'" << filePath << "' is an invalid .ply file.";
		return false;
	}

	auto delegate = std::make_shared<MassSpringPlyReaderDelegate>(this->shared_from_this());
	if (!reader.parseWithDelegate(delegate))
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger())
			<< "The input file '" << filePath << "' does not have the property required by MassSpring.";
		return false;
	}

	return true;
}

} // namespace Physics
} // namespace SurgSim
