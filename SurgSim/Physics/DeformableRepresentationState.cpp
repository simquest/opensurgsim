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

#include "SurgSim/Framework/Assert.h"

#include "SurgSim/Physics/DeformableRepresentationState.h"

namespace SurgSim
{

namespace Physics
{

DeformableRepresentationState::DeformableRepresentationState() :
	m_numDofPerNode(0u), m_numNodes(0u)
{
}

DeformableRepresentationState::~DeformableRepresentationState()
{
}

bool DeformableRepresentationState::operator ==(const DeformableRepresentationState& state) const
{
	return m_x == state.m_x && m_v == state.m_v && m_boundaryConditionsPerDof == state.m_boundaryConditionsPerDof;
}

bool DeformableRepresentationState::operator !=(const DeformableRepresentationState& state) const
{
	return ! ((*this) == state);
}

void DeformableRepresentationState::reset()
{
	m_x.setZero();
	m_v.setZero();
	m_a.setZero();
	m_boundaryConditionsPerDof.setConstant(false);
	m_boundaryConditionsAsDofIds.clear();
}

void DeformableRepresentationState::setNumDof(unsigned int numDofPerNode, unsigned int numNodes)
{
	const unsigned int numDof = numDofPerNode * numNodes;

	m_numDofPerNode = numDofPerNode;
	m_numNodes = numNodes;

	m_x.resize(numDof);
	m_v.resize(numDof);
	m_a.resize(numDof);
	m_boundaryConditionsPerDof.resize(numDof);

	// Zero-out everything
	reset();
}

unsigned int DeformableRepresentationState::getNumDof() const
{
	const unsigned int numDof = m_numDofPerNode * m_numNodes;

	SURGSIM_ASSERT(m_x.size() == m_v.size() && m_x.size() == m_a.size() &&
		m_x.size() == m_boundaryConditionsPerDof.size() && m_x.size() == static_cast<int>(numDof));

	return numDof;
}

unsigned int DeformableRepresentationState::getNumNodes() const
{
	return m_numNodes;
}

SurgSim::Math::Vector& DeformableRepresentationState::getPositions()
{
	return m_x;
}

const SurgSim::Math::Vector& DeformableRepresentationState::getPositions() const
{
	return m_x;
}

const SurgSim::Math::Vector3d DeformableRepresentationState::getPosition(unsigned int nodeId) const
{
	return SurgSim::Math::getSubVector(m_x, nodeId, m_numDofPerNode).segment(0, 3);
}

SurgSim::Math::Vector& DeformableRepresentationState::getVelocities()
{
	return m_v;
}

const SurgSim::Math::Vector& DeformableRepresentationState::getVelocities() const
{
	return m_v;
}

const SurgSim::Math::Vector3d DeformableRepresentationState::getVelocity(unsigned int nodeId) const
{
	return SurgSim::Math::getSubVector(m_v, nodeId, m_numDofPerNode).segment(0, 3);
}

SurgSim::Math::Vector& DeformableRepresentationState::getAccelerations()
{
	return m_a;
}

const SurgSim::Math::Vector& DeformableRepresentationState::getAccelerations() const
{
	return m_a;
}

const SurgSim::Math::Vector3d DeformableRepresentationState::getAcceleration(unsigned int nodeId) const
{
	return SurgSim::Math::getSubVector(m_a, nodeId, m_numDofPerNode);
}

void DeformableRepresentationState::addBoundaryCondition(unsigned int dof)
{
	if (! m_boundaryConditionsPerDof[dof])
	{
		m_boundaryConditionsPerDof[dof] = true;
		m_boundaryConditionsAsDofIds.push_back(dof);
	}
}

unsigned int DeformableRepresentationState::getNumBoundaryConditions() const
{
	return m_boundaryConditionsAsDofIds.size();
}

const std::vector<unsigned int>& DeformableRepresentationState::getBoundaryConditions() const
{
	return m_boundaryConditionsAsDofIds;
}

bool DeformableRepresentationState::isBoundaryCondition(unsigned int dof) const
{
	return m_boundaryConditionsPerDof[dof];
}

}; // namespace Physics

}; // namespace SurgSim

