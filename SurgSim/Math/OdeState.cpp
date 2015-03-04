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

#include "SurgSim/Math/OdeState.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Valid.h"

namespace SurgSim
{

namespace Math
{

OdeState::OdeState() : m_numDofPerNode(0u), m_numNodes(0u)
{
}

OdeState::~OdeState()
{
}

bool OdeState::operator ==(const OdeState& state) const
{
	return m_x == state.m_x && m_v == state.m_v && m_boundaryConditionsPerDof == state.m_boundaryConditionsPerDof;
}

bool OdeState::operator !=(const OdeState& state) const
{
	return !((*this) == state);
}

void OdeState::reset()
{
	m_x.setZero();
	m_v.setZero();
	m_boundaryConditionsPerDof.setConstant(false);
	m_boundaryConditionsAsDofIds.clear();
}

void OdeState::setNumDof(size_t numDofPerNode, size_t numNodes)
{
	const size_t numDof = numDofPerNode * numNodes;

	m_numDofPerNode = numDofPerNode;
	m_numNodes = numNodes;

	m_x.resize(numDof);
	m_v.resize(numDof);
	m_boundaryConditionsPerDof.resize(numDof);

	// Zero-out everything
	reset();
}

size_t OdeState::getNumDof() const
{
	const size_t numDof = m_numDofPerNode * m_numNodes;

	SURGSIM_ASSERT(m_x.size() == m_v.size() && m_x.size() == m_boundaryConditionsPerDof.size() && m_x.size() >= 0
				   && static_cast<size_t>(m_x.size()) == numDof);

	return numDof;
}

size_t OdeState::getNumNodes() const
{
	return m_numNodes;
}

SurgSim::Math::Vector& OdeState::getPositions()
{
	return m_x;
}

const SurgSim::Math::Vector& OdeState::getPositions() const
{
	return m_x;
}

const SurgSim::Math::Vector3d OdeState::getPosition(size_t nodeId) const
{
	return SurgSim::Math::getSubVector(m_x, nodeId, m_numDofPerNode).segment(0, 3);
}

SurgSim::Math::Vector& OdeState::getVelocities()
{
	return m_v;
}

const SurgSim::Math::Vector& OdeState::getVelocities() const
{
	return m_v;
}

const SurgSim::Math::Vector3d OdeState::getVelocity(size_t nodeId) const
{
	return SurgSim::Math::getSubVector(m_v, nodeId, m_numDofPerNode).segment(0, 3);
}

void OdeState::addBoundaryCondition(size_t nodeId)
{
	SURGSIM_ASSERT(m_numDofPerNode != 0u) <<
										  "Number of dof per node = 0. Make sure to call setNumDof() prior to adding boundary conditions.";

	for (size_t nodeDofId = 0; nodeDofId < m_numDofPerNode; ++nodeDofId)
	{
		addBoundaryCondition(nodeId, nodeDofId);
	}
}

void OdeState::addBoundaryCondition(size_t nodeId, size_t nodeDofId)
{
	SURGSIM_ASSERT(m_numDofPerNode != 0u) <<
										  "Number of dof per node = 0. Make sure to call setNumDof() prior to adding boundary conditions.";
	SURGSIM_ASSERT(nodeId < m_numNodes) << "Invalid nodeId " << nodeId << " number of nodes is " << m_numNodes;
	SURGSIM_ASSERT(nodeDofId < m_numDofPerNode) <<
			"Invalid nodeDofId " << nodeDofId << " number of dof per node is " << m_numDofPerNode;

	size_t globalDofId = nodeId * m_numDofPerNode + nodeDofId;
	if (! m_boundaryConditionsPerDof[globalDofId])
	{
		m_boundaryConditionsPerDof[globalDofId] = true;
		m_boundaryConditionsAsDofIds.push_back(globalDofId);
	}
}

size_t OdeState::getNumBoundaryConditions() const
{
	return m_boundaryConditionsAsDofIds.size();
}

const std::vector<size_t>& OdeState::getBoundaryConditions() const
{
	return m_boundaryConditionsAsDofIds;
}

bool OdeState::isBoundaryCondition(size_t dof) const
{
	return m_boundaryConditionsPerDof[dof];
}

Vector* OdeState::applyBoundaryConditionsToVector(Vector* vector) const
{
	SURGSIM_ASSERT(vector != nullptr && vector->size() >= 0 && static_cast<size_t>(vector->size()) == getNumDof())
			<< "Invalid vector to apply boundary conditions on";

	for (std::vector<size_t>::const_iterator it = getBoundaryConditions().cbegin();
		 it != getBoundaryConditions().cend();
		 ++it)
	{
		(*vector)[*it] = 0.0;
	}

	return vector;
}

void OdeState::applyBoundaryConditionsToMatrix(Matrix* matrix, bool hasCompliance) const
{
	SURGSIM_ASSERT(matrix != nullptr && matrix->rows() >= 0 && matrix->cols() >= 0
				   && static_cast<size_t>(matrix->rows()) == getNumDof()
				   && static_cast<size_t>(matrix->cols()) == getNumDof())
			<< "Invalid matrix to apply boundary conditions on";

	double complianceValue  = 0.0;

	if (hasCompliance)
	{
		complianceValue = 1.0;
	}

	for (std::vector<size_t>::const_iterator it = getBoundaryConditions().cbegin();
		 it != getBoundaryConditions().cend();
		 ++it)
	{
		(*matrix).middleRows(*it, 1).setZero();
		(*matrix).middleCols(*it, 1).setZero();
		(*matrix)(*it, *it) = complianceValue;
	}
}

void OdeState::applyBoundaryConditionsToMatrix(SparseMatrix* matrix, bool hasCompliance) const
{
	SURGSIM_ASSERT(matrix != nullptr && matrix->rows() >= 0 && matrix->cols() >= 0
				   && static_cast<size_t>(matrix->rows()) == getNumDof()
				   && static_cast<size_t>(matrix->cols()) == getNumDof())
			<< "Invalid matrix to apply boundary conditions on";

	double complianceValue  = 0.0;

	if (hasCompliance)
	{
		complianceValue = 1.0;
	}

	for (std::vector<size_t>::const_iterator it = getBoundaryConditions().cbegin();
		 it != getBoundaryConditions().cend();
		 ++it)
	{
		Math::zeroRow(*it, matrix);
		Math::zeroColumn(*it, matrix);
		(*matrix).coeffRef(*it, *it) = complianceValue;
	}
}

bool OdeState::isValid() const
{
	using SurgSim::Math::isValid;

	/// http://steve.hollasch.net/cgindex/coding/ieeefloat.html
	/// We use the IEEE754 standard stipulating that any arithmetic operation with a NaN operand will produce NaN
	/// and any sum of +-INF with a finite number or +-INF will produce +-INF.
	/// Therefore, testing if a vector contains only finite numbers can be achieve easily by summing all the values
	/// and testing if the result is a finite number or not.
	return isValid(getPositions().sum()) && isValid(getVelocities().sum());
}

}; // namespace Math

}; // namespace SurgSim
