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
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/FemElement.h"

namespace SurgSim
{

namespace Physics
{

FemElement::FemElement() : m_numDofPerNode(0), m_rho(0.0), m_E(0.0), m_nu(0.0), m_initializedFMDK(false),
	m_useDamping(false)
{}

FemElement::~FemElement()
{}

void FemElement::initialize(const SurgSim::Math::OdeState& state)
{
	initializeFMDK();
	SURGSIM_ASSERT(m_rho != 0.0) << "Mass density is not set. Did you call setMassDensity() ?";
	SURGSIM_ASSERT(m_nu != 0.0) << "Poisson ratio is not set. Did you call setPoissonRatio() ?";
	SURGSIM_ASSERT(m_E != 0.0) << "Young modulus is not set. Did you call setYoungModulus() ?";
	SURGSIM_ASSERT(m_rho > 0.0) << "Mass density ("<<m_rho<<") is invalid, it should be positive";
	SURGSIM_ASSERT(m_nu > 0.0 && m_nu < 0.5) << "Poisson ratio ("<<m_nu<<") is invalid, it should be within [0 0.5)";
	SURGSIM_ASSERT(m_E > 0.0) << "Young modulus ("<<m_E<<") is invalid, it should be positive";
}

FemElement::FactoryType& FemElement::getFactory()
{
	static FactoryType factory;
	return factory;
}

size_t FemElement::getNumDofPerNode() const
{
	return m_numDofPerNode;
}

void FemElement::setNumDofPerNode(size_t numDofPerNode)
{
	m_numDofPerNode = numDofPerNode;
}

size_t FemElement::getNumNodes() const
{
	return m_nodeIds.size();
}

size_t FemElement::getNodeId(size_t elementNodeId) const
{
	return m_nodeIds[elementNodeId];
}

const std::vector<size_t>& FemElement::getNodeIds() const
{
	return m_nodeIds;
}

void FemElement::setYoungModulus(double E)
{
	m_E = E;
}

double FemElement::getYoungModulus() const
{
	return m_E;
}

void FemElement::setPoissonRatio(double nu)
{
	m_nu = nu;
}

double FemElement::getPoissonRatio() const
{
	return m_nu;
}

void FemElement::setMassDensity(double rho)
{
	m_rho = rho;
}

double FemElement::getMassDensity() const
{
	return m_rho;
}

double FemElement::getMass(const SurgSim::Math::OdeState& state) const
{
	return getVolume(state) * m_rho;
}

void FemElement::addForce(SurgSim::Math::Vector* F, double scale) const
{
	if (std::abs(scale - 1.0) < 1e-10)
	{
		Math::addSubVector(m_f, m_nodeIds, m_numDofPerNode, F);
	}
	else
	{
		Math::addSubVector(m_f * scale, m_nodeIds, m_numDofPerNode, F);
	}
}

void FemElement::addMass(SurgSim::Math::SparseMatrix* M, double scale) const
{
	if (std::abs(scale - 1.0) < 1e-10)
	{
		assembleMatrixBlocks(m_M, m_nodeIds, static_cast<int>(m_numDofPerNode), M, false);
	}
	else
	{
		assembleMatrixBlocks(m_M * scale, m_nodeIds, static_cast<int>(m_numDofPerNode), M, false);
	}
}

void FemElement::addDamping(SurgSim::Math::SparseMatrix* D, double scale) const
{
	if (m_useDamping)
	{
		if (std::abs(scale - 1.0) < 1e-10)
		{
			assembleMatrixBlocks(m_D, m_nodeIds, static_cast<int>(m_numDofPerNode), D, false);
		}
		else
		{
			assembleMatrixBlocks(m_D * scale, m_nodeIds, static_cast<int>(m_numDofPerNode), D, false);
		}
	}
}

void FemElement::addStiffness(SurgSim::Math::SparseMatrix* K, double scale) const
{
	if (std::abs(scale - 1.0) < 1e-10)
	{
		assembleMatrixBlocks(m_K, m_nodeIds, static_cast<int>(m_numDofPerNode), K, false);
	}
	else
	{
		assembleMatrixBlocks(m_K * scale, m_nodeIds, static_cast<int>(m_numDofPerNode), K, false);
	}
}

void FemElement::addFMDK(SurgSim::Math::Vector* F,
					 SurgSim::Math::SparseMatrix* M,
					 SurgSim::Math::SparseMatrix* D,
					 SurgSim::Math::SparseMatrix* K) const
{
	Math::addSubVector(m_f, m_nodeIds, m_numDofPerNode, F);
	assembleMatrixBlocks(m_M, m_nodeIds, static_cast<int>(m_numDofPerNode), M, false);
	if (m_useDamping)
	{
		assembleMatrixBlocks(m_D, m_nodeIds, static_cast<int>(m_numDofPerNode), D, false);
	}
	assembleMatrixBlocks(m_K, m_nodeIds, static_cast<int>(m_numDofPerNode), K, false);
}

bool FemElement::isValidCoordinate(const SurgSim::Math::Vector& naturalCoordinate) const
{
	return (std::abs(naturalCoordinate.sum() - 1.0) < SurgSim::Math::Geometry::ScalarEpsilon)
		&& (naturalCoordinate.size() >= 0)
		&& (static_cast<size_t>(naturalCoordinate.size()) == getNumNodes())
		&& (-SurgSim::Math::Geometry::ScalarEpsilon <= naturalCoordinate.minCoeff() &&
			naturalCoordinate.maxCoeff() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon);
}

void FemElement::updateFMDK(const Math::OdeState& state, int options)
{
	doUpdateFMDK(state, options);
}

void FemElement::initializeFMDK()
{
	if (!m_initializedFMDK)
	{
		m_initializedFMDK = true;
		doInitializeFMDK();
	}
}

void FemElement::doInitializeFMDK()
{
	size_t size = getNumNodes() * getNumDofPerNode();
	m_f.setZero(size);
	m_M.setZero(size, size);
	if (m_useDamping)
	{
		m_D.setZero(size, size);
	}
	m_K.setZero(size, size);
}

} // namespace Physics

} // namespace SurgSim
