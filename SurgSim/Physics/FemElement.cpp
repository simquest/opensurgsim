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

#include "SurgSim/Physics/FemElement.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Geometry.h"

namespace SurgSim
{

namespace Physics
{

FemElement::FemElement() : m_numDofPerNode(0), m_rho(0.0), m_E(0.0), m_nu(0.0)
{}

FemElement::~FemElement()
{}

void FemElement::initialize(const DeformableRepresentationState& state)
{
	SURGSIM_ASSERT(m_rho != 0.0) << "Mass density is not set. Did you call setMassDensity() ?";
	SURGSIM_ASSERT(m_nu != 0.0) << "Poisson ratio is not set. Did you call setPoissonRatio() ?";
	SURGSIM_ASSERT(m_E != 0.0) << "Young modulus is not set. Did you call setYoungModulus() ?";
	SURGSIM_ASSERT(m_rho > 0.0) << "Mass density ("<<m_rho<<") is invalid, it should be positive";
	SURGSIM_ASSERT(m_nu > 0.0 && m_nu < 0.5) << "Poisson ratio ("<<m_nu<<") is invalid, it should be within [0 0.5)";
	SURGSIM_ASSERT(m_E > 0.0) << "Young modulus ("<<m_E<<") is invalid, it should be positive";
}

unsigned int FemElement::getNumDofPerNode() const
{
	return m_numDofPerNode;
}

void FemElement::setNumDofPerNode(unsigned int numDofPerNode)
{
	m_numDofPerNode = numDofPerNode;
}

unsigned int FemElement::getNumNodes() const
{
	return m_nodeIds.size();
}

unsigned int FemElement::getNodeId(unsigned int elementNodeId) const
{
	return m_nodeIds[elementNodeId];
}

const std::vector<unsigned int>& FemElement::getNodeIds() const
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

double FemElement::getMass(const DeformableRepresentationState& state) const
{
	return getVolume(state) * m_rho;
}

bool FemElement::isValidCoordinate(const SurgSim::Math::Vector& naturalCoordinate) const
{
	return (std::abs(naturalCoordinate.sum() - 1.0) < SurgSim::Math::Geometry::ScalarEpsilon)
		&& (naturalCoordinate.size() == getNumNodes())
		&& (-SurgSim::Math::Geometry::ScalarEpsilon <= naturalCoordinate.minCoeff() &&
			naturalCoordinate.maxCoeff() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon);
}

} // namespace Physics

} // namespace SurgSim
