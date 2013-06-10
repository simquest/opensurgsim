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

#include <SurgSim/Physics/FemActorParameters.h>

namespace SurgSim
{

namespace Physics
{

FemActorParameters::FemActorParameters()
	: m_boundaryConditionsMass(0.0), m_boundaryConditionsInverseMass(0.0),
	m_rho(0.0), m_rayleighDampingMass(0.0), m_rayleighDampingStiffness(0.0),
	m_youngModulus(0.0), m_poissonRatio(0.0), m_isValid(false)
{
}

FemActorParameters::~FemActorParameters()
{
}

bool FemActorParameters::operator ==(const FemActorParameters &p) const
{
	return ( m_boundaryConditions == p.m_boundaryConditions &&
		m_boundaryConditionsMass == p.m_boundaryConditionsMass &&
		m_boundaryConditionsInverseMass == p.m_boundaryConditionsInverseMass &&
		m_rho == p.m_rho &&
		m_rayleighDampingMass == p.m_rayleighDampingMass &&
		m_rayleighDampingStiffness == p.m_rayleighDampingStiffness &&
		m_youngModulus == p.m_youngModulus &&
		m_poissonRatio == p.m_poissonRatio &&
		m_isValid == p.m_isValid);
}

bool FemActorParameters::operator !=(const FemActorParameters &p) const
{
	return ! ((*this) == p);
}

bool FemActorParameters::addBoundaryCondition(unsigned int nodeId)
{
	auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
	if (found == m_boundaryConditions.end())
	{
		m_boundaryConditions.push_back(nodeId);
		return true;
	}
	return false;
}

bool FemActorParameters::removeBoundaryCondition(unsigned int nodeId)
{
	auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
	if (found != m_boundaryConditions.end())
	{
		m_boundaryConditions.erase(found);
		return true;
	}
	return false;
}

unsigned int FemActorParameters::addBoundaryConditions(const std::vector<unsigned int>& boundaryConditions)
{
	unsigned int count = 0u;

	for(auto it = boundaryConditions.begin(); it != boundaryConditions.end(); it++)
	{
		auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), *it);
		if (found == m_boundaryConditions.end())
		{
			m_boundaryConditions.push_back(*it);
			count++;
		}
	}
	return count;
}

void FemActorParameters::clearBoundaryConditions()
{
	m_boundaryConditions.clear();
}

const std::vector<unsigned int>& FemActorParameters::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

void FemActorParameters::setBoundaryConditionMass(double mass)
{
	m_boundaryConditionsMass = mass;
}

double FemActorParameters::getBoundaryConditionMass() const
{
	return m_boundaryConditionsMass;
}

void FemActorParameters::setBoundaryConditionInverseMass(double invMass)
{
	m_boundaryConditionsInverseMass = invMass;
}

double FemActorParameters::getBoundaryConditionInverseMass() const
{
	return m_boundaryConditionsInverseMass;
}

void FemActorParameters::setDensity(double rho)
{
	m_rho = rho;
	checkValidity();
}

double FemActorParameters::getDensity() const
{
	return m_rho;
}

void FemActorParameters::setRayleighDampingMass(double massCoef)
{
	m_rayleighDampingMass = massCoef;
	checkValidity();
}

double FemActorParameters::getRayleighDampingMass() const
{
	return m_rayleighDampingMass;
}

void FemActorParameters::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDampingStiffness = stiffnessCoef;
	checkValidity();
}

double FemActorParameters::getRayleighDampingStiffness() const
{
	return m_rayleighDampingStiffness;
}

void FemActorParameters::setYoungModulus(double E)
{
	m_youngModulus = E;
	checkValidity();
}

double FemActorParameters::getYoungModulus() const
{
	return m_youngModulus;
}

void FemActorParameters::setPoissonRatio(double nu)
{
	m_poissonRatio = nu;
	checkValidity();
}

double FemActorParameters::getPoissonRatio() const
{
	return m_poissonRatio;
}

bool FemActorParameters::isValid() const
{
	return m_isValid;
}

void FemActorParameters::checkValidity()
{
	// Valid if mass density and Young modulus are strictly positive and Poisson ratio in valid range
	if (m_rho > 0.0 && m_youngModulus > 0.0 && m_poissonRatio > -1.0 && m_poissonRatio < 0.5 &&
		m_rayleighDampingMass >= 0.0 && m_rayleighDampingStiffness >= 0.0)
	{
		m_isValid = true;
	}
	else
	{
		m_isValid = false;
	}
}

}; // namespace Physics

}; // namespace SurgSim
