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

#include <SurgSim/Physics/FemRepresentationParameters.h>

namespace SurgSim
{

namespace Physics
{

FemRepresentationParameters::FemRepresentationParameters()
	: m_boundaryConditionsMass(0.0), m_boundaryConditionsInverseMass(0.0),
	m_rho(0.0), m_rayleighDampingMass(0.0), m_rayleighDampingStiffness(0.0),
	m_youngModulus(0.0), m_poissonRatio(0.0), m_isValid(false)
{
}

FemRepresentationParameters::~FemRepresentationParameters()
{
}

bool FemRepresentationParameters::operator ==(const FemRepresentationParameters &p) const
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

bool FemRepresentationParameters::operator !=(const FemRepresentationParameters &p) const
{
	return ! ((*this) == p);
}

bool FemRepresentationParameters::addBoundaryCondition(unsigned int nodeId)
{
	auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
	if (found == m_boundaryConditions.end())
	{
		m_boundaryConditions.push_back(nodeId);
		return true;
	}
	return false;
}

bool FemRepresentationParameters::removeBoundaryCondition(unsigned int nodeId)
{
	auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
	if (found != m_boundaryConditions.end())
	{
		m_boundaryConditions.erase(found);
		return true;
	}
	return false;
}

unsigned int FemRepresentationParameters::addBoundaryConditions(const std::vector<unsigned int>& boundaryConditions)
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

void FemRepresentationParameters::clearBoundaryConditions()
{
	m_boundaryConditions.clear();
}

const std::vector<unsigned int>& FemRepresentationParameters::getBoundaryConditions() const
{
	return m_boundaryConditions;
}

void FemRepresentationParameters::setBoundaryConditionMass(double mass)
{
	m_boundaryConditionsMass = mass;
}

double FemRepresentationParameters::getBoundaryConditionMass() const
{
	return m_boundaryConditionsMass;
}

void FemRepresentationParameters::setBoundaryConditionInverseMass(double invMass)
{
	m_boundaryConditionsInverseMass = invMass;
}

double FemRepresentationParameters::getBoundaryConditionInverseMass() const
{
	return m_boundaryConditionsInverseMass;
}

void FemRepresentationParameters::setDensity(double rho)
{
	m_rho = rho;
	checkValidity();
}

double FemRepresentationParameters::getDensity() const
{
	return m_rho;
}

void FemRepresentationParameters::setRayleighDampingMass(double massCoef)
{
	m_rayleighDampingMass = massCoef;
	checkValidity();
}

double FemRepresentationParameters::getRayleighDampingMass() const
{
	return m_rayleighDampingMass;
}

void FemRepresentationParameters::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDampingStiffness = stiffnessCoef;
	checkValidity();
}

double FemRepresentationParameters::getRayleighDampingStiffness() const
{
	return m_rayleighDampingStiffness;
}

void FemRepresentationParameters::setYoungModulus(double E)
{
	m_youngModulus = E;
	checkValidity();
}

double FemRepresentationParameters::getYoungModulus() const
{
	return m_youngModulus;
}

void FemRepresentationParameters::setPoissonRatio(double nu)
{
	m_poissonRatio = nu;
	checkValidity();
}

double FemRepresentationParameters::getPoissonRatio() const
{
	return m_poissonRatio;
}

bool FemRepresentationParameters::isValid() const
{
	return m_isValid;
}

void FemRepresentationParameters::checkValidity()
{
	// Valid if mass density and Young modulus are strictly positive and
	// Poisson ratio in valid range and Rayleigh parameters positives or nulls
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
