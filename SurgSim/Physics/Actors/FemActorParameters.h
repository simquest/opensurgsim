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

#ifndef SURGSIM_PHYSICS_FEMACTORPARAMETERS_H
#define SURGSIM_PHYSICS_FEMACTORPARAMETERS_H

#include <vector>

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Framework/Log.h>

namespace SurgSim 
{

namespace Physics
{

/// The FemActorParameters class defines the physical parameters for all Finite Element Model (1D, 2D, 3D)
class FemActorParameters
{
public:
	/// Default constructor
	FemActorParameters() : m_boundaryConditionsMass(0.0), m_boundaryConditionsInverseMass(0.0),
		m_rho(0.0), m_rayleighDampingMass(0.0), m_rayleighDampingStiffness(0.0),
		m_youngModulus(0.0), m_poissonRatio(0.0), m_isValid(false)
	{
	}

	/// Destructor
	virtual ~FemActorParameters()
	{
	}

	/// Comparison operator (equality test)
	/// \param p A FemActorParameters to compare it to
	/// \return True if the 2 parameters set are equals, False otherwise
	bool operator ==(const FemActorParameters &p) const
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

	/// Comparison operator (difference test)
	/// \param p A FemActorParameters to compare it to
	/// \return False if the 2 parameters set are equals, True otherwise
	bool operator !=(const FemActorParameters &p) const
	{
		return ! (operator ==(p));
	}

	/// Add a boundary condition
	/// \param nodeId The node of the Fem to be fixed
	/// \return True if the boundary condition has been added, False otherwise
	bool addBoundaryCondition(unsigned int nodeId)
	{
		using namespace SurgSim::Framework;

		auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
		if (found == m_boundaryConditions.end())
		{
			m_boundaryConditions.push_back(nodeId);
			return true;
		}
		return false;
	}

	/// Remove a boundary condition
	/// \param nodeId The node of the Fem to be removed from the boundary conditions list
	/// \return True if the boundary condition has been removed, False otherwise
	bool removeBoundaryCondition(unsigned int nodeId)
	{
		auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), nodeId);
		if (found != m_boundaryConditions.end())
		{
			m_boundaryConditions.erase(found);
			return true;
		}
		return false;
	}

	/// Add boundary conditions
	/// \param boundaryConditions The vector of all boundary conditions to be added (nodeIdx)
	/// \return The number of boundary conditions actually added
	unsigned int addBoundaryConditions(const std::vector<unsigned int>& boundaryConditions)
	{
		using namespace SurgSim::Framework;
		unsigned int count = 0;

		for(auto it = boundaryConditions.begin(); it != boundaryConditions.end(); it++)
		{
			auto found = std::find(m_boundaryConditions.begin(), m_boundaryConditions.end(), *it) ;
			if (found == m_boundaryConditions.end())
			{
				m_boundaryConditions.push_back(*it);
				count++;
			}
		}
		return count;
	}

	/// Remove all boundary conditions
	void clearBoundaryConditions()
	{
		m_boundaryConditions.clear();
	}

	/// Get all boundary conditions
	/// \return The vector of all boundary conditions (nodeId)
	const std::vector<unsigned int>& getBoundaryConditions() const 
	{
		return m_boundaryConditions;
	}

	/// Set the boundary condition mass property
	/// \param mass The mass to be assigned to boundary condition nodes
	void setBoundaryConditionMass(double mass)
	{
		m_boundaryConditionsMass = mass;
	}

	/// Get the boundary condition mass property
	/// \return The mass assigned to boundary condition nodes
	double getBoundaryConditionMass() const
	{
		return m_boundaryConditionsMass;
	}

	/// Set the boundary condition inverse mass property
	/// \param invMass The inverse mass to be assigned to boundary condition nodes
	void setBoundaryConditionInverseMass(double invMass)
	{
		m_boundaryConditionsInverseMass = invMass;
	}

	/// Get the boundary condition inverse mass property
	/// \return The inverse mass assigned to boundary condition nodes
	double getBoundaryConditionInverseMass() const
	{
		return m_boundaryConditionsInverseMass;
	}

	/// Set the mass density of the fem
	/// \param rho The mass density (in Kg.m-3)
	void setDensity(double rho)
	{
		m_rho = rho;
		checkValidity();
	}

	/// Get the mass density of the fem
	/// \return The density if it has been provided, 0 otherwise (in Kg.m-3)
	double getDensity() const
	{
		return m_rho;
	}

	/// Set the Rayleigh damping mass parameter
	/// \param massCoef The Rayleigh damping mass parameter (in s-1)
	void setRayleighDampingMass(double massCoef)
	{
		m_rayleighDampingMass = massCoef;
	}

	/// Get the Rayleigh damping mass parameter
	/// \return The Rayleigh damping mass parameter (in s-1)
	double getRayleighDampingMass() const
	{
		return m_rayleighDampingMass;
	}

	/// Set the Rayleigh damping stiffness parameter
	/// \param stiffnessCoef The Rayleigh damping stiffness parameter (in s)
	void setRayleighDampingStiffness(double stiffnessCoef)
	{
		m_rayleighDampingStiffness = stiffnessCoef;
	}

	/// Get the Rayleigh damping stiffness parameter
	/// \return The Rayleigh damping stiffness parameter (in s)
	double getRayleighDampingStiffness() const
	{
		return m_rayleighDampingStiffness;
	}

	/// Set the Young modulus of the material
	/// \param E The Young modulus of the material (in N.m-2)
	void setYoungModulus(double E)
	{
		m_youngModulus = E;
		checkValidity();
	}

	/// Get the material Young modulus
	/// \return The Young modulus of the material (in N.m-2)
	double getYoungModulus() const
	{
		return m_youngModulus;
	}

	/// Set the Poisson ratio of the material
	/// \param nu The Poisson ratio of the material (unitless)
	void setPoissonRatio(double nu)
	{
		m_poissonRatio = nu;
		checkValidity();
	}

	/// Get the material Poisson ratio
	/// \return The Poisson ratio of the material (unitless)
	double getPoissonRatio() const
	{
		return m_poissonRatio;
	}

	/// Test if the the parameters are fully set and ready
	/// \return True if the set of parameters are valid
	/// \note i.e. Mass density and Young modulus strictly positive and Poisson ratio in ]-1, 0.5[
	bool isValid() const
	{
		return m_isValid;
	}

private:
	/// Boundary conditions (vector of node indices to fix)
	std::vector<unsigned int> m_boundaryConditions;

	/// Boundary conditions mass property (useful to build the system matrix)
	double m_boundaryConditionsMass;
	
	/// Boundary conditions mass property (useful to build the system matrix inverse)
	/// Note that m_boundaryConditionsInverseMass can be different than 1.0/m_boundaryConditionsMass
	double m_boundaryConditionsInverseMass;

	/// Density of the object (in Kg.m-3)
	double m_rho;

	/// Rayleigh damping, mass parameter (in s-1)
	double m_rayleighDampingMass;

	/// Rayleigh damping, stiffness parameter (in s)
	double m_rayleighDampingStiffness;

	/// Young modulus (in N.m-2 or Pa or Kg.m-1.s-2)
	double m_youngModulus;

	/// Poisson ratio (unitless)
	/// Theoritically within ]-1, 0.5[  with 0.5 for uncompressible material
	/// In general    within [ 0, 0.5[
	double m_poissonRatio;

	/// Validity of the set of parameters
	bool m_isValid;

	/// Check the validity of the parameters and set the flag m_isValid accordingly
	void checkValidity()
	{
		using namespace SurgSim::Framework;

		// Valid if mass density and Young modulus are strictly positive and Poisson ratio in valid range
		if (m_rho > 0.0 && m_youngModulus > 0.0)
		{
			SURGSIM_ASSERT(m_poissonRatio > -1 && m_poissonRatio < 0.5) << "Poisson ratio is out of range ]-1, 0.5[";
			m_isValid = true;
		}
		else
		{
			m_isValid = false;
		}
	}
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_FEMACTORPARAMETERS_H
