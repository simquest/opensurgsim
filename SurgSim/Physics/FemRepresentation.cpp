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
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/FemRepresentationCoordinate.h"

namespace SurgSim
{

namespace Physics
{

FemRepresentation::FemRepresentation(const std::string& name) :
	DeformableRepresentation(name)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;
}

FemRepresentation::~FemRepresentation()
{
}

bool FemRepresentation::doInitialize()
{
	SURGSIM_ASSERT(m_initialState != nullptr) << "You must set the initial state before calling Initialize";

	// Initialize the FemElements
	for (auto element = std::begin(m_femElements); element != std::end(m_femElements); element++)
	{
		(*element)->initialize(*m_initialState);
	}

	// Allocate the vector m_massPerNode
	if (m_massPerNode.size() == 0 || m_massPerNode.size() < m_initialState->getNumNodes())
	{
		m_massPerNode.resize(m_initialState->getNumNodes());
	}

	// Compute the entries of m_massPerNode from the FemElements mass information
	for (auto element = std::begin(m_femElements); element != std::end(m_femElements); element++)
	{
		double mass = (*element)->getMass(*m_initialState);
		for (auto nodeId = std::begin((*element)->getNodeIds()); nodeId != std::end((*element)->getNodeIds()); nodeId++)
		{
			m_massPerNode[*nodeId] += mass / (*element)->getNumNodes();
		}
	}

	return true;
}

void FemRepresentation::addFemElement(const std::shared_ptr<FemElement> femElement)
{
	m_femElements.push_back(femElement);
}

unsigned int FemRepresentation::getNumFemElements() const
{
	return m_femElements.size();
}

std::shared_ptr<FemElement> FemRepresentation::getFemElement(unsigned int femElementId)
{
	SURGSIM_ASSERT(femElementId < getNumFemElements()) << "Invalid femElement id";
	return m_femElements[femElementId];
}

bool FemRepresentation::isValidCoordinate(const FemRepresentationCoordinate& coordinate) const
{
	return (coordinate.elementId < m_femElements.size())
		   && m_femElements[coordinate.elementId]->isValidCoordinate(coordinate.naturalCoordinate);
}

double FemRepresentation::getTotalMass() const
{
	double mass = 0.0;
	for (auto it = std::begin(m_femElements); it != std::end(m_femElements); it++)
	{
		mass += (*it)->getMass(*m_currentState);
	}
	return mass;
}

double FemRepresentation::getRayleighDampingStiffness() const
{
	return m_rayleighDamping.stiffnessCoefficient;
}

double FemRepresentation::getRayleighDampingMass() const
{
	return m_rayleighDamping.massCoefficient;
}

void FemRepresentation::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDamping.stiffnessCoefficient = stiffnessCoef;
}

void FemRepresentation::setRayleighDampingMass(double massCoef)
{
	m_rayleighDamping.massCoefficient = massCoef;
}

void FemRepresentation::beforeUpdate(double dt)
{
	DeformableRepresentation::beforeUpdate(dt);

	SURGSIM_ASSERT(getNumFemElements())
			<< "No fem element specified yet, call addFemElement() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof())
			<<	"State has not been initialized yet, call setInitialState() prior to running the simulation";
}

SurgSim::Math::Vector& FemRepresentation::computeF(const DeformableRepresentationState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	SurgSim::Math::resizeVector(&m_f, state.getNumDof(), true);

	addGravityForce(&m_f, state);
	addRayleighDampingForce(&m_f, state);
	addFemElementsForce(&m_f, state);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_f[*boundaryCondition] = 0.0;
	}

	return m_f;
}

const SurgSim::Math::Matrix& FemRepresentation::computeM(const DeformableRepresentationState& state)
{
	// Make sure the mass matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_M, state.getNumDof(), state.getNumDof(), true);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addMass(state, &m_M);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_M.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_M.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_M(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_M;
}

const SurgSim::Math::Matrix& FemRepresentation::computeD(const DeformableRepresentationState& state)
{
	const double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const double& rayleighMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_D, state.getNumDof(), state.getNumDof(), true);

	// D += rayleighMass.M
	if (rayleighMass != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addMass(state, &m_D, rayleighMass);
		}
	}

	// D += rayleighStiffness.K
	if (rayleighStiffness != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addStiffness(state, &m_D, rayleighStiffness);
		}
	}

	// D += FemElements damping matrix
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addDamping(state, &m_D);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_D.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_D.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_D(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_D;
}

const SurgSim::Math::Matrix& FemRepresentation::computeK(const DeformableRepresentationState& state)
{
	// Make sure the stiffness matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_K, state.getNumDof(), state.getNumDof(), true);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addStiffness(state, &m_K);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_K.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_K.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_K(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_K;
}

void FemRepresentation::computeFMDK(const DeformableRepresentationState& state, SurgSim::Math::Vector** f,
									SurgSim::Math::Matrix** M, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K)
{
	// Make sure the force vector has been properly allocated and zeroed out
	SurgSim::Math::resizeVector(&m_f, state.getNumDof(), true);

	// Make sure the mass matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_M, state.getNumDof(), state.getNumDof(), true);

	// Make sure the damping matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_D, state.getNumDof(), state.getNumDof(), true);

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	SurgSim::Math::resizeMatrix(&m_K, state.getNumDof(), state.getNumDof(), true);

	// Add all the FemElement contribution to f, M, D, K
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addFMDK(state, &m_f, &m_M, &m_D, &m_K);
	}

	// Add the Rayleigh damping matrix
	if (m_rayleighDamping.massCoefficient)
	{
		m_D += m_M * m_rayleighDamping.massCoefficient;
	}
	if (m_rayleighDamping.stiffnessCoefficient)
	{
		m_D += m_K * m_rayleighDamping.stiffnessCoefficient;
	}

	// Add the gravity to m_f
	addGravityForce(&m_f, state);

	// Add the Rayleigh damping force to m_f
	addRayleighDampingForce(&m_f, state, true, true);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_M.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_M.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_M(*boundaryCondition, *boundaryCondition) = 1e9;

		m_D.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_D.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_D(*boundaryCondition, *boundaryCondition) = 1e9;

		m_K.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_K.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_K(*boundaryCondition, *boundaryCondition) = 1e9;

		m_f[*boundaryCondition] = 0.0;
	}

	*f = &m_f;
	*M = &m_M;
	*D = &m_D;
	*K = &m_K;
}

void FemRepresentation::addRayleighDampingForce(
	SurgSim::Math::Vector* force, const DeformableRepresentationState& state,
	bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	// Temporary variables for convenience
	double& rayleighMass = m_rayleighDamping.massCoefficient;
	double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const SurgSim::Math::Vector& v = state.getVelocities();

	// Rayleigh damping mass: F = -rayleighMass.M.v(t)
	if (rayleighMass != 0.0)
	{
		// If we have the mass matrix, we can compute directly F = -rayleighMass.M.v(t)
		if (useGlobalMassMatrix)
		{
			*force -= (scale * rayleighMass) * (m_M * v);
		}
		else
		{
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
			{
				(*femElement)->addMatVec(state, - scale * rayleighMass, 0.0, 0.0, v, force);
			}
		}
	}

	// Rayleigh damping stiffness: F = - rayleighStiffness.K.v(t)
	// K is not diagonal and links all dof of the N connected nodes
	if (rayleighStiffness != 0.0)
	{
		if (useGlobalStiffnessMatrix)
		{
			*force -= scale * rayleighStiffness * (m_K * v);
		}
		else
		{
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
			{
				(*femElement)->addMatVec(state, 0.0, 0.0, - scale * rayleighStiffness, v, force);
			}
		}
	}
}

void FemRepresentation::addFemElementsForce(SurgSim::Math::Vector* force,
		const DeformableRepresentationState& state,
		double scale)
{
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addForce(state, force, scale);
	}
}

void FemRepresentation::addGravityForce(SurgSim::Math::Vector* f,
		const DeformableRepresentationState& state,
		double scale)
{
	using SurgSim::Math::addSubVector;

	SURGSIM_ASSERT(m_massPerNode.size() == state.getNumNodes()) <<
			"Mass per node has not been properly allocated. Did you call Initialize() ?";

	// Prepare a gravity vector of the proper size
	SurgSim::Math::Vector gravitynD;
	SurgSim::Math::resizeVector(&gravitynD, getNumDofPerNode(), true);
	gravitynD.segment(0, 3) = getGravity();

	if (isGravityEnabled())
	{
		for (unsigned int nodeId = 0; nodeId < state.getNumNodes(); nodeId++)
		{
			// F = mg
			addSubVector(gravitynD * (scale * m_massPerNode[nodeId]), nodeId, getNumDofPerNode(), f);
		}
	}
}

} // namespace Physics

} // namespace SurgSim