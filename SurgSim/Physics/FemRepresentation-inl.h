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

#ifndef SURGSIM_PHYSICS_FEMREPRESENTATION_INL_H
#define SURGSIM_PHYSICS_FEMREPRESENTATION_INL_H

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/FemRepresentationCoordinate.h"

namespace SurgSim
{

namespace Physics
{

template <class MT, class DT, class KT, class ST>
FemRepresentation<MT, DT, KT, ST>::FemRepresentation(const std::string& name) :
	DeformableRepresentation<MT, DT, KT, ST>(name)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;
}

template <class MT, class DT, class KT, class ST>
FemRepresentation<MT, DT, KT, ST>::~FemRepresentation()
{
}

template <class MT, class DT, class KT, class ST>
bool FemRepresentation<MT, DT, KT, ST>::doInitialize()
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

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addFemElement(const std::shared_ptr<FemElement> femElement)
{
	m_femElements.push_back(femElement);
}

template <class MT, class DT, class KT, class ST>
unsigned int FemRepresentation<MT, DT, KT, ST>::getNumFemElements() const
{
	return m_femElements.size();
}

template <class MT, class DT, class KT, class ST>
std::shared_ptr<FemElement> FemRepresentation<MT, DT, KT, ST>::getFemElement(unsigned int femElementId)
{
	SURGSIM_ASSERT(femElementId < getNumFemElements()) << "Invalid femElement id";
	return m_femElements[femElementId];
}

template <class MT, class DT, class KT, class ST>
bool FemRepresentation<MT, DT, KT, ST>::isValidCoordinate(const FemRepresentationCoordinate& coordinate) const
{
	return (coordinate.elementId < m_femElements.size())
		   && m_femElements[coordinate.elementId]->isValidCoordinate(coordinate.naturalCoordinate);
}

template <class MT, class DT, class KT, class ST>
double FemRepresentation<MT, DT, KT, ST>::getTotalMass() const
{
	double mass = 0.0;
	for (auto it = std::begin(m_femElements); it != std::end(m_femElements); it++)
	{
		mass += (*it)->getMass(*m_currentState);
	}
	return mass;
}

template <class MT, class DT, class KT, class ST>
double FemRepresentation<MT, DT, KT, ST>::getRayleighDampingStiffness() const
{
	return m_rayleighDamping.stiffnessCoefficient;
}

template <class MT, class DT, class KT, class ST>
double FemRepresentation<MT, DT, KT, ST>::getRayleighDampingMass() const
{
	return m_rayleighDamping.massCoefficient;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDamping.stiffnessCoefficient = stiffnessCoef;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::setRayleighDampingMass(double massCoef)
{
	m_rayleighDamping.massCoefficient = massCoef;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::beforeUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(getNumFemElements())
			<< "No fem element specified yet, call addFemElement() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof())
			<<	"State has not been initialized yet, call setInitialState() prior to running the simulation";

	// Call the DeformableRepresentation implementation to take care of the OdeSolver setup
	DeformableRepresentation<MT, DT, KT, ST>::beforeUpdate(dt);
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::update(double dt)
{
	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(m_odeSolver != nullptr) <<
										   "Ode solver has not been set yet. Did you call beforeUpdate() ?";
	SURGSIM_ASSERT(m_initialState != nullptr) <<
			"Initial state has not been set yet. Did you call setInitialState() ?";

	// Solve the ode
	m_odeSolver->solve(dt, *m_currentState, m_newState.get());

	// Back up the current state into the previous state (by swapping)
	m_currentState.swap(m_previousState);
	// Make the new state, the current state (by swapping)
	m_currentState.swap(m_newState);
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::afterUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(m_initialState != nullptr) <<
			"Initial state has not been set yet. Did you call setInitialState() ?";

	// Back up the current state into the final state
	*m_finalState = *m_currentState;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::applyCorrection(double dt,
		const Eigen::VectorBlock<SurgSim::Math::Vector>& deltaVelocity)
{
}

template <class MT, class DT, class KT, class ST>
SurgSim::Math::Vector& FemRepresentation<MT, DT, KT, ST>::computeF(const DeformableRepresentationState& state)
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

template <class MT, class DT, class KT, class ST>
const MT& FemRepresentation<MT, DT, KT, ST>::computeM(const DeformableRepresentationState& state)
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

template <class MT, class DT, class KT, class ST>
const DT& FemRepresentation<MT, DT, KT, ST>::computeD(const DeformableRepresentationState& state)
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

template <class MT, class DT, class KT, class ST>
const KT& FemRepresentation<MT, DT, KT, ST>::computeK(const DeformableRepresentationState& state)
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

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::computeFMDK(const DeformableRepresentationState& state,
		SurgSim::Math::Vector** f, MT** M, DT** D, KT** K)
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
	addRayleighDampingForce(&m_f, state, true);

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

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addRayleighDampingForce(
	SurgSim::Math::Vector* force, const DeformableRepresentationState& state,
	bool useGlobalDampingMatrix, bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	// Temporary variables for convenience
	double& rayleighMass = m_rayleighDamping.massCoefficient;
	double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const SurgSim::Math::Vector& v = state.getVelocities();

	// If we have the damping matrix build (D = rayleighMass.M + rayleighStiffness.K), F = -D.v(t)
	if (useGlobalDampingMatrix && (rayleighStiffness != 0.0 || rayleighMass != 0.0))
	{
		*force -= scale * (m_D * v);
	}
	else // Otherwise we unroll the calculation separately on the mass and stiffness components
	{
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
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addFemElementsForce(SurgSim::Math::Vector* force,
		const DeformableRepresentationState& state,
		double scale)
{
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addForce(state, force, scale);
	}
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addGravityForce(SurgSim::Math::Vector* f,
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

#endif // SURGSIM_PHYSICS_FEMREPRESENTATION_INL_H
