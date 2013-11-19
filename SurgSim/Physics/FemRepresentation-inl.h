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

#include <SurgSim/Framework/Assert.h>
#include <SurgSim/Math/Matrix.h>

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
void FemRepresentation<MT, DT, KT, ST>::Initialize()
{
	SURGSIM_ASSERT(this->m_initialState != nullptr) << "You must set the initial state befor ecalling Initialize";

	// Allocate the vector m_massPerNode
	if (m_massPerNode.size() == 0 || m_massPerNode.size() < this->m_initialState->getNumNodes())
	{
		m_massPerNode.resize(this->m_initialState->getNumNodes());
	}

	// Compute the entries of m_massPerNode from the FemElements mass information
	for (auto element = std::begin(m_femElements); element != std::end(m_femElements); element++)
	{
		double mass = (*element)->getMass(*(this->m_initialState));
		for (auto nodeId = std::begin((*element)->getNodeIds()); nodeId != std::end((*element)->getNodeIds()); nodeId++)
		{
			m_massPerNode[*nodeId] += mass / (*element)->getNumNodes();
		}
	}
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
double FemRepresentation<MT, DT, KT, ST>::getTotalMass() const
{
	double mass = 0.0;
	for (auto it = std::begin(m_femElements); it != std::end(m_femElements); it++)
	{
		mass += (*it)->getMass(*(this->m_currentState));
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
	if (! this->isActive())
	{
		return;
	}

	SURGSIM_ASSERT(getNumFemElements()) <<
		"No fem element specified yet, call addFemElement() prior to running the simulation";
	SURGSIM_ASSERT(this->getNumDof()) <<
		"State has not been initialized yet, call setInitialState() prior to running the simulation";

	// Call the DeformableRepresentation implementation to take care of the OdeSolver setup
	DeformableRepresentation<MT, DT, KT, ST>::beforeUpdate(dt);
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::update(double dt)
{
	if (! this->isActive())
	{
		return;
	}

	SURGSIM_ASSERT(this->m_odeSolver != nullptr) <<
		"Ode solver has not been set yet. Did you call beforeUpdate() ?";
	SURGSIM_ASSERT(this->m_initialState != nullptr) <<
		"Initial state has not been set yet. Did you call setInitialState() ?";

	// Solve the ode
	this->m_odeSolver->solve(dt, *(this->m_currentState), this->m_newState.get());

	// Back up the current state into the previous state (by swapping)
	this->m_currentState.swap(this->m_previousState);
	// Make the new state, the current state (by swapping)
	this->m_currentState.swap(this->m_newState);
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::afterUpdate(double dt)
{
	if (! this->isActive())
	{
		return;
	}

	SURGSIM_ASSERT(this->m_initialState != nullptr) <<
		"Initial state has not been set yet. Did you call setInitialState() ?";

	// Back up the current state into the final state
	*(this->m_finalState) = *(this->m_currentState);
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::applyDofCorrection(double dt, const Eigen::VectorBlock<Vector>& block)
{
	if (! this->isActive())
	{
		return;
	}
}

template <class MT, class DT, class KT, class ST>
Vector& FemRepresentation<MT, DT, KT, ST>::computeF(const DeformableRepresentationState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_f, state.getNumDof(), true);

	addGravityForce(&this->m_f, state);
	addRayleighDampingForce(&this->m_f, state);
	addFemElementsForce(&this->m_f, state);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		this->m_f[*boundaryCondition] = 0.0;
	}

	return this->m_f;
}

template <class MT, class DT, class KT, class ST>
const MT& FemRepresentation<MT, DT, KT, ST>::computeM(const DeformableRepresentationState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;

	// Make sure the mass matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_M, state.getNumDof(), state.getNumDof(), true);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addMass(state, &this->m_M);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		this->m_M.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_M.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_M(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return this->m_M;
}

template <class MT, class DT, class KT, class ST>
const DT& FemRepresentation<MT, DT, KT, ST>::computeD(const DeformableRepresentationState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::addSubMatrix;

	const double& rayStiff = m_rayleighDamping.stiffnessCoefficient;
	const double& rayMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_D, state.getNumDof(), state.getNumDof(), true);

	// D += rayMass.M
	if (rayMass != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addMass(state, &this->m_D, rayMass);
		}
	}

	// D += rayStiff.K
	if (rayStiff != 0.0)
	{
		for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
		{
			(*femElement)->addStiffness(state, &this->m_D, rayStiff);
		}
	}

	// D += Springs damping matrix
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addDamping(state, &this->m_D);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		this->m_D.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_D.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_D(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return this->m_D;
}

template <class MT, class DT, class KT, class ST>
const KT& FemRepresentation<MT, DT, KT, ST>::computeK(const DeformableRepresentationState& state)
{
	using SurgSim::Math::addSubMatrix;

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_K, state.getNumDof(), state.getNumDof(), true);

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addStiffness(state, &this->m_K);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		this->m_K.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_K.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_K(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return this->m_K;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::computeFMDK(const DeformableRepresentationState& state,
	Vector** f, MT** M, DT** D, KT** K)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::addSubMatrix;

	// Make sure the force vector has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_f, state.getNumDof(), true);

	// Make sure the mass matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_M, state.getNumDof(), state.getNumDof(), true);

	// Make sure the damping matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_D, state.getNumDof(), state.getNumDof(), true);

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	SurgSim::Math::resize(&this->m_K, state.getNumDof(), state.getNumDof(), true);

	// Add all the FemElement contribution to f, M, D, K
	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addFMDK(state, &this->m_f, &this->m_M, &this->m_D, &this->m_K);
	}

	// Add the Rayleigh damping matrix
	if (m_rayleighDamping.massCoefficient)
	{
		this->m_D += this->m_M * m_rayleighDamping.massCoefficient;
	}
	if (m_rayleighDamping.stiffnessCoefficient)
	{
		this->m_D += this->m_K * m_rayleighDamping.stiffnessCoefficient;
	}

	// Add the gravity to m_f
	addGravityForce(&this->m_f, state);

	// Add the Rayleigh damping force to m_f
	addRayleighDampingForce(&this->m_f, state, true);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		this->m_M.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_M.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_M(*boundaryCondition, *boundaryCondition) = 1e9;

		this->m_D.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_D.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_D(*boundaryCondition, *boundaryCondition) = 1e9;

		this->m_K.block(*boundaryCondition, 0, 1, this->getNumDof()).setZero();
		this->m_K.block(0, *boundaryCondition, this->getNumDof(), 1).setZero();
		this->m_K(*boundaryCondition, *boundaryCondition) = 1e9;

		this->m_f[*boundaryCondition] = 0.0;
	}

	*f = &this->m_f;
	*M = &this->m_M;
	*D = &this->m_D;
	*K = &this->m_K;
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addRayleighDampingForce(
	Vector* force, const DeformableRepresentationState& state,
	bool useGlobalDampingMatrix, bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubMatrix;

	// Temporary variables for convenience
	double& rayMass = m_rayleighDamping.massCoefficient;
	double& rayStiff = m_rayleighDamping.stiffnessCoefficient;
	const Vector& v = state.getVelocities();

	// If we have the damping matrix build (D = rayMass.M + rayStiff.K), F = -D.v(t)
	if (useGlobalDampingMatrix && rayStiff != 0.0 && rayMass != 0.0)
	{
		*force -= scale * (this->m_D * v);
	}
	else // Otherwise we unroll the calculation separately on the mass and stiffness components
	{
		// Rayleigh damping mass: F = -rayMass.M.v(t)
		if (rayMass != 0.0)
		{
			// If we have the mass matrix, we can compute directly F = -rayMass.M.v(t)
			if (useGlobalMassMatrix)
			{
				*force -= (scale * rayMass) * (this->m_M * v);
			}
			else
			{
				// Otherwise, we loop through each fem element to compute its contribution
				for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
				{
					(*femElement)->addMatVec(state, - scale * rayMass, 0.0, 0.0, v, force);
				}
			}
		}

		// Rayleigh damping stiffness: F = - rayStiff.K.v(t)
		// K is not diagonal and links all dof of the N connected nodes
		if (rayStiff != 0.0)
		{
			if (useGlobalStiffnessMatrix)
			{
				*force -= scale * rayStiff * (this->m_K * v);
			}
			else
			{
				// Otherwise, we loop through each fem element to compute its contribution
				for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
				{
					(*femElement)->addMatVec(state, 0.0, 0.0, - scale * rayStiff, v, force);
				}
			}
		}
	}
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addFemElementsForce(Vector *force, const DeformableRepresentationState& state,
															double scale)
{
	using SurgSim::Math::addSubVector;

	for (auto femElement = std::begin(m_femElements); femElement != std::end(m_femElements); femElement++)
	{
		(*femElement)->addForce(state, force, scale);
	}
}

template <class MT, class DT, class KT, class ST>
void FemRepresentation<MT, DT, KT, ST>::addGravityForce(Vector *f, const DeformableRepresentationState& state,
														double scale)
{
	using SurgSim::Math::addSubVector;

	SURGSIM_ASSERT(m_massPerNode.size() == state.getNumNodes()) <<
		"Mass per node has not been properly allocated. Did you call Initialize() ?";

	// Prepare a gravity vector of the proper size
	Vector gravitynD;
	gravitynD.resize(this->getNumDofPerNode());
	gravitynD.setZero();
	gravitynD.segment(0, 3) = this->getGravity();

	if (this->isGravityEnabled())
	{
		for (unsigned int nodeId = 0; nodeId < state.getNumNodes(); nodeId++)
		{
			// F = mg
			addSubVector(gravitynD * (scale * m_massPerNode[nodeId]), nodeId, this->getNumDofPerNode(), f);
		}
	}
}

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_FEMREPRESENTATION_INL_H
