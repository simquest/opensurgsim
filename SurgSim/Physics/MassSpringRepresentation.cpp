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

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Framework/Assert.h>

#include <SurgSim/Physics/MassSpringRepresentation.h>

#include <SurgSim/Math/OdeSolverEulerExplicit.h>
#include <SurgSim/Math/OdeSolverEulerExplicitModified.h>
#include <SurgSim/Math/OdeSolverEulerImplicit.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::Math::ExplicitEuler;
using SurgSim::Math::ModifiedExplicitEuler;
using SurgSim::Math::ImplicitEuler;

using SurgSim::Math::Vector3d;
using SurgSim::Math::setSubVector;
using SurgSim::Math::getSubVector;
using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubMatrix;
using SurgSim::Math::addSubMatrix;

namespace SurgSim
{

namespace Physics
{

MassSpringRepresentation::MassSpringRepresentation(const std::string& name) :
	DeformableRepresentation(name),
	m_integrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;
}

MassSpringRepresentation::~MassSpringRepresentation()
{
}

void MassSpringRepresentation::addMass(const std::shared_ptr<Mass> mass)
{
	m_masses.push_back(mass);
}

void MassSpringRepresentation::addSpring(const std::shared_ptr<Spring> spring)
{
	m_springs.push_back(spring);
}

unsigned int MassSpringRepresentation::getNumMasses() const
{
	return m_masses.size();
}

unsigned int MassSpringRepresentation::getNumSprings() const
{
	return m_springs.size();
}

std::shared_ptr<Mass> MassSpringRepresentation::getMass(unsigned int nodeId)
{
	SURGSIM_ASSERT(nodeId < getNumMasses()) << "Invalid node id to request a mass from";
	return m_masses[nodeId];
}

std::shared_ptr<Spring> MassSpringRepresentation::getSpring(unsigned int springId)
{
	SURGSIM_ASSERT(springId < getNumSprings()) << "Invalid spring id";
	return m_springs[springId];
}

double MassSpringRepresentation::getTotalMass() const
{
	double mass = 0.0;
	for (auto it = std::begin(m_masses); it != std::end(m_masses); it++)
	{
		mass += (*it)->getMass();
	}
	return mass;
}

double MassSpringRepresentation::getRayleighDampingStiffness() const
{
	return m_rayleighDamping.stiffnessCoefficient;
}

double MassSpringRepresentation::getRayleighDampingMass() const
{
	return m_rayleighDamping.massCoefficient;
}

void MassSpringRepresentation::setRayleighDampingStiffness(double stiffnessCoef)
{
	m_rayleighDamping.stiffnessCoefficient = stiffnessCoef;
}

void MassSpringRepresentation::setRayleighDampingMass(double massCoef)
{
	m_rayleighDamping.massCoefficient = massCoef;
}

void MassSpringRepresentation::setIntegrationScheme(IntegrationScheme integrationScheme)
{
	if (m_integrationScheme != integrationScheme )
	{
		// Sets the integration scheme variable
		m_integrationScheme = integrationScheme;
		// Reset the ode solver (will be re-allocated with the proper ode solver on the next call to beforeUpdate())
		m_odeSolver.reset();
	}
}

MassSpringRepresentation::IntegrationScheme MassSpringRepresentation::getIntegrationScheme() const
{
	return m_integrationScheme;
}

RepresentationType MassSpringRepresentation::getType() const
{
	return REPRESENTATION_TYPE_MASSSPRING;
}

void MassSpringRepresentation::beforeUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(3 * getNumMasses() == getNumDof()) <<
		"Mismatch between the number of masses ("<<getNumMasses()<<") and the number of dof ("<<getNumDof()<<")";
	SURGSIM_ASSERT(getNumMasses()) << "No masses specified yet, call addMass() prior to running the simulation";
	SURGSIM_ASSERT(getNumSprings()) << "No springs specified yet, call addSpring() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof()) <<
		"State has not been initialized yet, call setInitialState() prior to running the simulation";

	if (! m_odeSolver)
	{
		switch(m_integrationScheme)
		{
		case INTEGRATIONSCHEME_EXPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ExplicitEuler<DeformableRepresentationState, DiagonalMatrix, Matrix, Matrix, Matrix>>
				(*this, *m_initialState.get());
			break;
		case INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ModifiedExplicitEuler<DeformableRepresentationState, DiagonalMatrix, Matrix, Matrix, Matrix>>
				(*this, *m_initialState.get());
			break;
		case INTEGRATIONSCHEME_IMPLICIT_EULER:
			m_odeSolver = std::make_shared
				<ImplicitEuler<DeformableRepresentationState, DiagonalMatrix, Matrix, Matrix, Matrix>>
				(*this, *m_initialState.get());
			break;
		default:
			SURGSIM_ASSERT(m_odeSolver) <<
				"Ode solver (integration scheme) not initialized yet, call setIntegrationScheme()";
			break;
		}
	}
}

void MassSpringRepresentation::update(double dt)
{
	if (! isActive())
	{
		return;
	}

	m_odeSolver->solve(dt, *m_currentState, m_newState.get());

	// Back up the current state into the previous state (by swapping)
	m_currentState.swap(m_previousState);
	// Make the new state, the current state (by swapping)
	m_currentState.swap(m_newState);
}

void MassSpringRepresentation::afterUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	// Back up the current state into the final state
	*m_finalState = *m_currentState;
}

void MassSpringRepresentation::applyDofCorrection(double dt, const Eigen::VectorBlock<Vector>& block)
{
	if (! isActive())
	{
		return;
	}
}

Vector& MassSpringRepresentation::computeF(const DeformableRepresentationState& state)
{
	// Make sure the force vector has been properly allocated
	if (! m_f.size() && state.getNumDof())
	{
		m_f.resize(state.getNumDof());
	}

	m_f.setZero();
	addGravityForce(&m_f, state);
	addRayleighDampingForce(&m_f, state);
	addSpringsForce(&m_f, state);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		m_f[*boundaryCondition] = 0.0;
	}

	return m_f;
}

const DiagonalMatrix& MassSpringRepresentation::computeM(const DeformableRepresentationState& state)
{
	// Make sure the mass matrix has been properly allocated
	if (! m_M.rows() && state.getNumDof())
	{
		m_M.resize(state.getNumDof());
	}

	DiagonalMatrix::DiagonalVectorType& diagonal = m_M.diagonal();

	for (unsigned int massId = 0; massId < getNumMasses(); massId++)
	{
		setSubVector(Vector3d::Ones() * getMass(massId)->getMass(), massId, 3, &diagonal);
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		diagonal[*boundaryCondition] = 1e9;
	}

	return m_M;
}

const Matrix& MassSpringRepresentation::computeD(const DeformableRepresentationState& state)
{
	const double& rayStiff = m_rayleighDamping.stiffnessCoefficient;
	const double& rayMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated
	if (! m_D.rows() && state.getNumDof())
	{
		m_D.resize(state.getNumDof(), state.getNumDof());
	}

	m_D.setZero();

	// D += rayMass.M
	if (rayMass)
	{
		for (unsigned int massId = 0; massId < getNumMasses(); massId++)
		{
			double coef = rayMass * getMass(massId)->getMass();
			Matrix::DiagonalReturnType Ddiagonal = m_D.diagonal();
			setSubVector(Vector3d::Ones() * coef, massId, 3, &Ddiagonal);
		}
	}

	// D += rayStiff.K
	if (rayStiff)
	{
		for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
		{
			addSubMatrix(rayStiff * (*spring)->computeStiffness(state), (*spring)->getNodeIds(), 3, &m_D);
		}
	}

	// D += Springs damping matrix
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		addSubMatrix((*spring)->computeDamping(state), (*spring)->getNodeIds(), 3, &m_D);
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

const Matrix& MassSpringRepresentation::computeK(const DeformableRepresentationState& state)
{
	// Make sure the stiffness matrix has been properly allocated
	if (! m_K.rows() && state.getNumDof())
	{
		m_K.resize(state.getNumDof(), state.getNumDof());
	}

	m_K.setZero();

	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		addSubMatrix((*spring)->computeStiffness(state), (*spring)->getNodeIds(), 3, &m_K);
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

void MassSpringRepresentation::computeFMDK(const DeformableRepresentationState& state,
	Vector** f, DiagonalMatrix** M, Matrix** D, Matrix** K)
{
	// Make sure the force vector has been properly allocated
	if (! m_f.size() && state.getNumDof())
	{
		m_f.resize(state.getNumDof());
	}
	m_f.setZero();

	// Make sure the mass matrix has been properly allocated
	// No need to zero out the mass matrix, it will be directly set
	if (! m_M.rows() && state.getNumDof())
	{
		m_M.resize(state.getNumDof());
	}

	// Make sure the damping matrix has been properly allocated
	if (! m_D.rows() && state.getNumDof())
	{
		m_D.resize(state.getNumDof(), state.getNumDof());
	}
	m_D.setZero();

	// Make sure the stiffness matrix has been properly allocated
	if (! m_K.rows() && state.getNumDof())
	{
		m_K.resize(state.getNumDof(), state.getNumDof());
	}
	m_K.setZero();

	// Computes the mass matrix m_M
	computeM(state);

	// Computes the stiffness matrix m_K
	// Add the springs damping matrix to m_D
	// Add the springs force to m_f
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		Vector* f;
		Matrix* D;
		Matrix* K;
		(*spring)->computeFDK(state, &f, &D, &K);

		addSubMatrix(*K, (*spring)->getNodeIds(), 3, &m_K);
		addSubMatrix(*D, (*spring)->getNodeIds(), 3, &m_D);
		addSubVector(*f, (*spring)->getNodeIds(), 3, &m_f);
	}

	// Add the Rayleigh damping matrix
	if (m_rayleighDamping.massCoefficient)
	{
		m_D.diagonal() += m_M.diagonal() * m_rayleighDamping.massCoefficient;
	}
	if (m_rayleighDamping.stiffnessCoefficient)
	{
		m_D += m_K * m_rayleighDamping.stiffnessCoefficient;
	}

	// Add the gravity to m_f
	addGravityForce(&m_f, state);

	// Add the Rayleigh damping force to m_f
	addRayleighDampingForce(&m_f, state, false);

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		boundaryCondition != std::end(state.getBoundaryConditions());
		boundaryCondition++)
	{
		m_M.diagonal()[*boundaryCondition] = 1e9;

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

void MassSpringRepresentation::addRayleighDampingForce(Vector* force, const DeformableRepresentationState& state,
	bool useGlobalStiffnessMatrix, double scale)
{
	// Temporary variables for convenience
	double& rayMass = m_rayleighDamping.massCoefficient;
	double& rayStiff = m_rayleighDamping.stiffnessCoefficient;
	const Vector& v = state.getVelocities();

	// Rayleigh damping mass: F = - rayMass.M.v(t)
	// M is diagonal, so this calculation can be done node per node
	if (rayMass)
	{
		for (unsigned int nodeID = 0; nodeID < getNumMasses(); nodeID++)
		{
			double mass = getMass(nodeID)->getMass();
			Vector3d f = - scale * rayMass * mass * getSubVector(v, nodeID, 3);
			addSubVector(f, nodeID, 3, force);
		}
	}

	// Rayleigh damping stiffness: F = - rayStiff.K.v(t)
	// K is not diagonal and links all dof of the N connected nodes
	if (rayStiff)
	{
		if (useGlobalStiffnessMatrix)
		{
			*force -= scale * rayStiff * (m_K * v);
		}
		else
		{
			// Loop through each spring to compute its contribution
			for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
			{
				const Matrix& springK = (*spring)->computeStiffness(state);
				const unsigned int springNumNodes = (*spring)->getNumNodes();

				// This spring contribution is F = - rayStiff.springK.v (with v a subset of the global velocity vector)
				// We do this matrix-vector product node by node
				for (unsigned int springNodeIdRow = 0; springNodeIdRow < springNumNodes; springNodeIdRow++)
				{
					unsigned int nodeIdRow = (*spring)->getNodeId(springNodeIdRow);

					for (unsigned int springNodeIdCol = 0; springNodeIdCol < springNumNodes; springNodeIdCol++)
					{
						unsigned int nodeIdCol = (*spring)->getNodeId(springNodeIdCol);

						auto springKij = getSubMatrix(springK, springNodeIdRow, springNodeIdCol, 3, 3);
						auto vj        = getSubVector(v, nodeIdCol, 3);
						addSubVector(- scale * rayStiff * (springKij * vj), nodeIdRow, 3, force);
					}
				}
			}
		}
	}
}

void MassSpringRepresentation::addSpringsForce(Vector *force, const DeformableRepresentationState& state, double scale)
{
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		addSubVector((*spring)->computeForce(state), (*spring)->getNodeIds(), 3, force);
	}
}

void MassSpringRepresentation::addGravityForce(Vector *f, const DeformableRepresentationState& state, double scale)
{
	if (isGravityEnabled())
	{
		for (unsigned int massId = 0; massId < getNumMasses(); massId++)
		{
			addSubVector(getGravity() * getMass(massId)->getMass(), massId, 3, f);
		}
	}
}

static void transformVectorByBlockOf3(const RigidTransform3d& transform, Vector* x, bool rotationOnly = false)
{
	unsigned int numNodes = x->size() / 3;
	SURGSIM_ASSERT(numNodes * 3 == x->size()) <<
		"Unexpected number of dof in a MassSpring state vector (not a multiple of 3)";

	for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
	{
		Vector3d xi = getSubVector(*x, nodeId, 3);
		Vector3d xiTransformed;
		if (rotationOnly)
		{
			xiTransformed = transform.linear() * xi;
		}
		else
		{
			xiTransformed = transform * xi;
		}
		setSubVector(xiTransformed, nodeId, 3, x);
	}
}

void MassSpringRepresentation::transformState(std::shared_ptr<DeformableRepresentationState> state,
	const RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
	transformVectorByBlockOf3(transform, &state->getAccelerations(), true);
}

} // namespace Physics

} // namespace SurgSim
