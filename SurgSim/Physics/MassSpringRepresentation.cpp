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
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"

using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;
using SurgSim::Math::SparseMatrix;

namespace SurgSim
{

namespace Physics
{

MassSpringRepresentation::MassSpringRepresentation(const std::string& name) :
	DeformableRepresentation(name)
{
	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;

	// Reminder: m_numDofPerNode is held by DeformableRepresentation
	// but needs to be set by all concrete derived classes
	m_numDofPerNode = 3;
}

MassSpringRepresentation::~MassSpringRepresentation()
{
}

bool MassSpringRepresentation::doInitialize()
{
	SURGSIM_ASSERT(m_initialState != nullptr) << "You must set the initial state before calling Initialize";

	// Initialize the Springs
	for (auto spring : m_springs)
	{
		spring->initialize(*m_initialState);
	}

	return true;
}

void MassSpringRepresentation::addMass(const std::shared_ptr<Mass> mass)
{
	m_masses.push_back(mass);
}

void MassSpringRepresentation::addSpring(const std::shared_ptr<Spring> spring)
{
	m_springs.push_back(spring);
}

size_t MassSpringRepresentation::getNumMasses() const
{
	return m_masses.size();
}

size_t MassSpringRepresentation::getNumSprings() const
{
	return m_springs.size();
}

std::shared_ptr<Mass> MassSpringRepresentation::getMass(size_t nodeId)
{
	SURGSIM_ASSERT(nodeId < getNumMasses()) << "Invalid node id to request a mass from";
	return m_masses[nodeId];
}

std::shared_ptr<Spring> MassSpringRepresentation::getSpring(size_t springId)
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

RepresentationType MassSpringRepresentation::getType() const
{
	return REPRESENTATION_TYPE_MASSSPRING;
}

void MassSpringRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K,
		const SurgSim::Math::Matrix& D)
{
	std::shared_ptr<MassSpringRepresentationLocalization> localization3D =
		std::dynamic_pointer_cast<MassSpringRepresentationLocalization>(localization);
	SURGSIM_ASSERT(localization3D != nullptr) <<
			"Invalid localization type (not a MassSpringRepresentationLocalization)";

	const size_t dofPerNode = getNumDofPerNode();

	const size_t nodeId = localization3D->getLocalNode();
	SURGSIM_ASSERT(nodeId >= 0 && nodeId < getNumMasses()) << "Invalid nodeId " << nodeId <<
			". Valid range is {0.." << getNumMasses() << "}";

	m_externalGeneralizedForce.segment(dofPerNode * nodeId, dofPerNode) += generalizedForce;
	/* Replaced:
	m_externalGeneralizedStiffness.block(dofPerNode * nodeId, dofPerNode * nodeId, dofPerNode, dofPerNode) += K;
	m_externalGeneralizedDamping.block(dofPerNode * nodeId, dofPerNode * nodeId, dofPerNode, dofPerNode) += D;
	*/
	// Replaced the above with:
	Math::addSubMatrix(K, nodeId, nodeId, dofPerNode, dofPerNode, &m_externalGeneralizedStiffness);
	Math::addSubMatrix(D, nodeId, nodeId, dofPerNode, dofPerNode, &m_externalGeneralizedDamping);
	m_hasExternalGeneralizedForce = true;
}

void MassSpringRepresentation::beforeUpdate(double dt)
{
	// Call the DeformableRepresentation implementation
	DeformableRepresentation::beforeUpdate(dt);

	if (! isActive())
	{
		return;
	}

	SURGSIM_ASSERT(3 * getNumMasses() == getNumDof()) <<
			"Mismatch between the number of masses (" << getNumMasses() << ") and the number of dof (" << getNumDof() << ")";
	SURGSIM_ASSERT(getNumMasses()) << "No masses specified yet, call addMass() prior to running the simulation";
	SURGSIM_ASSERT(getNumSprings()) << "No springs specified yet, call addSpring() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof()) <<
								"State has not been initialized yet, call setInitialState() prior to running the simulation";
}

Vector& MassSpringRepresentation::computeF(const SurgSim::Math::OdeState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	addGravityForce(&m_f, state);
	addRayleighDampingForce(&m_f, state);
	addSpringsForce(&m_f, state);

	// Add external generalized force
	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		m_f[*boundaryCondition] = 0.0;
	}

	return m_f;
}

const SparseMatrix& MassSpringRepresentation::computeM(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;

	// Make sure the mass matrix has been properly allocated
	m_M.resize(state.getNumDof(), state.getNumDof());

	/* Replaced:
	Eigen::MatrixBase<const Eigen::SparseMatrix<double>>::DiagonalReturnType diagonal = m_M.diagonal();

	for (size_t massId = 0; massId < getNumMasses(); massId++)
	{
		setSubVector(Vector3d::Ones() * getMass(massId)->getMass(), massId, 3, &diagonal);
	}
	*/
	// Replace the above loop with the following:
	for (size_t massId = 0; massId < getNumMasses(); massId++)
	{
		m_M.coeffRef(3 * massId, 3 * massId) = getMass(massId)->getMass();
		m_M.coeffRef((3 * massId) + 1, (3 * massId) + 1) = getMass(massId)->getMass();
		m_M.coeffRef((3 * massId) + 2, (3 * massId) + 2) = getMass(massId)->getMass();
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		/* Replaced:
		diagonal[*boundaryCondition] = 1e9;
		*/
		// Replaced the above with:
		m_M.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_M;
}

const SparseMatrix& MassSpringRepresentation::computeD(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::addSubMatrix;

	const double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const double& rayleighMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	m_D.resize(state.getNumDof(), state.getNumDof());

	// D += rayleighMass.M
	if (rayleighMass != 0.0)
	{
		for (size_t massId = 0; massId < getNumMasses(); massId++)
		{
			double coef = rayleighMass * getMass(massId)->getMass();
			/* Replaced:
			Eigen::MatrixBase<const Eigen::SparseMatrix<double>>::DiagonalReturnType Ddiagonal = m_D.diagonal();
			setSubVector(Vector3d::Ones() * coef, massId, 3, &Ddiagonal);
			*/
			// Replaced the above with ...
			m_D.coeffRef(3 * massId, 3 * massId) = coef;
			m_D.coeffRef((3 * massId) + 1, (3 * massId) + 1) = coef;
			m_D.coeffRef((3 * massId) + 2, (3 * massId) + 2) = coef;
		}
	}

	// D += rayleighStiffness.K
	if (rayleighStiffness != 0.0)
	{
		for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
		{
			(*spring)->addStiffness(state, &m_D, rayleighStiffness);
		}
	}

	// D += Springs damping matrix
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		(*spring)->addDamping(state, &m_D);
	}

	// Add external generalized damping
	if (m_hasExternalGeneralizedForce)
	{
		m_D += m_externalGeneralizedDamping;
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		/* Replaced:
		m_D.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_D.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_D(*boundaryCondition, *boundaryCondition) = 1e9;
		*/
		Math::zeroRow(*boundaryCondition, &m_D);
		Math::zeroColumn(*boundaryCondition, &m_D);
		m_D.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_D;
}

const SparseMatrix& MassSpringRepresentation::computeK(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::addSubMatrix;

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	m_K.resize(state.getNumDof(), state.getNumDof());

	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		(*spring)->addStiffness(state, &m_K);
	}

	// Add external generalized stiffness
	if (m_hasExternalGeneralizedForce)
	{
		m_K += m_externalGeneralizedStiffness;
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		/* Replaced:
		m_K.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_K.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_K(*boundaryCondition, *boundaryCondition) = 1e9;
		*/
		Math::zeroRow(*boundaryCondition, &m_K);
		Math::zeroColumn(*boundaryCondition, &m_K);
		m_K.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;
	}

	return m_K;
}

void MassSpringRepresentation::computeFMDK(const SurgSim::Math::OdeState& state,
		Vector** f, SparseMatrix** M, SparseMatrix** D, SparseMatrix** K)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::addSubMatrix;

	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	// Make sure the mass matrix has been properly allocated
	m_M.resize(state.getNumDof(), state.getNumDof());

	// Make sure the damping matrix has been properly allocated and zeroed out
	m_D.resize(state.getNumDof(), state.getNumDof());

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	m_K.resize(state.getNumDof(), state.getNumDof());

	// Computes the mass matrix m_M
	computeM(state);

	// Computes the stiffness matrix m_K
	// Add the springs damping matrix to m_D
	// Add the springs force to m_f
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		(*spring)->addFDK(state, &m_f, &m_D, &m_K);
	}

	// Add the Rayleigh damping matrix
	if (m_rayleighDamping.massCoefficient)
	{
		/* Replaced:
		m_D.diagonal() += m_M.diagonal() * m_rayleighDamping.massCoefficient;
		*/
		for (size_t diagonal = 0; diagonal < state.getNumDof(); ++diagonal)
		{
			m_D.coeffRef(diagonal, diagonal) += m_M.coeff(diagonal, diagonal) * m_rayleighDamping.massCoefficient;
		}
	}
	if (m_rayleighDamping.stiffnessCoefficient)
	{
		m_D += m_K * m_rayleighDamping.stiffnessCoefficient;
	}

	// Add the gravity to m_f
	addGravityForce(&m_f, state);

	// Add the Rayleigh damping force to m_f (using the damping matrix)
	addRayleighDampingForce(&m_f, state, true, true);

	// Add external generalized force, stiffness and damping
	if (m_hasExternalGeneralizedForce)
	{
		m_f += m_externalGeneralizedForce;
		m_K += m_externalGeneralizedStiffness;
		m_D += m_externalGeneralizedDamping;
	}

	// Apply boundary conditions globally
	for (auto boundaryCondition = std::begin(state.getBoundaryConditions());
		 boundaryCondition != std::end(state.getBoundaryConditions());
		 boundaryCondition++)
	{
		/* Replaced:
		m_M.diagonal()[*boundaryCondition] = 1e9;

		m_D.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_D.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_D(*boundaryCondition, *boundaryCondition) = 1e9;

		m_K.block(*boundaryCondition, 0, 1, getNumDof()).setZero();
		m_K.block(0, *boundaryCondition, getNumDof(), 1).setZero();
		m_K(*boundaryCondition, *boundaryCondition) = 1e9;
		*/
		m_M.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;

		Math::zeroRow(*boundaryCondition, &m_D);
		Math::zeroColumn(*boundaryCondition, &m_D);
		m_D.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;

		Math::zeroRow(*boundaryCondition, &m_K);
		Math::zeroColumn(*boundaryCondition, &m_K);
		m_K.coeffRef(*boundaryCondition, *boundaryCondition) = 1e9;

		m_f[*boundaryCondition] = 0.0;
	}

	*f = &m_f;
	*M = &m_M;
	*D = &m_D;
	*K = &m_K;
}

void MassSpringRepresentation::addRayleighDampingForce(Vector* force, const SurgSim::Math::OdeState& state,
		bool useGlobalStiffnessMatrix, bool useGlobalMassMatrix, double scale)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubMatrix;

	// Temporary variables for convenience
	double& rayleighMass = m_rayleighDamping.massCoefficient;
	double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const Vector& v = state.getVelocities();

	// Rayleigh damping mass: F = - rayleighMass.M.v(t)
	if (rayleighMass != 0.0)
	{
		// If we have the mass matrix, we can compute directly F = -rayleighMass.M.v(t)
		if (useGlobalMassMatrix)
		{
			// M is diagonal, take advantage of it...
			*force -= (scale * rayleighMass) * (m_M.diagonal().cwiseProduct(v));
		}
		else
		{
			for (size_t nodeID = 0; nodeID < getNumMasses(); nodeID++)
			{
				double mass = getMass(nodeID)->getMass();
				SurgSim::Math::Vector3d f = - scale * rayleighMass * mass * getSubVector(v, nodeID, 3);
				addSubVector(f, nodeID, 3, force);
			}
		}
	}

	// Rayleigh damping stiffness: F = - rayleighStiffness.K.v(t)
	if (rayleighStiffness != 0.0)
	{
		if (useGlobalStiffnessMatrix)
		{
			Math::Vector tempVector = (scale * rayleighStiffness) * (m_K * v);
			*force -= tempVector;
		}
		else
		{
			// Otherwise, we loop through each fem element to compute its contribution
			for (auto spring = std::begin(m_springs); spring != std::end(m_springs); ++spring)
			{
				(*spring)->addMatVec(state, 0.0, - scale * rayleighStiffness, v, force);
			}
		}
	}
}

void MassSpringRepresentation::addSpringsForce(Vector* force, const SurgSim::Math::OdeState& state, double scale)
{
	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		(*spring)->addForce(state, force, scale);
	}
}

void MassSpringRepresentation::addGravityForce(Vector* f, const SurgSim::Math::OdeState& state, double scale)
{
	using SurgSim::Math::addSubVector;

	if (isGravityEnabled())
	{
		for (size_t massId = 0; massId < getNumMasses(); massId++)
		{
			addSubVector(getGravity() * getMass(massId)->getMass(), massId, 3, f);
		}
	}
}

static void transformVectorByBlockOf3(const SurgSim::Math::RigidTransform3d& transform,
									  Vector* x, bool rotationOnly = false)
{
	size_t numNodes = x->size() / 3;
	SURGSIM_ASSERT(static_cast<ptrdiff_t>(numNodes * 3) == x->size()) <<
			"Unexpected number of dof in a MassSpring state vector (not a multiple of 3)";

	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		SurgSim::Math::Vector3d xi = SurgSim::Math::getSubVector(*x, nodeId, 3);
		SurgSim::Math::Vector3d xiTransformed;
		if (rotationOnly)
		{
			xiTransformed = transform.linear() * xi;
		}
		else
		{
			xiTransformed = transform * xi;
		}
		SurgSim::Math::setSubVector(xiTransformed, nodeId, 3, x);
	}
}

void MassSpringRepresentation::transformState(std::shared_ptr<SurgSim::Math::OdeState> state,
		const SurgSim::Math::RigidTransform3d& transform)
{
	transformVectorByBlockOf3(transform, &state->getPositions());
	transformVectorByBlockOf3(transform, &state->getVelocities(), true);
}

} // namespace Physics

} // namespace SurgSim
