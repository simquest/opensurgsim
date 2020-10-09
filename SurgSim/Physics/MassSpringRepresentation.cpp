// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Physics/MassSpringRepresentation.h"

#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/MassSpring.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpringLocalization.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;
using SurgSim::Math::SparseMatrix;

namespace SurgSim
{

namespace Physics
{

SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::MassSpringRepresentation, MassSpringRepresentation);

MassSpringRepresentation::MassSpringRepresentation(const std::string& name) :
	DeformableRepresentation(name),
	m_mesh(std::make_shared<MassSpring>())
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
	// DeformableRepresentation::doInitialize will
	// 1) assert if initial state is not set
	// 2) transform m_initialState properly with the initial pose
	// => Spring::initialize(m_initialState) is using the correct transformed state
	if (!DeformableRepresentation::doInitialize())
	{
		return false;
	}

	// Initialize the Springs
	for (auto spring : m_springs)
	{
		spring->initialize(*m_initialState);
	}

	// Precompute the sparsity pattern for the global arrays. M is diagonal, the
	// rest need to be calculated.
	m_M.resize(static_cast<Eigen::Index>(getNumDof()), static_cast<Eigen::Index>(getNumDof()));
	m_D.resize(static_cast<Eigen::Index>(getNumDof()), static_cast<Eigen::Index>(getNumDof()));
	m_K.resize(static_cast<Eigen::Index>(getNumDof()), static_cast<Eigen::Index>(getNumDof()));
	for (auto& spring : m_springs)
	{
		Math::Matrix block = Math::Matrix::Zero(getNumDofPerNode(),
												getNumDofPerNode());
		for (auto nodeId1 : spring->getNodeIds())
		{
			for (auto nodeId2 : spring->getNodeIds())
			{
				Math::addSubMatrix(block, static_cast<Eigen::Index>(nodeId1),
					static_cast<Eigen::Index>(nodeId2), &m_D);
				Math::addSubMatrix(block, static_cast<Eigen::Index>(nodeId1),
					static_cast<Eigen::Index>(nodeId2), &m_K);
			}
		}
	}
	m_M.setIdentity();
	m_M.makeCompressed();
	Math::clearMatrix(&m_M);
	m_D.makeCompressed();
	m_K.makeCompressed();

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

size_t MassSpringRepresentation::getNumElements() const
{
	return m_mesh->getNumElements();
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

void MassSpringRepresentation::addExternalGeneralizedForce(std::shared_ptr<Localization> localization,
		const SurgSim::Math::Vector& generalizedForce,
		const SurgSim::Math::Matrix& K,
		const SurgSim::Math::Matrix& D)
{
	auto typedLocalization = std::dynamic_pointer_cast<MassSpringLocalization>(localization);
	SURGSIM_ASSERT(typedLocalization != nullptr) << "Invalid localization type (not a MassSpringLocalization)";

	m_hasExternalGeneralizedForce = true;
	const size_t dofPerNode = getNumDofPerNode();
	if (typedLocalization->getLocalNode().hasValue())
	{
		const size_t nodeId = typedLocalization->getLocalNode().getValue();
		SURGSIM_ASSERT(nodeId >= 0 && nodeId < getNumMasses()) << "Invalid nodeId " << nodeId <<
				". Valid range is {0.." << getNumMasses() << "}";

		m_externalGeneralizedForce.segment(dofPerNode * nodeId, dofPerNode).noalias() += generalizedForce;
		Math::addSubMatrix(K, static_cast<Eigen::Index>(nodeId), static_cast<Eigen::Index>(nodeId),
			&m_externalGeneralizedStiffness);
		Math::addSubMatrix(D, static_cast<Eigen::Index>(nodeId), static_cast<Eigen::Index>(nodeId),
			&m_externalGeneralizedDamping);
	}
	else if (typedLocalization->getLocalPosition().hasValue())
	{
		const auto& nodeIds = m_mesh->getNodeIds(typedLocalization->getLocalPosition().getValue().index);
		const Math::Vector& coordinate = typedLocalization->getLocalPosition().getValue().coordinate;
		size_t index = 0;
		for (const auto& nodeId : nodeIds)
		{
			m_externalGeneralizedForce.segment(dofPerNode * nodeId, dofPerNode).noalias() +=
				generalizedForce * coordinate[index];
			++index;
		}

		if (K.size() != 0 || D.size() != 0)
		{
			size_t index1 = 0;
			for (auto nodeId1 : nodeIds)
			{
				size_t index2 = 0;
				for (auto nodeId2 : nodeIds)
				{
					if (K.size() != 0)
					{
						Math::addSubMatrix(coordinate[index1] * coordinate[index2] * K, nodeId1, nodeId2,
							&m_externalGeneralizedStiffness);
					}
					if (D.size() != 0)
					{
						Math::addSubMatrix(coordinate[index1] * coordinate[index2] * D, nodeId1, nodeId2,
							&m_externalGeneralizedDamping);
					}
					++index2;
				}
				++index1;
			}
		}
		m_externalGeneralizedStiffness.makeCompressed();
		m_externalGeneralizedDamping.makeCompressed();
	}
	else
	{
		SURGSIM_FAILURE() << "MassSpringRepresentation::addExternalGeneralizedForce passed a location without " <<
			"either node or local position.";
	}
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
			"Mismatch between the number of masses (" << getNumMasses() <<
			") and the number of dof (" << getNumDof() << ")";
	SURGSIM_ASSERT(getNumMasses()) << "No masses specified yet, call addMass() prior to running the simulation";
	SURGSIM_ASSERT(getNumSprings()) << "No springs specified yet, call addSpring() prior to running the simulation";
	SURGSIM_ASSERT(getNumDof()) << "State has not been initialized yet, call setInitialState() " <<
								"prior to running the simulation";
}

void MassSpringRepresentation::computeF(const SurgSim::Math::OdeState& state)
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
}

void MassSpringRepresentation::computeM(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;

	// Make sure the mass matrix has been properly allocated
	Math::clearMatrix(&m_M);

	for (Eigen::Index massId = 0; massId < static_cast<Eigen::Index>(getNumMasses()); massId++)
	{
		m_M.coeffRef(3 * massId, 3 * massId) = getMass(massId)->getMass();
		m_M.coeffRef(3 * massId + 1, 3 * massId + 1) = getMass(massId)->getMass();
		m_M.coeffRef(3 * massId + 2, 3 * massId + 2) = getMass(massId)->getMass();
	}
}

void MassSpringRepresentation::computeD(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::setSubVector;

	const double& rayleighStiffness = m_rayleighDamping.stiffnessCoefficient;
	const double& rayleighMass = m_rayleighDamping.massCoefficient;

	// Make sure the damping matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_D);

	// D += rayleighMass.M
	if (rayleighMass != 0.0)
	{
		for (Eigen::Index massId = 0; massId < static_cast<Eigen::Index>(getNumMasses()); massId++)
		{
			double coef = rayleighMass * getMass(massId)->getMass();
			m_D.coeffRef(3 * massId, 3 * massId) = coef;
			m_D.coeffRef(3 * massId + 1, 3 * massId + 1) = coef;
			m_D.coeffRef(3 * massId + 2, 3 * massId + 2) = coef;
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
}

void MassSpringRepresentation::computeK(const SurgSim::Math::OdeState& state)
{
	// Make sure the stiffness matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_K);

	for (auto spring = std::begin(m_springs); spring != std::end(m_springs); spring++)
	{
		(*spring)->addStiffness(state, &m_K);
	}

	// Add external generalized stiffness
	if (m_hasExternalGeneralizedForce)
	{
		m_K += m_externalGeneralizedStiffness;
	}
}

void MassSpringRepresentation::computeFMDK(const SurgSim::Math::OdeState& state)
{
	// Make sure the force vector has been properly allocated and zeroed out
	m_f.setZero(state.getNumDof());

	// Make sure the mass matrix has been properly allocated
	Math::clearMatrix(&m_M);

	// Make sure the damping matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_D);

	// Make sure the stiffness matrix has been properly allocated and zeroed out
	Math::clearMatrix(&m_K);

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
		for (Eigen::Index diagonal = 0; diagonal < static_cast<Eigen::Index>(state.getNumDof()); ++diagonal)
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

std::shared_ptr<Localization> MassSpringRepresentation::createLocalization(
	const SurgSim::DataStructures::Location& location)
{
	auto result = std::make_shared<MassSpringLocalization>(
		std::static_pointer_cast<Physics::Representation>(getSharedPtr()));

	if (location.index.hasValue())
	{
		result->setLocalNode(*location.index);
	}
	else if (location.elementMeshLocalCoordinate.hasValue())
	{
		result->setLocalPosition(location.elementMeshLocalCoordinate.getValue());
	}
	else
	{
		SURGSIM_FAILURE() << "Location passed to MassSpringRepresentation::createLocalization must have an index or " <<
			"elementMeshLocalCoordinate." << "\nFor '" << getFullName() << "', at:\n" << location << std::endl;
	}

	return result;
}

void MassSpringRepresentation::loadMassSpring(const std::string& filename)
{
	auto mesh = std::make_shared<MassSpring>();
	mesh->load(filename);
	setMassSpring(mesh);
}

void MassSpringRepresentation::setMassSpring(std::shared_ptr<Framework::Asset> mesh)
{
	SURGSIM_ASSERT(!isInitialized()) << "The mesh cannot be set after initialization.";
	SURGSIM_ASSERT(mesh != nullptr) << "Mesh for MassSpringRepresentation cannot be a nullptr.";
	auto massSpring = std::dynamic_pointer_cast<MassSpring>(mesh);
	SURGSIM_ASSERT(massSpring != nullptr) <<
		"Mesh for MassSpringRepresentation needs to be a SurgSim::Physics::MassSpring.";
	m_mesh = massSpring;
	auto state = std::make_shared<Math::OdeState>();
	state->setNumDof(getNumDofPerNode(), m_mesh->getNumVertices());
	for (size_t i = 0; i < m_mesh->getNumVertices(); ++i)
	{
		state->getPositions().segment<3>(getNumDofPerNode() * i) = m_mesh->getVertexPosition(i);
		addMass(m_mesh->getMass(i));
	}
	for (const auto& spring : m_mesh->getSprings())
	{
		addSpring(spring);
	}
	for (const auto& boundaryCondition : m_mesh->getBoundaryConditions())
	{
		state->addBoundaryCondition(boundaryCondition);
	}

	// setInitialState: Initialize all the states + apply initialPose if any.
	setInitialState(state);
}

std::shared_ptr<MassSpring> MassSpringRepresentation::getMassSpring() const
{
	return m_mesh;
}

bool MassSpringRepresentation::isValidCoordinate(const SurgSim::Math::Vector& naturalCoordinate) const
{
	return (std::abs(naturalCoordinate.sum() - 1.0) < SurgSim::Math::Geometry::ScalarEpsilon)
		&& (naturalCoordinate.size() >= 0) && (m_mesh->getNumElements() > 0)
		&& (static_cast<size_t>(naturalCoordinate.size()) == m_mesh->getNumNodesPerElement())
		&& (-SurgSim::Math::Geometry::ScalarEpsilon <= naturalCoordinate.minCoeff() &&
			naturalCoordinate.maxCoeff() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon);
}

bool MassSpringRepresentation::isValidCoordinate(const DataStructures::IndexedLocalCoordinate& localCoordinate) const
{
	return (localCoordinate.index < m_mesh->getNumElements()) && isValidCoordinate(localCoordinate.coordinate);
}

Math::Vector3d MassSpringRepresentation::computeCartesianCoordinate(const Math::OdeState& state,
	const DataStructures::IndexedLocalCoordinate& localCoordinate) const
{
	SURGSIM_ASSERT(isValidCoordinate(localCoordinate)) <<
		"MassSpringRepresentation::computeCartesianCoordinate was passed an invalid localCoordinate:\n" <<
		localCoordinate;
	const Math::Vector& positions = state.getPositions();
	const auto& nodeIds = m_mesh->getNodeIds(localCoordinate.index);
	Math::Vector3d result = Math::Vector3d::Zero();
	for (Eigen::Index i = 0; i < localCoordinate.coordinate.size(); ++i)
	{
		result += localCoordinate.coordinate(i) * Math::getSubVector(positions, nodeIds[i], 3).segment<3>(0);
	}
	return result;
}

const std::vector<size_t>& MassSpringRepresentation::getNodeIds(size_t index) const
{
	return m_mesh->getNodeIds(index);
}

bool MassSpringRepresentation::saveMassSpring(const std::string & fileName, double physicsLength) const
{
	return m_mesh->save(fileName, physicsLength);
}

} // namespace Physics

} // namespace SurgSim
