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

#include <SurgSim/Physics/MassSpringRepresentation.h>
#include <SurgSim/DataStructures/Vertex.h>
#include <SurgSim/DataStructures/MeshElement.h>

using SurgSim::DataStructures::Vertex;
using SurgSim::DataStructures::MeshElement;

namespace SurgSim
{

namespace Physics
{

MassSpringRepresentation::MassSpringRepresentation(const std::string& name) :
Representation(name), m_integrationScheme(MassSpringRepresentation::INTEGRATIONSCHEME_EXPLICIT_EULER)
{
	m_identityPose.setIdentity();
	m_initialPose.setIdentity();

	m_rayleighDamping.massCoefficient = 0.0;
	m_rayleighDamping.stiffnessCoefficient = 0.0;
}

MassSpringRepresentation::~MassSpringRepresentation()
{

}

unsigned int MassSpringRepresentation::getNumMasses() const
{
	return m_finalState.getNumVertices();
}

unsigned int MassSpringRepresentation::getNumSprings() const
{
	return m_finalState.getNumEdges();
}

MassParameter& MassSpringRepresentation::getMassParameter(unsigned int nodeId)
{
	SURGSIM_ASSERT(nodeId < getNumMasses()) << "Invalid node id to request a mass from";
	return m_finalState.getVertex(nodeId).data;
}

LinearSpringParameter& MassSpringRepresentation::getSpringParameter(unsigned int springId)
{
	SURGSIM_ASSERT(springId < getNumSprings()) << "Invalid spring id";
	return m_finalState.getEdge(springId).data;
}

const TetrahedronMesh<MassParameter, LinearSpringParameter, void, void>&
	MassSpringRepresentation::getFinalState() const
{
	return m_finalState;
}

const TetrahedronMesh<MassParameter, LinearSpringParameter, void, void>&
	MassSpringRepresentation::getInitialState() const
{
	return m_initialState;
}

double MassSpringRepresentation::getTotalMass() const
{
	double mass = 0.0;
	const std::vector<Vertex<MassParameter>> &vertices = m_finalState.getVertices();
	for (std::vector<Vertex<MassParameter>>::const_iterator it = vertices.begin(); it != vertices.end(); it++)
	{
		mass += it->data.getMass();
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

void MassSpringRepresentation::addBoundaryCondition(unsigned int nodeId)
{
	m_boundaryConditions.insert(nodeId);
}

unsigned int MassSpringRepresentation::getBoundaryCondition(unsigned int boundaryConditionId) const
{
	SURGSIM_ASSERT(boundaryConditionId >= 0u && boundaryConditionId < getNumBoundaryConditions()) \
		<< "Invalid boundary condition id " << boundaryConditionId;
	std::set<unsigned int>::const_iterator it = m_boundaryConditions.begin();
	for (unsigned int i = 0; i < boundaryConditionId; i++) it++;
	return *it;
}

unsigned int MassSpringRepresentation::getNumBoundaryConditions() const
{
	return static_cast<unsigned int>(m_boundaryConditions.size());
}

void MassSpringRepresentation::setIntegrationScheme(IntegrationScheme integrationScheme)
{
	m_integrationScheme = integrationScheme;
}

MassSpringRepresentation::IntegrationScheme MassSpringRepresentation::getIntegrationScheme() const
{
	return m_integrationScheme;
}

void MassSpringRepresentation::init1D(const Vector3d extremities[2], unsigned int numNodesPerDim[1],
	double totalMass, double springStiffness, double springDamping)
{
	SURGSIM_ASSERT(numNodesPerDim[0] > 0) << "Number of nodes incorrect: " << numNodesPerDim[0];

	// Allocate all Eigen data structures and all states
	allocate(numNodesPerDim[0] * 3);

	// Initialize the initial mesh state
	{
		// Initialize the nodes position (transforming them by m_initialPose), velocity and mass
		Vector3d delta = (extremities[1] - extremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
		for (unsigned int massId = 0; massId < numNodesPerDim[0]; massId++)
		{
			MassParameter mass;
			mass.setMass(totalMass / static_cast<double>(numNodesPerDim[0]));
			mass.setVelocity(Vector3d::Zero());
			Vertex<MassParameter> vertex( m_initialPose * (extremities[0] + massId * delta), mass);
			m_initialState.addVertex(vertex);
		}

		// Initialize the springs
		for (unsigned int massId = 0; massId < numNodesPerDim[0] - 1; massId++)
		{
			LinearSpringParameter spring;
			std::array<unsigned int, 2> element;

			element[0] = massId;
			element[1] = massId + 1;
			const Vector3d& A = m_initialState.getVertexPosition(element[0]);
			const Vector3d& B = m_initialState.getVertexPosition(element[1]);
			spring.setStiffness(springStiffness);
			spring.setDamping(springDamping);
			spring.setInitialLength((B-A).norm());

			MeshElement<2, LinearSpringParameter> edge(element, spring);
			m_initialState.addEdge(edge);
		}
	}

	// Initialize the final state as a copy of the initial state
	m_finalState = m_initialState;

	// Initialize the internal current, previous state (Eigen vector) as well as velocity
	for (unsigned int nodeId = 0; nodeId < numNodesPerDim[0]; nodeId++)
	{
		m_x.segment(3 * nodeId, 3) = m_initialState.getVertexPosition(nodeId);
		m_xPrevious.segment(3 * nodeId, 3) = m_initialState.getVertexPosition(nodeId);
		m_v.segment(3 * nodeId, 3) = m_initialState.getVertex(nodeId).data.getVelocity();
	}

	// Set the number of dof for the Representation
	setNumDof(numNodesPerDim[0] * 3);
}

void MassSpringRepresentation::init2D(const Vector3d extremities[2][2], unsigned int numNodesPerDim[2],
	double totalMass, double springStiffness, double springDamping)
{

}
void MassSpringRepresentation::init3D(const Vector3d extremities[2][2][2], unsigned int numNodesPerDim[3],
	double totalMass, double springStiffness, double springDamping)
{

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

	// Backup current state into previous state
	m_xPrevious = m_x;
}

void MassSpringRepresentation::update(double dt)
{
	if (! isActive())
	{
		return;
	}

	if (getIntegrationScheme() == INTEGRATIONSCHEME_EXPLICIT_EULER)
	{
		updateEulerExplicit(dt);
	}
	else if (getIntegrationScheme() == INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER)
	{
		updateEulerExplicit(dt, true);
	}
}

void MassSpringRepresentation::afterUpdate(double dt)
{
	if (! isActive())
	{
		return;
	}

	// Back fill the new mass spring position into the mesh
	for (unsigned int vertexId = 0 ; vertexId < m_finalState.getNumVertices(); vertexId++)
	{
		m_finalState.getVertex(vertexId).position = m_x.segment(3 * vertexId, 3);
		m_finalState.getVertex(vertexId).data.setVelocity(m_v.segment(3 * vertexId, 3));
	}
}

void MassSpringRepresentation::applyDofCorrection(double dt,
	const Eigen::VectorBlock<SurgSim::Math::MlcpSolution::Vector>& block)
{
	if (! isActive())
	{
		return;
	}
}

void MassSpringRepresentation::updateEulerExplicit(double dt, bool useModifiedEuler)
{
	// For all node, we have m.a = F
	// Note that at this stage, m_x and m_v contains information at time t (not t+dt)
	// 1) Add gravity forces if gravity enabled
	// 2) Add Rayleigh damping forces
	// 3) Add spring forces
	// 4) Compute acceleration a(t) = F(t)/m
	// 5) Apply integration scheme (with boundary conditions)
	//     Euler explicit               OR   Modified Euler Explicit
	// {x(t+dt) = x(t) + dt.v(t)             {v(t+dt) = v(t) + dt.a(t)
	// {v(t+dt) = v(t) + dt.a(t)             {x(t+dt) = x(t) + dt.v(t+dt)

	m_f.setZero();

	// 1) Add gravity
	if (isGravityEnabled())
	{
		for (unsigned int nodeId = 0; nodeId < getNumMasses(); nodeId++)
		{
			m_f.segment(3 * nodeId, 3) = getGravity() * getMassParameter(nodeId).getMass();
		}
	}

	// 2) Add Rayleigh damping
	addRayleighDampingForce(&m_f, m_v);

	// 3) Add spring forces
	addSpringForces(&m_f, m_x, m_v);

	// 4) Compute acceleration (dividing by the mass)
	for (unsigned int nodeId = 0; nodeId < getNumMasses(); nodeId++)
	{
		m_a.segment(3 * nodeId, 3) = m_f.segment(3 * nodeId, 3) / getMassParameter(nodeId).getMass();
	}

	// 5) Apply numerical integration scheme
	if (useModifiedEuler)
	{
		m_v += m_a * dt;
		// apply the boundary conditions (v = 0 => x unchanged when updated)
		for (std::set<unsigned int>::const_iterator bcIt = m_boundaryConditions.begin();
			bcIt != m_boundaryConditions.end();
			bcIt++)
		{
			m_v.segment(3 * (*bcIt), 3) = Vector3d::Zero();
		}
		m_x += m_v * dt;
	}
	else
	{
		// apply the boundary conditions (v = 0 => x unchanged + f = 0 => v = 0)
		for (std::set<unsigned int>::const_iterator bcIt = m_boundaryConditions.begin();
			bcIt != m_boundaryConditions.end();
			bcIt++)
		{
			m_a.segment(3 * (*bcIt), 3) = Vector3d::Zero();
			m_v.segment(3 * (*bcIt), 3) = Vector3d::Zero();
		}
		m_x += m_v * dt;
		m_v += m_a * dt;
	}
}

void MassSpringRepresentation::allocate(int numDof)
{
	// Allocate internal Eigen data structure
	m_x.resize(numDof);
	m_xPrevious.resize(numDof);
	m_v.resize(numDof);
	m_f.resize(numDof);
	m_a.resize(numDof);

	// Zero-out the 4 states
	m_x.setZero();
	m_xPrevious.setZero();
	m_v.setZero();
	m_f.setZero();
	m_a.setZero();
}

void MassSpringRepresentation::addRayleighDampingForce(Vector *f, const Vector &v, double scale)
{
	// Rayleigh damping mass: F = - (coeffMass.M).v
	// M is diagonal, so this calculation can be done nodes per nodes
	if (m_rayleighDamping.massCoefficient)
	{
		for (unsigned int nodeID = 0; nodeID < getNumMasses(); nodeID++)
		{
			double mass = getMassParameter(nodeID).getMass();
			f->segment(3 * nodeID, 3) -= scale * m_rayleighDamping.massCoefficient * mass * v.segment(3 * nodeID, 3);
		}
	}

	// Rayleigh damping stiffness: F = - (coeffStiffness.K).v
	// K is not diagonal and links all dof of the 2 connected nodes
	// We need the spring stiffness matrix to complete that part...to be completed !
	if (m_rayleighDamping.stiffnessCoefficient)
	{
	}
}

void MassSpringRepresentation::addSpringForces(Vector *f, const Vector& x, const Vector& v, double scale)
{
	for (unsigned int springId = 0; springId < getNumSprings(); springId++)
	{
		unsigned int rowNodeId0 = 3 * m_finalState.getEdge(springId).verticesId[0];
		unsigned int rowNodeId1 = 3 * m_finalState.getEdge(springId).verticesId[1];

		const Vector3d localF = m_finalState.getEdge(springId).data.getF(
			x.segment(rowNodeId0, 3), x.segment(rowNodeId1, 3),
			v.segment(rowNodeId0, 3), v.segment(rowNodeId1, 3));

		f->segment(rowNodeId0, 3) += localF * scale;
		f->segment(rowNodeId1, 3) -= localF * scale;
	}
}

} // namespace Physics

} // namespace SurgSim