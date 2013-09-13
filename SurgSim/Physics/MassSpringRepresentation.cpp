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

MassSpringRepresentation::MassSpringRepresentation(const std::string& name) : DeformableRepresentation(name)
{

}

MassSpringRepresentation::~MassSpringRepresentation()
{

}

unsigned int MassSpringRepresentation::getNumMasses(void) const
{
	return m_tetrahedronMesh.getNumVertices();
}

unsigned int MassSpringRepresentation::getNumSprings(void) const
{
	return m_tetrahedronMesh.getNumEdges();
}

const Mass& MassSpringRepresentation::getMass(unsigned int massId) const
{
	SURGSIM_ASSERT(massId < getNumMasses()) << "Invalid mass id";
	return m_tetrahedronMesh.getVertex(massId).data;
}

const LinearSpring& MassSpringRepresentation::getSpring(unsigned int springId) const
{
	SURGSIM_ASSERT(springId < getNumSprings()) << "Invalid spring id";
	return m_tetrahedronMesh.getEdge(springId).data;
}

double MassSpringRepresentation::getTotalMass(void) const
{
	using std::vector;

	double mass = 0.0;
	const vector<Vertex<Mass>> &vertices = m_tetrahedronMesh.getVertices();

	for (vector<Vertex<Mass>>::const_iterator it = vertices.begin(); it != vertices.end(); it++)
	{
		mass += it->data.getMass();
	}

	return mass;
}

double MassSpringRepresentation::getRayleighDampingStiffness(void) const
{
	return m_rayleighDamping.stiffnessCoefficient;
}

double MassSpringRepresentation::getRayleighDampingMass(void) const
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

void MassSpringRepresentation::addBC(int nodeID)
{
	m_boundaryConditions.push_back(nodeID);
}

int MassSpringRepresentation::getBC(size_t bcID) const
{
	SURGSIM_ASSERT(bcID >= 0 && bcID < m_boundaryConditions.size()) << "Invalid boundary condition " << bcID;
	return m_boundaryConditions[bcID];
}

size_t MassSpringRepresentation::getNumBC(void) const
{
	return m_boundaryConditions.size();
}

void MassSpringRepresentation::setIntegrationScheme(IntegrationScheme integrationScheme)
{
	m_integrationScheme = integrationScheme;
}

MassSpringRepresentation::IntegrationScheme MassSpringRepresentation::getIntegrationScheme(void) const
{
	return m_integrationScheme;
}

//! Initialization
void MassSpringRepresentation::init1D(const Vector3d extremities[2], int numNodesPerDim[1],
	double totalMass, double springStiffness, double springDamping)
{
	SURGSIM_ASSERT(numNodesPerDim[0] > 0) << "Number of nodes incorrect: " << numNodesPerDim[0];

	// Allocate all Eigen data structures and all states
	allocate(numNodesPerDim[0] * 3);

	Vector3d delta = (extremities[1] - extremities[0]) / static_cast<double>(numNodesPerDim[0] - 1);
	for (int massId = 0; massId < numNodesPerDim[0]; massId++)
	{
		Mass mass(totalMass / static_cast<double>(numNodesPerDim[0]));
		Vertex<Mass> vertex(extremities[0] + massId * delta, mass);
		m_tetrahedronMesh.addVertex(vertex);
	}

	for (int massId = 0; massId < numNodesPerDim[0] - 1; massId++)
	{
		LinearSpring spring;
		std::array<unsigned int, 2> element;

		spring.setStiffness(springStiffness);
		spring.setDamping(springDamping);
		element[0] = massId;
		element[1] = massId + 1;

		MeshElement<2, LinearSpring> edge(element, spring);
		m_tetrahedronMesh.addEdge(edge);
	}

	// Initialize all 4 states with the proper vertices position
	for (int massId = 0; massId < numNodesPerDim[0]; massId++)
	{
		m_initialState.getPositions().block(3*massId, 0, 3, 1) = m_tetrahedronMesh.getVertexPosition(massId);
	}
	m_previousState = m_initialState;
	m_currentState  = m_initialState;
	m_finalState    = m_initialState;

	// Set the number of dof for the Representation
	setNumDof(numNodesPerDim[0]*3);
}

void MassSpringRepresentation::init2D(const Vector3d extremities[2][2], int numNodesPerDim[2])
{

}
void MassSpringRepresentation::init3D(const Vector3d extremities[2][2][2], int numNodesPerDim[3])
{

}

/// Query the representation type
/// \return the RepresentationType for this representation
RepresentationType MassSpringRepresentation::getType() const
{
	return REPRESENTATION_TYPE_MASSSPRING;
}

/// Preprocessing done before the update call
/// \param dt The time step (in seconds)
void MassSpringRepresentation::beforeUpdate(double dt)
{
	// Backup current state into previous state
	m_previousState = m_currentState;
}

/// Update the representation state to the current time step
/// \param dt The time step (in seconds)
void MassSpringRepresentation::update(double dt)
{
	if (getIntegrationScheme() == INTEGRATIONSCHEME_EXPLICIT_EULER)
	{
		updateEulerExplicit(dt);
	}
	else if (getIntegrationScheme() == INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER)
	{
		updateEulerExplicit(dt, true);
	}
}

/// Postprocessing done after the update call
/// \param dt The time step (in seconds)
void MassSpringRepresentation::afterUpdate(double dt)
{
	// Backup current state into final state
	m_finalState = m_currentState;
}

/// Apply a correction to the internal degrees of freedom
/// \param dt The time step
/// \param block The block of a vector containing the correction to be applied to the dof
void MassSpringRepresentation::applyDofCorrection(double dt, const Eigen::VectorBlock<SurgSim::Math::MlcpSolution::Vector>& block)
{

}

void MassSpringRepresentation::updateEulerExplicit(double dt, bool useModifiedEuler)
{
	// For all node, we have m.a = F
	// Note that at this stage, m_x and m_v contains information at time t (not t+dt)
	// 1) Apply gravity to all node if gravity enabled
	// 2) Loop through all springs and compute forces F(t) on proper nodes
	// 3) Compute acceleration a(t) = F(t)/m
	// 4) Apply integration scheme 
	//     Euler explicit               OR   Modified Euler Explicit
	// {x(t+dt) = x(t) + dt.v(t)             {v(t+dt) = v(t) + dt.a(t)
	// {v(t+dt) = v(t) + dt.a(t)             {x(t+dt) = x(t) + dt.v(t+dt)
	
	m_f.setZero();

	// 1) Apply gravity
	if (isGravityEnabled())
	{
		for (unsigned int nodeId = 0; nodeId < getNumMasses(); nodeId++)
		{
			m_f.block(3 * nodeId, 0, 3, 1) += getGravity() * getMass(nodeId).getMass();
		}
	}
	
	// 2) Apply spring forces
	for (unsigned int springId = 0; springId < getNumSprings(); springId++)
	{
		const Vector3d& f = m_tetrahedronMesh.getEdge(springId).data.getF();
		int nodeId0 = m_tetrahedronMesh.getEdge(springId).vertices[0];
		int nodeId1 = m_tetrahedronMesh.getEdge(springId).vertices[1];
		m_f.block(3 * nodeId0, 0, 3, 1) += f;
		m_f.block(3 * nodeId1, 0, 3, 1) -= f;
	}

	// 3) Compute acceleration (dividing by the mass)
	for (unsigned int nodeId = 0; nodeId < getNumMasses(); nodeId++)
	{
		m_f.block(3 * nodeId, 0, 3, 1) /= getMass(nodeId).getMass();
	}

	// 4) Apply numerical integration scheme
	Vector& x = m_currentState.getPositions();
	Vector& v = m_currentState.getVelocities();
	if (useModifiedEuler)
	{
		v += m_f * dt;
		x +=   v * dt;
	}
	else
	{
		x +=   v * dt;
		v += m_f * dt;
	}
}

void MassSpringRepresentation::allocate(int numDof)
{	
	m_f.resize(numDof);

	// Allocate the 4 states
	m_initialState.allocate(numDof);
	m_previousState.allocate(numDof);
	m_currentState.allocate(numDof);
	m_finalState.allocate(numDof);

	// Zero-out the 4 states
	m_initialState.reset();
	m_previousState.reset();
	m_currentState.reset();
	m_finalState.reset();
}

} // namespace Physics

} // namespace SurgSim