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

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::DeformableCollisionRepresentation);
}

namespace SurgSim
{
namespace Physics
{
DeformableCollisionRepresentation::DeformableCollisionRepresentation(const std::string& name) :
	SurgSim::Collision::Representation(name)
{

}

DeformableCollisionRepresentation::~DeformableCollisionRepresentation()
{
}

void DeformableCollisionRepresentation::setMesh(std::shared_ptr<SurgSim::DataStructures::TriangleMesh> mesh)
{
	SURGSIM_ASSERT(!isInitialized()) << "Can't set mesh after initialization.";
	SURGSIM_ASSERT(mesh != nullptr) << "Can't use nullptr mesh.";

	m_shape = std::make_shared<SurgSim::Math::MeshShape>(*mesh);
	m_mesh = m_shape->getMesh();
}

std::shared_ptr<SurgSim::DataStructures::TriangleMesh> DeformableCollisionRepresentation::getMesh() const
{
	return m_mesh;
}


void DeformableCollisionRepresentation::update(const double& dt)
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr)
		<< "Failed to update.  The DeformableCollisionRepresentation either was not attached to a "
		"Physics::Representation or the Physics::Representation has expired.";

	auto state = physicsRepresentation->getCurrentState();

	const size_t numNodes = state->getNumNodes();

	SURGSIM_ASSERT(m_mesh->getNumVertices() == numNodes)
		<< "The number of nodes in the deformable does not match the number of vertices in the mesh.";

	for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
	{
		m_mesh->setVertexPosition(nodeId, state->getPosition(nodeId));
	}
	m_mesh->update();
}

bool DeformableCollisionRepresentation::doInitialize()
{
	SURGSIM_ASSERT(m_mesh != nullptr) << "Mesh was not set.";

	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr)
		<< "Failed to initialize.  The DeformableCollisionRepresentation either was not attached to a "
		   "Physics::Representation or the Physics::Representation has expired.";

	auto state = physicsRepresentation->getCurrentState();
	SURGSIM_ASSERT(m_mesh->getNumVertices() == state->getNumNodes())
		<< "The number of nodes in the deformable does not match the number of vertices in the mesh.";

	update(0.0);
	return true;
}

int DeformableCollisionRepresentation::getShapeType() const
{
	SURGSIM_ASSERT(m_shape != nullptr) << "No mesh or shape assigned to DeformableCollisionRepresentation "
									   << getName();
	return m_shape->getType();
}

void DeformableCollisionRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_ASSERT(shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH)
		<< "Deformable collision shape has to be a mesh.  Currently " << m_shape->getType();

	auto meshShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(shape);
	m_mesh = meshShape->getMesh();
}

const std::shared_ptr<SurgSim::Math::Shape> DeformableCollisionRepresentation::getShape() const
{
	return m_shape;
}

void DeformableCollisionRepresentation::setDeformableRepresentation(
	std::shared_ptr<SurgSim::Physics::DeformableRepresentationBase>representation)
{
	m_deformable = representation;
}

const std::shared_ptr<SurgSim::Physics::DeformableRepresentationBase>
	DeformableCollisionRepresentation::getDeformableRepresentation() const
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr)
		<< "Failed to get the deformable representation.  The DeformableCollisionRepresentation either was not "
		   "attached to a Physics::Representation or the Physics::Representation has expired.";

	return physicsRepresentation;
}

}
}

