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

#include "SurgSim/Physics/DeformableCollisionRepresentation.h"

#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"

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
	m_mesh = mesh;
}

std::shared_ptr<SurgSim::DataStructures::TriangleMesh> DeformableCollisionRepresentation::getMesh() const
{
	return m_mesh;
}


void DeformableCollisionRepresentation::update(const double& dt)
{
	SURGSIM_ASSERT(!m_deformable.expired()) << "Deformable has expired, cannot update the mesh.";
	auto state = m_deformable.lock()->getCurrentState();

	const size_t numNodes = state->getNumNodes();

	SURGSIM_ASSERT(m_mesh->getNumVertices() == numNodes) << "The number of nodes in the deformable does not match " <<
			"the number of vertices in the mesh.";

	for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
	{
		m_mesh->setVertexPosition(nodeId, state->getPosition(nodeId));
	}
}

bool DeformableCollisionRepresentation::doInitialize()
{
	SURGSIM_ASSERT(m_mesh != nullptr) << "Mesh was not set.";
	SURGSIM_ASSERT(!m_deformable.expired()) << "Can't startup without a deformable.";

	auto state = m_deformable.lock()->getCurrentState();
	SURGSIM_ASSERT(m_mesh->getNumVertices() == state->getNumNodes()) <<
			"The number of nodes in the deformable does not match " <<
			"the number of vertices in the mesh.";

	update(0.0);
	return true;
}

int DeformableCollisionRepresentation::getShapeType() const
{
	SURGSIM_ASSERT(m_shape != nullptr) << "No mesh or shape assigned to DeformableCollisionRepresentation " <<
									   getName();
	return m_shape->getType();
}

void DeformableCollisionRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_ASSERT(shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH) <<
			"Deformable collision shape has to be a mesh." <<
			" currently " << m_shape->getType();

	auto meshShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(shape);
	m_mesh = meshShape->getMesh();

}

void DeformableCollisionRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "The initial pose cannot be set";
}

const SurgSim::Math::RigidTransform3d& DeformableCollisionRepresentation::getInitialPose() const
{
	SURGSIM_ASSERT(m_deformable.expired()) <<
										   "Cannot get the initial pose because the deformable was not initialized.";
	return m_deformable.lock()->getInitialPose();
}

void DeformableCollisionRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "The pose cannot be set";
}

const SurgSim::Math::RigidTransform3d& DeformableCollisionRepresentation::getPose() const
{
	SURGSIM_ASSERT(m_deformable.expired()) << "Cannot get the pose because the deformable was not initialized.";
	return m_deformable.lock()->getInitialPose();
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

}
}

