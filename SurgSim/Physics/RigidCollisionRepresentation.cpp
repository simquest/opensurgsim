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

#include "SurgSim/Physics/RigidCollisionRepresentation.h"

#include "SurgSim/Math/MeshShape.h"

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Physics
{

RigidCollisionRepresentation::RigidCollisionRepresentation(const std::string& name):
	Representation(name)
{
}

RigidCollisionRepresentation::~RigidCollisionRepresentation()
{

}

void RigidCollisionRepresentation::update(const double& dt)
{
	auto physicsRepresentation = m_physicsRepresentation.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr)
		<< "PhysicsRepresentation went out of scope for Collision Representation " << getName();

	auto shape = physicsRepresentation->getCurrentParameters().getShapeUsedForMassInertia();

	if (shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH)
	{
		auto localMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(shape);

		if (m_globalShape == nullptr)
		{
			m_globalShape = std::make_shared<SurgSim::Math::MeshShape>(*localMesh->getMesh());
		}

		auto globalMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(m_globalShape);

		// Update global-space mesh using local-space mesh
		globalMesh->getMesh()->setTransformedFrom(physicsRepresentation->getCurrentPose(), *localMesh->getMesh());
		m_aabbTree = globalMesh->createAabbTree();
	}
}

void RigidCollisionRepresentation::setRigidRepresentation(
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> representation)
{
	m_physicsRepresentation = representation;
}

std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> RigidCollisionRepresentation::getRigidRepresentation()
{
	return m_physicsRepresentation.lock();
}

int RigidCollisionRepresentation::getShapeType() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	return m_physicsRepresentation.lock()->getCurrentParameters().getShapeUsedForMassInertia()->getType();
}

const std::shared_ptr<SurgSim::Math::Shape> RigidCollisionRepresentation::getShape() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	return m_physicsRepresentation.lock()->getCurrentParameters().getShapeUsedForMassInertia();
}

const std::shared_ptr<SurgSim::Math::Shape> RigidCollisionRepresentation::getGlobalShape() const
{
	return m_globalShape;
}

const std::shared_ptr<SurgSim::DataStructures::AabbTree> RigidCollisionRepresentation::getAabbTree() const
{
	return m_aabbTree;
}

const SurgSim::Math::RigidTransform3d& RigidCollisionRepresentation::getPose() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	return m_physicsRepresentation.lock()->getCurrentPose();
}

void RigidCollisionRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "Cannot set the pose on a RigidCollisionRepresentation.";
}

void RigidCollisionRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "Cannot set the intial pose on a RigidCollisionRepresentation.";
}

const SurgSim::Math::RigidTransform3d& RigidCollisionRepresentation::getInitialPose() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	return m_physicsRepresentation.lock()->getInitialPose();
}


}; // namespace Collision
}; // namespace SurgSim




