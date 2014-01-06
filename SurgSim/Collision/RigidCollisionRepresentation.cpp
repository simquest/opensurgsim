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

#include "SurgSim/Collision/RigidCollisionRepresentation.h"

namespace SurgSim
{
namespace Collision
{

RigidCollisionRepresentation::RigidCollisionRepresentation(const std::string& name):
	Representation(name),
	m_physicsRepresentation()
{
}

void RigidCollisionRepresentation::setRepresentation(
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> representation)
{
	m_physicsRepresentation = representation;
}

RigidCollisionRepresentation::~RigidCollisionRepresentation()
{

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

std::shared_ptr<SurgSim::Physics::Representation> RigidCollisionRepresentation::getPhysicsRepresentation()
{
	return m_physicsRepresentation.lock();
}

}; // namespace Collision
}; // namespace SurgSim




