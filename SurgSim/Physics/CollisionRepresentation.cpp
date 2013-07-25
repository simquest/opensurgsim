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

#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/Representation.h>

namespace SurgSim
{
namespace Physics
{

CollisionRepresentation::CollisionRepresentation(const std::string& name) :
	Representation(name)
{

}

CollisionRepresentation::CollisionRepresentation(
	const std::string& name,
	std::shared_ptr<SurgSim::Physics::Representation> representation) :
	Representation(name),
	m_physicsRepresentation(representation)
{

}

CollisionRepresentation::~CollisionRepresentation()
{

}

void CollisionRepresentation::setPhysicsRepresentation(
	const std::shared_ptr<SurgSim::Physics::Representation>& physicsRepresentation)
{
	m_physicsRepresentation = physicsRepresentation;
}

std::shared_ptr<Representation> CollisionRepresentation::getPhysicsRepresentation()
{
	return m_physicsRepresentation.lock();
}

const SurgSim::Math::RigidTransform3d& CollisionRepresentation::getPose() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
		"PhysicsRepresentation went out of scope for CollisionRepresentation " << getName();
	return m_physicsRepresentation.lock()->getPose();
}

void CollisionRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "Cannot set an initial pose on a collision representation.";
}

const SurgSim::Math::RigidTransform3d& CollisionRepresentation::getInitialPose() const
{
	SURGSIM_ASSERT(!m_physicsRepresentation.expired()) <<
		"PhysicsRepresentation went out of scope for CollisionRepresentation " << getName();
	return m_physicsRepresentation.lock()->getPose();
}

void CollisionRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_FAILURE() << "Cannot set the pose on a collision representation.";
}

}; // Physics
}; // SurgSim
