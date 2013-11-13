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

#include <SurgSim/Collision/RigidCollisionRepresentation.h>

namespace SurgSim
{
namespace Collision
{

RigidCollisionRepresentation::RigidCollisionRepresentation(
	const std::string& name,
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> representation):
	CollisionRepresentation(name, representation),
	m_localRepresentation(representation)
{
	setPhysicsRepresentation(representation);
}

RigidCollisionRepresentation::~RigidCollisionRepresentation()
{

}

int RigidCollisionRepresentation::getShapeType() const
{
	return m_localRepresentation->getCurrentParameters().getShapeUsedForMassInertia()->getType();
}

const std::shared_ptr<SurgSim::Math::Shape> RigidCollisionRepresentation::getShape() const
{
	return m_localRepresentation->getCurrentParameters().getShapeUsedForMassInertia();
}

const SurgSim::Math::RigidTransform3d& RigidCollisionRepresentation::getPose() const
{
	return m_localRepresentation->getCurrentPose();
}

}; // namespace Collision
}; // namespace SurgSim




