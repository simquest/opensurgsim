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

#include <SurgSim/Physics/UnitTests/MockCollisionRepresentation.h>
#include <SurgSim/Physics/Representation.h>

namespace SurgSim
{
namespace Physics
{

MockCollisionRepresentation::MockCollisionRepresentation(
		const std::string& name,
		const std::shared_ptr<RigidShape>& shape,
		const SurgSim::Math::Quaterniond& quat,
		const SurgSim::Math::Vector3d& translation,
		std::shared_ptr<SurgSim::Physics::Representation> representation) :
	CollisionRepresentation(name, representation),
	m_shape(shape),
	m_transform(SurgSim::Math::makeRigidTransform(quat, translation))
{

}

MockCollisionRepresentation::MockCollisionRepresentation(
		const std::string& name,
		const std::shared_ptr<RigidShape>& shape,
		const SurgSim::Math::RigidTransform3d& pose,
		std::shared_ptr<SurgSim::Physics::Representation> representation) :
	CollisionRepresentation(name,representation),
	m_shape(shape),
	m_transform(pose)
{

}


MockCollisionRepresentation::~MockCollisionRepresentation()
{

}

int MockCollisionRepresentation::getShapeType() const
{
	return m_shape->getType();
}

const std::shared_ptr<SurgSim::Physics::RigidShape> MockCollisionRepresentation::getShape() const
{
	return m_shape;
}

const SurgSim::Math::RigidTransform3d& MockCollisionRepresentation::getPose() const
{
	return m_transform;
}

}; // Physics
}; // SurgSim
