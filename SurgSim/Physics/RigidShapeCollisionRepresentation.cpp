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

#include <SurgSim/Physics/RigidShapeCollisionRepresentation.h>

namespace SurgSim
{
namespace Physics
{


RigidShapeCollisionRepresentation::RigidShapeCollisionRepresentation(
	const std::shared_ptr<RigidShape>& shape,
	const SurgSim::Math::Quaterniond& quat,
	const SurgSim::Math::Vector3d& translation) :
	m_shape(shape), m_transform(SurgSim::Math::makeRigidTransform(quat, translation))
{

}

RigidShapeCollisionRepresentation::RigidShapeCollisionRepresentation(
	const std::shared_ptr<RigidShape>& shape,
	const SurgSim::Math::RigidTransform3d& pose) :
	m_shape(shape), m_transform(pose)
{

}


RigidShapeCollisionRepresentation::~RigidShapeCollisionRepresentation()
{

}

int RigidShapeCollisionRepresentation::getShapeType() const
{
	return m_shape->getType();
}

const std::shared_ptr<SurgSim::Physics::RigidShape> RigidShapeCollisionRepresentation::getShape() const
{
	return m_shape;
}

const SurgSim::Math::RigidTransform3d& RigidShapeCollisionRepresentation::getCurrentPose() const
{
	return m_transform;
}

}; // Physics
}; // SurgSim
