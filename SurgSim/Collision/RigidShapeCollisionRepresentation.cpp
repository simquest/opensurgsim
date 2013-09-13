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

#include <SurgSim/Collision/RigidShapeCollisionRepresentation.h>
#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/RigidShape.h>

namespace SurgSim
{
namespace Collision
{


RigidShapeCollisionRepresentation::RigidShapeCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<SurgSim::Physics::RigidShape> shape,
		std::shared_ptr<SurgSim::Physics::Representation> representation) :
	CollisionRepresentation(name,representation),
	m_shape(shape)
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



}; // namespace Collision
}; // namespace SurgSim
