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

#include <SurgSim/Physics/RigidCollisionRepresentation.h>

namespace SurgSim
{
namespace Physics
{

RigidCollisionRepresentation::RigidCollisionRepresentation(std::shared_ptr<RigidRepresentation> representation) : m_representation(representation)
{

}

RigidCollisionRepresentation::~RigidCollisionRepresentation()
{

}

int RigidCollisionRepresentation::getShapeType() const
{
	return m_representation->getCurrentParameters().getShapeUsedForMassInertia()->getType();
}

const std::shared_ptr<RigidShape> RigidCollisionRepresentation::getShape() const
{
	return m_representation->getCurrentParameters().getShapeUsedForMassInertia();
}

const SurgSim::Math::RigidTransform3d& RigidCollisionRepresentation::getCurrentPose() const
{
	return m_representation->getCurrentPose();
}

}; // Physics
}; // SurgSim




