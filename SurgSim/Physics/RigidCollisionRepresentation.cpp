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

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Physics/RigidRepresentationBase.h"

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::RigidCollisionRepresentation,
				 RigidCollisionRepresentation);

RigidCollisionRepresentation::RigidCollisionRepresentation(const std::string& name):
	Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(RigidCollisionRepresentation, std::shared_ptr<SurgSim::Math::Shape>,
									  Shape, getShape, setShape);
}

RigidCollisionRepresentation::~RigidCollisionRepresentation()
{
}

bool RigidCollisionRepresentation::doWakeUp()
{
	if (m_shape == nullptr)
	{
		auto physicsRepresentation = m_physicsRepresentation.lock();
		SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
		setShape(physicsRepresentation->getShape());
	}
	return m_shape != nullptr;
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
	return m_shape->getType();
}

std::shared_ptr<Math::Shape> RigidCollisionRepresentation::getShape() const
{
	return m_shape;
}

void RigidCollisionRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_ASSERT(shape != nullptr) << "Cannot set nullptr shape on " << getFullName();
	m_shape = shape;
	m_previousShape = shape->getCopy();
}

void RigidCollisionRepresentation::update(const double& time)
{
	Math::RigidTransform3d physicsPreviousPose;
	Math::RigidTransform3d physicsCurrentPose;
	Math::RigidTransform3d transform;
	{
		auto physicsRepresentation = m_physicsRepresentation.lock();
		SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
		physicsCurrentPose = physicsRepresentation->getCurrentState().getPose();
		physicsPreviousPose = physicsRepresentation->getPreviousState().getPose();
		transform = physicsRepresentation->getLocalPose().inverse() * getLocalPose();
	}

	m_shape->setPose(physicsCurrentPose * transform);

	if (getCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)
	{
		m_previousShape->setPose(physicsPreviousPose * transform);
		Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape1(m_previousShape, m_previousShape->getPose());
		Math::PosedShape<std::shared_ptr<Math::Shape>> posedShape2(m_shape, m_shape->getPose());
		Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion(posedShape1, posedShape2);
		setPosedShapeMotion(posedShapeMotion);
	}
}

SurgSim::Math::RigidTransform3d RigidCollisionRepresentation::getPose() const
{
	auto physicsRepresentation = m_physicsRepresentation.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	const SurgSim::Math::RigidTransform3d& physicsPose = physicsRepresentation->getCurrentState().getPose();
	return physicsPose * physicsRepresentation->getLocalPose().inverse() * getLocalPose();
}

}; // namespace Collision
}; // namespace SurgSim
