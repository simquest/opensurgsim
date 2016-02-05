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
	return getShape()->getType();
}

const std::shared_ptr<SurgSim::Math::Shape> RigidCollisionRepresentation::getShape() const
{
	if (m_shape != nullptr)
	{
		return m_shape;
	}
	else
	{
		auto physicsRepresentation = m_physicsRepresentation.lock();
		SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
				"PhysicsRepresentation went out of scope for Collision Representation " << getName();
		return physicsRepresentation->getShape();
	}
}

void RigidCollisionRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	m_shape = shape;
}

void RigidCollisionRepresentation::update(const double& time)
{
	using Math::PosedShape;
	using Math::PosedShapeMotion;
	using Math::Shape;

	if (getCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)
	{
		Math::RigidTransform3d previousPose;
		Math::RigidTransform3d currentPose;
		{
			auto physicsRepresentation = m_physicsRepresentation.lock();
			SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
				"PhysicsRepresentation went out of scope for Collision Representation " << getName();
			const Math::RigidTransform3d& physicsCurrentPose = physicsRepresentation->getCurrentState().getPose();
			const Math::RigidTransform3d& physicsPreviousPose = physicsRepresentation->getPreviousState().getPose();

			previousPose = physicsPreviousPose * physicsRepresentation->getLocalPose().inverse() * getLocalPose();
			currentPose = physicsCurrentPose * physicsRepresentation->getLocalPose().inverse() * getLocalPose();
		}

		std::shared_ptr<Shape> previousShape = getShape();
		if (previousShape->isTransformable())
		{
			previousShape = getShape()->getTransformed(previousPose);
		}

		std::shared_ptr<Shape> currentShape = getShape();
		if (currentShape->isTransformable())
		{
			currentShape = getShape()->getTransformed(currentPose);
		}

		PosedShape<std::shared_ptr<Shape>> posedShape1(previousShape, previousPose);
		PosedShape<std::shared_ptr<Shape>> posedShape2(currentShape, currentPose);
		PosedShapeMotion<std::shared_ptr<Shape>> posedShapeMotion(posedShape1, posedShape2);

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
