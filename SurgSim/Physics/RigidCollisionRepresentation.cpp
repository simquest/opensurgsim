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

SurgSim::Math::RigidTransform3d RigidCollisionRepresentation::getPose() const
{
	auto physicsRepresentation = m_physicsRepresentation.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
			"PhysicsRepresentation went out of scope for Collision Representation " << getName();
	const SurgSim::Math::RigidTransform3d& physicsPose = physicsRepresentation->getCurrentState().getPose();
	return physicsPose * physicsRepresentation->getLocalPose().inverse() * getLocalPose();
}

void RigidCollisionRepresentation::updateShapeData()
{
	// All we want to do is to transform the shape into the current pose
	getPosedShape();
}


void RigidCollisionRepresentation::updateDcdData()
{
	// HS-2-Mar-2016
	// #todo need to trigger the aabb tree build/update here
}


void RigidCollisionRepresentation::updateCcdData(double interval)
{
	using Math::PosedShape;
	using Math::PosedShapeMotion;
	using Math::Shape;

	Math::RigidTransform3d previousPose;
	Math::RigidTransform3d currentPose;
	{
		auto physicsRepresentation = m_physicsRepresentation.lock();
		SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
				"PhysicsRepresentation went out of scope for Collision Representation " << getName();
		const Math::RigidTransform3d& physicsCurrentPose = physicsRepresentation->getCurrentState().getPose();
		const Math::RigidTransform3d& physicsPreviousPose = physicsRepresentation->getPreviousState().getPose();

		Math::RigidTransform3d transform = physicsRepresentation->getLocalPose().inverse() * getLocalPose();
		previousPose = physicsPreviousPose * transform;
		currentPose = physicsCurrentPose * transform;
	}

	std::shared_ptr<Shape> previousShape = getShape();
	std::shared_ptr<Shape> currentShape = getShape();
	if (getShape()->isTransformable())
	{
		previousShape = getShape()->getTransformed(previousPose);
		currentShape = getShape()->getTransformed(currentPose);
	}

	PosedShape<std::shared_ptr<Shape>> posedShape1(previousShape, previousPose);
	PosedShape<std::shared_ptr<Shape>> posedShape2(currentShape, currentPose);
	PosedShapeMotion<std::shared_ptr<Shape>> posedShapeMotion(posedShape1, posedShape2);

	setPosedShapeMotion(posedShapeMotion);

	// HS-2-Mar-2016
	// #todo Add AABB tree for the posedShapeMotion (i.e. that is the tree where each bounding box consists of the
	// corresponding elements from posedShape1 and posedShape2

}


}; // namespace Collision
}; // namespace SurgSim
