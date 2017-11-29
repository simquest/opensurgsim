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

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Collision
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Collision::ShapeCollisionRepresentation,
				 ShapeCollisionRepresentation);

ShapeCollisionRepresentation::ShapeCollisionRepresentation(const std::string& name) :
	Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(ShapeCollisionRepresentation, std::shared_ptr<SurgSim::Math::Shape>, Shape,
									  getShape, setShape);
}

ShapeCollisionRepresentation::~ShapeCollisionRepresentation()
{
}

int ShapeCollisionRepresentation::getShapeType() const
{
	return m_shape->getType();
}


void ShapeCollisionRepresentation::setLocalPose(const SurgSim::Math::RigidTransform3d& pose)
{
	Representation::setLocalPose(pose);
	if (m_shape != nullptr)
	{
		if (m_shape->isTransformable())
		{
			m_shape->setPose(getPose());
		}
		Math::PosedShape<std::shared_ptr<Math::Shape>> posedShapeFirst(m_shape, getPose());
		Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion(posedShapeFirst, posedShapeFirst);
		setPosedShapeMotion(posedShapeMotion);
	}
}

void ShapeCollisionRepresentation::setShape(const std::shared_ptr<SurgSim::Math::Shape>& shape)
{
	SURGSIM_ASSERT(nullptr != shape) << "Can not set a empty shape.";
	m_shape = shape;
	if (m_shape->isTransformable())
	{
		m_shape->setPose(getPose());
	}
	Math::PosedShape<std::shared_ptr<Math::Shape>> posedShapeFirst(m_shape, getPose());
	Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion(posedShapeFirst, posedShapeFirst);
	setPosedShapeMotion(posedShapeMotion);
}

std::shared_ptr<Math::Shape> ShapeCollisionRepresentation::getShape() const
{
	return m_shape;
}

bool ShapeCollisionRepresentation::doInitialize()
{
	if (nullptr != m_shape)
	{
		SURGSIM_ASSERT(m_shape->isValid()) <<
			"An invalid MeshShape is used in this ShapeCollisionRepresentation.";
	}

	return true;
}

void ShapeCollisionRepresentation::updateCcdData(double timeOfImpact)
{
	using Math::PosedShape;
	using Math::PosedShapeMotion;
	using Math::Shape;

	Math::RigidTransform3d pose = getPose();
	if (!pose.isApprox(m_previousCcdCurrentPose))
	{
		m_previousCcdCurrentPose = pose;
		if (m_shape->isTransformable())
		{
			m_shape->setPose(pose);
			m_shape->updateShape();
		}

		PosedShape<std::shared_ptr<Shape>> posedShape1(m_shape, pose);
		PosedShapeMotion<std::shared_ptr<Shape>> newPosedShapeMotion(posedShape1, posedShape1);
		setPosedShapeMotion(newPosedShapeMotion);
	}
}

}; // namespace Collision
}; // namespace SurgSim
