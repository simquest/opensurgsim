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
	update(0.0);
}

void ShapeCollisionRepresentation::setShape(const std::shared_ptr<SurgSim::Math::Shape>& shape)
{
	SURGSIM_ASSERT(nullptr != shape) << "Can not set a empty shape.";
	m_shape = shape;
	update(0.0);
}

const std::shared_ptr<SurgSim::Math::Shape> ShapeCollisionRepresentation::getShape() const
{
	return m_shape;
}

void ShapeCollisionRepresentation::update(const double& dt)
{
	auto meshShape = std::dynamic_pointer_cast<SurgSim::Math::MeshShape>(m_shape);
	if (nullptr != meshShape)
	{
		SURGSIM_LOG_IF(!meshShape->isValid(), SurgSim::Framework::Logger::getDefaultLogger(), WARNING) <<
			"Try to update an invalid MeshShape.";
		if (!meshShape->setPose(getPose()))
		{
			setLocalActive(false);
			SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getLogger("Collision/ShapeCollisionRepresentation")) <<
				"SceneElement '" << getSceneElement()->getName() << "', Collision representation '" << getName() <<
				"' went inactive because its shape failed in moving to a pose of:" << std::endl << getPose().matrix();
		}
	}
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

}; // namespace Collision
}; // namespace SurgSim
