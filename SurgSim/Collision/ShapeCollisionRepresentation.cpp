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
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Collision
{


ShapeCollisionRepresentation::ShapeCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<SurgSim::Math::Shape> shape,
		const SurgSim::Math::RigidTransform3d& pose ) :
	Representation(name),
	m_shape(shape),
	m_pose(pose)
{
}

ShapeCollisionRepresentation::~ShapeCollisionRepresentation()
{

}

int ShapeCollisionRepresentation::getShapeType() const
{
	return m_shape->getType();
}

const std::shared_ptr<SurgSim::Math::Shape> ShapeCollisionRepresentation::getShape() const
{
	return m_shape;
}

const std::shared_ptr<SurgSim::Math::Shape> ShapeCollisionRepresentation::getGlobalShape() const
{
	return m_globalShape;
}

const std::shared_ptr<SurgSim::DataStructures::AabbTree> ShapeCollisionRepresentation::getAabbTree() const
{
	return m_aabbTree;
}

void ShapeCollisionRepresentation::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

const SurgSim::Math::RigidTransform3d& ShapeCollisionRepresentation::getPose() const
{
	return m_pose;
}

void ShapeCollisionRepresentation::setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

const SurgSim::Math::RigidTransform3d& ShapeCollisionRepresentation::getInitialPose() const
{
	return m_pose;
}

void ShapeCollisionRepresentation::update(const double& dt)
{
	auto& shape = m_shape;

	if (shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH)
	{
		auto localMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(shape);

		if (m_globalShape == nullptr)
		{
			m_globalShape = std::make_shared<SurgSim::Math::MeshShape>(*localMesh->getMesh());
		}

		auto globalMesh = std::static_pointer_cast<SurgSim::Math::MeshShape>(m_globalShape);

		// Update global-space mesh using local-space mesh
		globalMesh->getMesh()->setTransformedFrom(m_pose, *localMesh->getMesh());
		m_aabbTree = globalMesh->createAabbTree();
	}
}

}; // namespace Collision
}; // namespace SurgSim
