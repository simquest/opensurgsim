// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"
#include "SurgSim/Math/Shape.h"
#include "SurgSim/Math/SurfaceMeshShape.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"

namespace SurgSim
{
namespace Physics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Physics::DeformableCollisionRepresentation,
				 DeformableCollisionRepresentation);

DeformableCollisionRepresentation::DeformableCollisionRepresentation(const std::string& name) :
	SurgSim::Collision::Representation(name),
	m_oldVolume(0.0),
	m_previousOldVolume(0.0),
	m_aabbThreshold(0.1)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(DeformableCollisionRepresentation, std::shared_ptr<SurgSim::Math::Shape>,
									  Shape, getShape, setShape);
}

DeformableCollisionRepresentation::~DeformableCollisionRepresentation()
{
}

namespace
{

/// Call update on the shape if the AABB tree has changed significantly, otherwise just update the AABB tree.
/// \param odeState The state.
/// \param [in,out] shape The shape to update.
/// \param [in,out] oldVolume The previous volume of the AABB tree.
/// \param threshold The aabb volume threshold that triggers a tree rebuild.
void updateShapeFromOdeState(const Math::OdeState& odeState, SurgSim::Math::Shape* shape,
	double* oldVolume, double threshold)
{
	if (shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH ||
		shape->getType() == SurgSim::Math::SHAPE_TYPE_SURFACEMESH)
	{
		auto meshShape = dynamic_cast<SurgSim::Math::MeshShape*>(shape);
		SURGSIM_ASSERT(meshShape != nullptr) << "The shape is neither a mesh nor a surface mesh";
		const size_t numNodes = odeState.getNumNodes();
		SURGSIM_ASSERT(meshShape->getNumVertices() == numNodes) <<
			"The number of nodes in the deformable does not match the number of vertices in the shape.";

		for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
		{
			meshShape->setVertexPosition(nodeId, odeState.getPosition(nodeId));
		}

		if (std::abs(*oldVolume - meshShape->getBoundingBox().volume()) >(*oldVolume) * threshold)
		{
			meshShape->update();
			*oldVolume = meshShape->getBoundingBox().volume();
		}
		else
		{
			meshShape->updateAabbTree();
			meshShape->calculateNormals();
		}
	}
	else if (shape->getType() == SurgSim::Math::SHAPE_TYPE_SEGMENTMESH)
	{
		auto meshShape = dynamic_cast<SurgSim::Math::SegmentMeshShape*>(shape);
		SURGSIM_ASSERT(meshShape != nullptr) << "The shape is of type SegmentMeshShape but the dynamic cast failed.";
		const size_t numNodes = odeState.getNumNodes();
		SURGSIM_ASSERT(meshShape->getNumVertices() == numNodes) <<
			"The number of nodes in the deformable does not match the number of vertices in the shape.";

		for (size_t nodeId = 0; nodeId < numNodes; ++nodeId)
		{
			meshShape->setVertexPosition(nodeId, odeState.getPosition(nodeId));
		}

		if (std::abs(*oldVolume - meshShape->getBoundingBox().volume()) > (*oldVolume) * threshold)
		{
			meshShape->update();
			*oldVolume = meshShape->getBoundingBox().volume();
		}
		else
		{
			meshShape->updateAabbTree();
		}
	}
}
}

bool DeformableCollisionRepresentation::doInitialize()
{
	bool result = false;
	if (nullptr != m_shape && m_shape->isValid())
	{
		result = true;
	}

	return result;
}

bool DeformableCollisionRepresentation::doWakeUp()
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(nullptr != physicsRepresentation)
			<< "The Physics::Representation referred by this DeformableCollisionRepresentation has expired.";

	auto state = physicsRepresentation->getCurrentState();
	SURGSIM_ASSERT(nullptr != state)
			<< "DeformableRepresentation " << physicsRepresentation->getName() << " holds an empty OdeState.";
	auto shape = std::dynamic_pointer_cast<DataStructures::VerticesPlain>(m_shape);
	SURGSIM_ASSERT(shape != nullptr)
			<< "The shape object is not inherited from DataStructures::VerticesPlain, but should be.";
	SURGSIM_ASSERT(shape->getNumVertices() == state->getNumNodes())
			<< "The number of nodes in the deformable does not match the number of vertices in the mesh.";

	update(0.0);
	return true;
}

void DeformableCollisionRepresentation::setCollisionDetectionType(Collision::CollisionDetectionType type)
{
	Collision::Representation::setCollisionDetectionType(type);

	if ((m_shape != nullptr) &&
		((getCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS) ||
		(getSelfCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)))
	{
		if (m_previousShape == nullptr)
		{
			m_previousShape = m_shape->getTransformed(Math::RigidTransform3d::Identity());
		}
	}
	else
	{
		m_previousShape.reset();
	}
}

void DeformableCollisionRepresentation::setSelfCollisionDetectionType(Collision::CollisionDetectionType type)
{
	Collision::Representation::setSelfCollisionDetectionType(type);

	if ((m_shape != nullptr) &&
		((getCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS) ||
		(getSelfCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS)))
	{
		if (m_previousShape == nullptr)
		{
			m_previousShape = m_shape->getTransformed(Math::RigidTransform3d::Identity());
		}
	}
	else
	{
		m_previousShape.reset();
	}
}

int DeformableCollisionRepresentation::getShapeType() const
{
	SURGSIM_ASSERT(nullptr != m_shape) << "No mesh/shape assigned to DeformableCollisionRepresentation " << getName();
	return m_shape->getType();
}

void DeformableCollisionRepresentation::setShape(std::shared_ptr<SurgSim::Math::Shape> shape)
{
	SURGSIM_ASSERT(shape->getType() == SurgSim::Math::SHAPE_TYPE_MESH ||
				   shape->getType() == SurgSim::Math::SHAPE_TYPE_SEGMENTMESH ||
				   shape->getType() == SurgSim::Math::SHAPE_TYPE_SURFACEMESH) <<
		"Deformable collision shape has to be a MeshShape(" << SurgSim::Math::SHAPE_TYPE_MESH <<
		") or SegmentMeshShape(" << SurgSim::Math::SHAPE_TYPE_SEGMENTMESH <<
		") or SurfaceMeshShape(" << SurgSim::Math::SHAPE_TYPE_SURFACEMESH << "), but it is " << m_shape->getType();

	m_shape = shape;

	if ((getCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS) ||
		(getSelfCollisionDetectionType() == Collision::COLLISION_DETECTION_TYPE_CONTINUOUS))
	{
		m_previousShape = m_shape->getTransformed(Math::RigidTransform3d::Identity());
	}
	else
	{
		m_previousShape.reset();
	}
}

std::shared_ptr<Math::Shape> DeformableCollisionRepresentation::getShape() const
{
	return m_shape;
}

void DeformableCollisionRepresentation::setDeformableRepresentation(
	std::shared_ptr<SurgSim::Physics::DeformableRepresentation>representation)
{
	m_deformable = representation;
}

const std::shared_ptr<SurgSim::Physics::DeformableRepresentation>
DeformableCollisionRepresentation::getDeformableRepresentation() const
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(physicsRepresentation != nullptr) <<
			"Failed to get the deformable representation.  The DeformableCollisionRepresentation either was not "
			"attached to a Physics::Representation or the Physics::Representation has expired.";

	return physicsRepresentation;
}


void DeformableCollisionRepresentation::updateShapeData()
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(nullptr != physicsRepresentation) <<
			"Failed to update. The DeformableCollisionRepresentation either was not attached to a "
			"Physics::Representation or the Physics::Representation has expired.";

	updateShapeFromOdeState(*physicsRepresentation->getCurrentState().get(), m_shape.get(),
		&m_oldVolume, m_aabbThreshold);
	m_aabb = m_shape->getBoundingBox();

	if (m_previousShape != nullptr)
	{
		updateShapeFromOdeState(*physicsRepresentation->getPreviousState().get(), m_previousShape.get(),
			&m_previousOldVolume, m_aabbThreshold);
		m_aabb.extend(m_previousShape->getBoundingBox());
	}
}


void DeformableCollisionRepresentation::updateDcdData()
{
	// Already updated in updateShapeData, above.
}

void DeformableCollisionRepresentation::updateCcdData(double interval)
{
	auto physicsRepresentation = m_deformable.lock();
	SURGSIM_ASSERT(nullptr != physicsRepresentation) <<
			"Failed to update. The DeformableCollisionRepresentation either was not attached to a "
			"Physics::Representation or the Physics::Representation has expired.";

	// We should only need to update the previous state's shape & AABB once per CCD loop, right?
	// And we already did so in updateShapeData, above.
	//updateShapeFromOdeState(*physicsRepresentation->getPreviousState().get(), m_previousShape.get(),
	//	&m_previousOldVolume, m_aabbThreshold);

	updateShapeFromOdeState(*physicsRepresentation->getCurrentState().get(), m_shape.get(),
		&m_oldVolume, m_aabbThreshold);
	m_aabb.extend(m_shape->getBoundingBox());

	Math::PosedShape<std::shared_ptr<Math::Shape>> posedShapeFirst(m_previousShape, Math::RigidTransform3d::Identity());
	Math::PosedShape<std::shared_ptr<Math::Shape>> posedShapeSecond(m_shape, Math::RigidTransform3d::Identity());
	Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeMotion(posedShapeFirst, posedShapeSecond);
	setPosedShapeMotion(posedShapeMotion);

	// HS-2-Mar-2016
	// #todo Add AABB tree for the posedShapeMotion (i.e. that is the tree where each bounding box consists of the
	// corresponding elements from posedShape1 and posedShape2
}

SurgSim::Math::Aabbd DeformableCollisionRepresentation::getBoundingBox() const
{
	return m_aabb;
}

} // namespace Physics
} // namespace SurgSim
