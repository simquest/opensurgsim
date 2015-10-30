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

#ifndef SURGSIM_COLLISION_OCTREEDCDCONTACT_H
#define SURGSIM_COLLISION_OCTREEDCDCONTACT_H

#include <memory>

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/OctreeShape.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Collision
{

class CollisionPair;
class ShapeCollisionRepresentation;

/// Abstract base class to calculate intersections between an Octree and other shapes
///
/// Derived classes handle the calculation for specific shape types (ie
/// OctreeSphereDcdContact).
class OctreeDcdContact : public ContactCalculation
{
protected:
	/// Do the calculation between an octree node (BoxShape) and the other shape
	/// \param boxShape the box shape
	/// \param boxPose the pose of the box
	/// \param otherShape the other shape
	/// \param otherPose the pose of the other shape
	/// \return a list of contacts between the shapes, if any
	virtual std::list<std::shared_ptr<Contact>> boxContactCalculation(
				const SurgSim::Math::BoxShape& boxShape, const SurgSim::Math::RigidTransform3d& boxPose,
				const SurgSim::Math::Shape& otherShape, const SurgSim::Math::RigidTransform3d& otherPose) = 0;

private:
	std::list<std::shared_ptr<Contact>> doCalculateContact(
										 const std::shared_ptr<Math::Shape>& shape1,
										 const Math::RigidTransform3d& pose1,
										 const std::shared_ptr<Math::Shape>& shape2,
										 const Math::RigidTransform3d& pose2) override;

	/// Calculate the collision between a specific octree node and a shape
	/// This function will check for contact between the node and shape. If
	/// contact is found, this function will be called on each of the
	/// node's children. Once a leaf node is reached, contacts are added to the
	/// CollisionPair.
	/// \param node the octree node to collide with
	/// \param octreePose the pose of the octree shape
	/// \param shape the shape that the octree is colliding with
	/// \param shapePose the pose of the shape
	/// \param nodePath the NodePath of the current octree node
	/// \param result [in,out] all generated contacts are agreggated here
	/// \param nodePath [in,out] the path of the current node
	void calculateContactWithNode(
		std::shared_ptr<const SurgSim::Math::OctreeShape::NodeType> node,
		Math::RigidTransform3d octreePose,
		const std::shared_ptr<Math::Shape>& shape,
		const Math::RigidTransform3d& shapePose,
		SurgSim::DataStructures::OctreePath* nodePath,
		std::list<std::shared_ptr<Contact>>* result);

	/// Enable a Vector3d to be used as a key in an unordered map.
	class Vector3dHash
	{
	public:
		size_t operator()(const SurgSim::Math::Vector3d& id) const;
	};

	/// The shapes used for the contact calculations are cached for performance.
	std::unordered_map<SurgSim::Math::Vector3d, std::shared_ptr<SurgSim::Math::BoxShape>, Vector3dHash> m_shapes;
};

};
};



#endif // SURGSIM_COLLISION_OCTREEDCDCONTACT_H
