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
	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param pair The symmetric pair that is under consideration.
	void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;

	/// Calculate the collision between a specific octree node and a shape
	/// This function will check for contact between the node and shape. If
	/// contact is found, this function will be called on each of the
	/// node's children. Once a leaf node is reached, contacts are added to the
	/// CollisionPair.
	/// \param node the octree node to collide with
	/// \param [in,out] pair the collision pair that is under consideration
	/// \param nodePath the path of the current node
	void calculateContactWithNode(std::shared_ptr<const SurgSim::Math::OctreeShape::NodeType> node,
								  std::shared_ptr<CollisionPair> pair,
								  std::shared_ptr<SurgSim::DataStructures::OctreePath> nodePath);

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
