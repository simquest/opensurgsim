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
#include "SurgSim/Math/OctreeShape.h"

namespace SurgSim
{
namespace Math
{
class BoxShape;
};

namespace Collision
{

class CollisionPair;
class ShapeCollisionRepresentation;

/// Class to calculate intersections between an Octree and other shapes
class OctreeDcdContact : public ContactCalculation
{
public:

	/// Constructor.
	/// \param calculator The contact calculator to use on each octree node
	explicit OctreeDcdContact(std::shared_ptr<ContactCalculation> calculator);

	/// Function that returns the shapes between which this class performs collision detection.
	/// \return A pair of shape type ids
	std::pair<int, int> getShapeTypes() override;

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

	/// The contact calculator to use on each octree node
	const std::shared_ptr<ContactCalculation> m_calculator;

	/// The shape types that this contact calculation handles
	std::pair<int, int> m_shapeTypes;

	/// Collision Representation used to detect contacts with each octree node
	std::shared_ptr<ShapeCollisionRepresentation> m_nodeCollisionRepresentation;

	/// Enable a Vector3d to be used as a key in an unordered map.
	class Vector3dHash
	{
	public:
		size_t operator()(const SurgSim::Math::Vector3d& id) const;
	};

	/// The shapes used for the contact calculations are cached for performance.
	std::unordered_map<SurgSim::Math::Vector3d, std::shared_ptr<SurgSim::Math::Shape>, Vector3dHash> m_shapes;
};

};
};



#endif // SURGSIM_COLLISION_OCTREEDCDCONTACT_H
