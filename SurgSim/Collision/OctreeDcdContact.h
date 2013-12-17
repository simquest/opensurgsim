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
#include "SurgSim/DataStructures/OctreeNode.h"

namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between an Octree and other shapes
/// \tparam Data The data stored in each octree node
template <class Data>
class OctreeDcdContact : public ContactCalculation
{
public:

	/// Constructor.
	/// \param calculator The contact calculator to use on each octree node
	OctreeDcdContact(std::shared_ptr<ContactCalculation> calculator);

	/// Function that returns the shapes between which this class performs collision detection.
	/// \return A pair of shape type ids
	virtual std::pair<int, int> getShapeTypes() override;

private:
	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param pair The symmetric pair that is under consideration.
	virtual void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;

	/// Calculate the collision between a specific octree node and a shape
	/// \param node the octree node to collide with
	/// \param pair the collision pair that is under consideration
	/// \param nodePath the path of the current node
	void calculateContactWithNode(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> node,
			std::shared_ptr<CollisionPair> pair,
			std::shared_ptr<SurgSim::Math::OctreePath> nodePath);

	/// The contact calculator to use on each octree node
	const std::shared_ptr<ContactCalculation> m_calculator;

	/// The shape types that this contact caculation handles
	std::pair<int, int> m_shapeTypes;
};

};
};

#include <SurgSim/Collision/OctreeDcdContact-inl.h>


#endif // SURGSIM_COLLISION_OCTREEDCDCONTACT_H
