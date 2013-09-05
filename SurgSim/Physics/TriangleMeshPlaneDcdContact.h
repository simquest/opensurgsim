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

#ifndef SURGSIM_PHYSICS_TRIANGLEMESHDCDCONTACT_H
#define SURGSIM_PHYSICS_TRIANGLEMESHDCDCONTACT_H

#include <memory>

#include <SurgSim/Framework/ReuseFactory.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Math/Geometry.h>

namespace SurgSim
{
namespace Physics
{

/// Class to calculate intersections between a triangle mesh and a plan
template <class VertexType, class EdgeType, class TriangleType>
class TriangleMeshPlaneDcdContact : public ContactCalculation
{
public:
	
	explicit TriangleMeshPlaneDcdContact()
	{
	}

	virtual std::pair<int, int> getShapeTypes() override
	{
		return std::pair<int, int> (RIGID_SHAPE_TYPE_MESH, RIGID_SHAPE_TYPE_PLANE);
	}

private:
	/// Calculate the actual contact between two shapes of the given CollisionPair.
	/// \param pair The symmetric pair that is under consideration.
	virtual void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;

};

};
};

#include <SurgSim/Physics/TriangleMeshPlaneDcdContact-inl.h>


#endif
