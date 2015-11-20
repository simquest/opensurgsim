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

#ifndef SURGSIM_COLLISION_SPHERESPHERECONTACT_H
#define SURGSIM_COLLISION_SPHERESPHERECONTACT_H

#include <memory>

#include "SurgSim/Collision/ShapeShapeContactCalculation.h"
#include "SurgSim/Math/SphereShape.h"

namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between spheres
class SphereSphereContact : public ShapeShapeContactCalculation<Math::SphereShape, Math::SphereShape>
{
public:
	using ContactCalculation::calculateDcdContact;

	std::list<std::shared_ptr<Contact>> calculateDcdContact(
										 const Math::SphereShape& sphere1,
										 const Math::RigidTransform3d& sphere1Pose,
										 const Math::SphereShape& sphere2,
										 const Math::RigidTransform3d& sphere2Pose) const override;

	std::pair<int, int> getShapeTypes() override;

};

}; // namespace Collision
}; // namespace SurgSim

#endif
