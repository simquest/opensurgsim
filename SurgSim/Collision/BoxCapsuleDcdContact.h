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

#ifndef SURGSIM_COLLISION_BOXCAPSULEDCDCONTACT_H
#define SURGSIM_COLLISION_BOXCAPSULEDCDCONTACT_H

#include <memory>

#include "SurgSim/Collision/ShapeShapeContactCalculation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Collision
{

/// Class to calculate intersections between Boxes and Capsules
class BoxCapsuleDcdContact : public ShapeShapeContactCalculation<Math::BoxShape, Math::CapsuleShape>
{
public:
	using ContactCalculation::calculateContact;

	std::list<std::shared_ptr<Contact>> calculateDcdContact(
										 const SurgSim::Math::BoxShape& boxShape,
										 const SurgSim::Math::RigidTransform3d& boxPose,
										 const SurgSim::Math::CapsuleShape& capsuleShape,
										 const SurgSim::Math::RigidTransform3d& capsulePose) const override;

	std::pair<int, int> getShapeTypes() override;
};

}; // namespace Collision
}; // namespace SurgSim

#endif
