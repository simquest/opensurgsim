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

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/RigidTransform.h"


namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between Boxes and Capsules
class BoxCapsuleDcdContact : public ContactCalculation
{
public:

	/// Constructor.
	BoxCapsuleDcdContact();

	using ContactCalculation::calculateContact;

	/// Calculate the contacts using the typed shapes directly
	/// \param boxShape the box shape
	/// \param boxPose the pose of the box
	/// \param capsuleShape the capsule shape
	/// \param capsulePose the pose of the capsule
	/// \return a list of contacts between the shapes, if any
	std::list<std::shared_ptr<Contact>> calculateContact(
			const SurgSim::Math::BoxShape& boxShape, const SurgSim::Math::RigidTransform3d& boxPose,
			const SurgSim::Math::CapsuleShape& capsuleShape, const SurgSim::Math::RigidTransform3d& capsulePose);

	std::pair<int,int> getShapeTypes() override;

private:
	void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;

};

}; // namespace Collision
}; // namespace SurgSim

#endif
