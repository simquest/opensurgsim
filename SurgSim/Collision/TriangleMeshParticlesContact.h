// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_COLLISION_TRIANGLEMESHPARTICLESCONTACT_H
#define SURGSIM_COLLISION_TRIANGLEMESHPARTICLESCONTACT_H

#include <memory>

#include "SurgSim/Collision/ShapeShapeContactCalculation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/ParticlesShape.h"


namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between a triangle mesh and particles
class TriangleMeshParticlesContact : public ShapeShapeContactCalculation<Math::MeshShape, Math::ParticlesShape>
{
public:
	using ContactCalculation::calculateDcdContact;

	std::list<std::shared_ptr<Contact>> calculateDcdContact(
										 const Math::MeshShape& mesh,
										 const Math::RigidTransform3d& meshPose,
										 const Math::ParticlesShape& particles,
										 const Math::RigidTransform3d& particlesPose) const override;

	std::pair<int, int> getShapeTypes() override;

};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_TRIANGLEMESHPARTICLESCONTACT_H
