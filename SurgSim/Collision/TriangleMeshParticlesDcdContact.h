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

#ifndef SURGSIM_COLLISION_TRIANGLEMESHPARTICLESDCDCONTACT_H
#define SURGSIM_COLLISION_TRIANGLEMESHPARTICLESDCDCONTACT_H

#include <memory>

#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/ParticlesShape.h"


namespace SurgSim
{
namespace Collision
{

class CollisionPair;

/// Class to calculate intersections between a triangle mesh and particles
class TriangleMeshParticlesDcdContact : public ContactCalculation
{
public:
	/// Constructor.
	TriangleMeshParticlesDcdContact();

	using ContactCalculation::calculateContact;

	/// Calculate the contacts using the typed shapes directly
	/// \param mesh the mesh shape
	/// \param particles the particles shape
	/// \return a list of contacts between the shapes, if any
	std::list<std::shared_ptr<Contact>> calculateContact(const Math::MeshShape& mesh,
			const Math::ParticlesShape& particles);

	std::pair<int, int> getShapeTypes() override;

private:
	void doCalculateContact(std::shared_ptr<CollisionPair> pair) override;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_TRIANGLEMESHPARTICLESDCDCONTACT_H
