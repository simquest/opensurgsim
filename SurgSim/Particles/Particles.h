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

#ifndef SURGSIM_PARTICLES_PARTICLES_H
#define SURGSIM_PARTICLES_PARTICLES_H

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/Vector.h"


namespace SurgSim
{
namespace Particles
{

struct ParticleData
{
	double lifetime;
	Math::Vector3d velocity;

	/// Equality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are considered equivalent.
	bool operator==(const ParticleData& rhs) const
	{
		return lifetime == rhs.lifetime && velocity.isApprox(rhs.velocity);
	}

	/// Inequality operator.
	/// \param	rhs	The right hand side.
	/// \return	true if the parameters are not considered equivalent.
	bool operator!=(const ParticleData& rhs) const
	{
		return !((*this) == rhs);
	}
};

typedef DataStructures::Vertices<ParticleData> Particles;
typedef Particles::VertexType Particle;

};
};
#endif //SURGSIM_PARTICLES_PARTICLES_H


