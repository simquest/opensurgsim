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

#ifndef SURGSIM_PARTICLES_PARTICLESCOLLISIONREPRESENTATION_H
#define SURGSIM_PARTICLES_PARTICLESCOLLISIONREPRESENTATION_H

#include <memory>
#include <string>

#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{

namespace Math
{
class Shape;
class ParticlesShape;
};

namespace Particles
{

class ParticleSystemRepresentation;

class ParticlesCollisionRepresentation : public SurgSim::Collision::Representation
{
public:

	/// Constructor
	/// \param name Name of the Representation
	explicit ParticlesCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~ParticlesCollisionRepresentation();

	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;

	int getShapeType() const override;

	void update(const double& dt) override;

	/// Sets the particle system to which this collision representation is connected
	/// \param representation The paticle system that will be used to update the contained mesh
	void setParticleSystem(std::shared_ptr<ParticleSystemRepresentation> representation);

	/// \return The particle system that this collision representation is connected
	const std::shared_ptr<ParticleSystemRepresentation> getParticleSystem() const;

private:
	bool doInitialize() override;
	bool doWakeUp() override;

	/// Shape used for collision detection
	std::shared_ptr<SurgSim::Math::ParticlesShape> m_shape;

	/// Reference to the particle system driving changes to this collision representation
	std::weak_ptr<ParticleSystemRepresentation> m_particleSystem;
};

};
};

#endif // SURGSIM_PARTICLES_PARTICLESCOLLISIONREPRESENTATION_H

