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
#include "SurgSim/Framework/ObjectFactory.h"


namespace SurgSim
{

namespace Math
{
class Shape;
class ParticlesShape;
};

namespace Particles
{

class Representation;

SURGSIM_STATIC_REGISTRATION(ParticlesCollisionRepresentation);

/// A Collision Representation that can be attached to a Particle Representation
class ParticlesCollisionRepresentation : public SurgSim::Collision::Representation
{
public:

	/// Constructor
	/// \param name Name of the Representation
	explicit ParticlesCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~ParticlesCollisionRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Particles::ParticlesCollisionRepresentation);

	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;

	int getShapeType() const override;

	void updateShapeData() override;

	/// Sets the particle representation this collision representation is connected
	/// \param representation The paticle representation
	void setParticleRepresentation(std::shared_ptr<SurgSim::Particles::Representation> representation);

	/// Gets the particle representation this collision representation is connected
	/// \return The particle representation
	const std::shared_ptr<SurgSim::Particles::Representation> getParticleRepresentation() const;

	/// Set the particles' radius
	/// \param radius the radius being set to all particles
	void setParticleRadius(double radius);

	/// Get the radius of the particles
	/// \return The particles' radius
	double getParticleRadius() const;

private:
	bool doInitialize() override;

	bool doWakeUp() override;

	/// Shape used for collision detection
	std::shared_ptr<SurgSim::Math::ParticlesShape> m_shape;

	/// Reference to the particle representation driving changes to this collision representation
	std::weak_ptr<SurgSim::Particles::Representation> m_particleRepresentation;
};

};
};

#endif // SURGSIM_PARTICLES_PARTICLESCOLLISIONREPRESENTATION_H

