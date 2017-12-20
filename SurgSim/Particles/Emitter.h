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

#ifndef SURGSIM_PARTICLES_EMITTER_H
#define SURGSIM_PARTICLES_EMITTER_H

#include <cmath>
#include <memory>
#include <random>

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/RandomPointGenerator.h"


namespace SurgSim
{

namespace Framework
{
class Logger;
};

namespace Math
{
class Shape;
};

namespace Particles
{

class Representation;

/// Emitting modes of the Emitter
enum EmitMode
{
	/// Emit particles from within the shapes volume
	EMIT_MODE_VOLUME = 0,
	/// Emit particles only from the surface
	EMIT_MODE_SURFACE,
	/// The number of EmitModes
	EMIT_MODE_COUNT
};

SURGSIM_STATIC_REGISTRATION(Emitter);

/// Emitter emits particles into a ParticleSystem
class Emitter : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param name The Emitter's name
	explicit Emitter(const std::string& name);

	/// Destructor
	virtual ~Emitter();

	SURGSIM_CLASSNAME(SurgSim::Particles::Emitter);

	void update(double dt) override;

	int getTargetManagerType() const override;

	/// Set the target to emit to.
	/// \param target The ParticleSystem to emit to.
	void setTarget(const std::shared_ptr<SurgSim::Framework::Component> target);

	/// Get the target to emit to.
	/// \return The ParticleSystem to emit to.
	const std::shared_ptr<SurgSim::Framework::Component> getTarget();

	/// Set the shape of this emitter.
	/// \param shape Shape of this emitter.
	void setShape(std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Get the shape of this emitter.
	/// \return Shape of this emitter.
	std::shared_ptr<SurgSim::Math::Shape> getShape() const;

	/// Set the emit mode of this emitter.
	/// \param mode The emit mode.
	void setMode(int mode);

	/// Get the emit mode of this emitter.
	/// \return Emit mode of this emitter.
	int getMode() const;

	/// Set the emit rate of this emitter.
	/// \param rate The rate of emitting [particles/s].
	void setRate(double rate);

	/// Get the emit rate of this emitter.
	/// \return Emitting rate [particles/s].
	double getRate() const;

	/// Set the range of lifetimes of emitted particles.
	/// Each emitted particle will have a randomly chosen lifetime in the supplied range.
	/// \param range The shortest and longest lifetimes to produce.
	void setLifetimeRange(const std::pair<double, double>& range);

	/// Get the range of lifetimes of emitted particles.
	/// \return The shortest and longest lifetimes.
	std::pair<double, double> getLifetimeRange() const;

	/// Set the range of velocities of the emitted particles.
	/// Each produced particle will have a randomly chosen velocity in the supplied range.
	/// \param range The minimum and maximum velocity.
	void setVelocityRange(const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& range);

	/// Get the range of velocities of the emitted particles.
	const std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d>& getVelocityRange() const;

	/// Set the pose of the Emitter with respect to the Scene Element
	/// \param pose The pose to set the Emitter to
	virtual void setLocalPose(const SurgSim::Math::UnalignedRigidTransform3d& pose);

	/// Get the pose of the Emitter with respect to the Scene Element
	/// \return The pose of this Emitter
	virtual SurgSim::Math::UnalignedRigidTransform3d getLocalPose() const;

	/// Get the pose of the Emitter in world coordinates
	/// \return The pose of this Emitter
	virtual SurgSim::Math::RigidTransform3d getPose() const;

private:
	bool doInitialize() override;
	bool doWakeUp() override;

	// Cast Pose to unaligned for boost::any transport 
	SurgSim::Math::UnalignedRigidTransform3d getUnalignedPose() const;


	/// PointGenerator for generating random points within or on the emitter shape.
	RandomPointGenerator m_pointGenerator;

	/// The emit mode of this emitter.
	int m_mode;

	/// The emit rate of this emitter.
	double m_rate;

	/// The range of lifetimes of emitted particles.
	std::pair<double, double> m_lifetimeRange;

	/// The range of velocities of the emitted particles.
	std::pair<SurgSim::Math::Vector3d, SurgSim::Math::Vector3d> m_velocityRange;

	/// Number of particles not added during last update.
	double m_particlesNotAdded;

	///@{
	/// Random number generator and distribution used to assign random lifetimes and velocities
	std::mt19937 m_generator;
	std::uniform_real_distribution<double> m_zeroOneDistribution;
	///@}

	/// Shape of emitter.
	std::shared_ptr<SurgSim::Math::Shape> m_shape;

	/// Representation to emit to.
	std::shared_ptr<SurgSim::Particles::Representation> m_target;

	/// Local Pose of the Representation with respect to the SceneElement
	SurgSim::Math::UnalignedRigidTransform3d m_localPose;

	/// Logger used by the Emitter
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;
};

}; // namespace Particles
}; // namespace SurgSim

#endif // SURGSIM_PARTICLES_EMITTER_H
