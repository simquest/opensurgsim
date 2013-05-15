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

#ifndef SURGSIM_PHYSICS_RIGIDACTORBASE_H
#define SURGSIM_PHYSICS_RIGIDACTORBASE_H

#include <SurgSim/Physics/Actors/Actor.h>

#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::RigidTransform3d;

namespace SurgSim 
{
namespace Physics
{

/// The RigidActorBase class defines the base class for
/// all rigid motion based actors (fixed, rigid body, rigid body + vtc,...)
class RigidActorBase : public Actor
{
public:
	/// Constructor
	/// \param name The rigid actor's name
	explicit RigidActorBase(const std::string& name)
		: Actor(name)
	{
	}

	/// Destructor
	virtual ~RigidActorBase()
	{
	}

	/// Set the initial pose of the rigid actor
	/// \param pose The initial pose
	virtual void setInitialPose(const RigidTransform3d& pose) = 0;

	/// Get the initial pose of the rigid actor
	/// \return The initial pose
	virtual const RigidTransform3d& getInitialPose() const = 0;

	/// Set the current pose of the rigid actor
	/// \param pose The current pose
	virtual void setPose(const RigidTransform3d& pose) = 0;

	/// Get the current pose of the rigid actor
	/// \return The current pose
	virtual const RigidTransform3d& getPose() const = 0;

	/// Called before the scene does its physics update, lets the actor
	/// do some preprocessing
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt)
	{
		Actor::beforeUpdate(dt);
	};

	/// Called after beforeUpdate and prior to afterUpdate
	/// It compute the current free motion of the object using the time step dt
	/// \param dt The time step (in seconds)
	virtual void update(double dt)
	{
		Actor::update(dt);
	};

	/// Called after the scene update has concluded
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt)
	{
		Actor::afterUpdate(dt);
	};

	/// Called to reset the rigid actor to its initial/default state
	virtual void resetState()
	{
		Actor::resetState();
	};

	/// Called to reset the rigid actor parameters
	virtual void resetParameters()
	{
		Actor::resetParameters();
	};
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_RIGIDACTORBASE_H
