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

#include <SurgSim/Physics/Actor.h>

#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

/// The RigidActorBase class defines the base class for
/// all rigid motion based actors (fixed, rigid body, rigid body + vtc,...)
class RigidActorBase : public Actor
{
public:
	typedef SurgSim::Math::RigidTransform3d RigidTransform3d;

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
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_RIGIDACTORBASE_H
