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

#ifndef SURGSIM_PHYSICS_RIGIDACTORCOLLISIONREPRESENTATION_H
#define SURGSIM_PHYSICS_RIGIDACTORCOLLISIONREPRESENTATION_H

#include <memory>

#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/Actors/RigidActor.h>
#include <SurgSim/Physics/Actors/RigidShape.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{
namespace Physics
{

/// CollisionRepresentation class that wraps a RigidActor
class RigidActorCollisionRepresentation : public CollisionRepresentation
{
public:

	/// Constructor
	explicit RigidActorCollisionRepresentation(std::shared_ptr<RigidActor> actor);

	/// Destructor
	virtual ~RigidActorCollisionRepresentation();

	///@{
	/// Implementations of pure virtual functions
	virtual int getShapeType() const;
	virtual const std::shared_ptr<RigidShape> getShape() const;
	virtual const SurgSim::Math::RigidTransform3d& getLocalToWorldTransform() const;
	///@}

private:

	std::shared_ptr<RigidActor> m_actor;
};

}; // Physics
}; // SurgSim

#endif