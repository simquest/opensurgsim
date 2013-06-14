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

#ifndef SURGSIM_PHYSICS_COLLISIONREPRESENTATION_H
#define SURGSIM_PHYSICS_COLLISIONREPRESENTATION_H

#include <memory>

#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

class RigidShape;

/// Wrapper class to use for the collision operation, handles its enclosed shaped
/// and a possible local to global coordinate system transform
class CollisionRepresentation
{
public:

	virtual ~CollisionRepresentation() {}
	/// \return The unique type of the shape, used to determine which calculation to use.
	virtual int getShapeType() const = 0;

	/// \return The actual shape used for collision.
	virtual const std::shared_ptr<SurgSim::Physics::RigidShape> getShape() const = 0;

	/// \return Transformation to transform the shape into world coordinates
	virtual const SurgSim::Math::RigidTransform3d& getCurrentPose() const = 0;

};


}; // namespace Physics
}; // namespace SurgSim

#endif