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

#ifndef SURGSIM_COLLISION_RIGIDSHAPECOLLISIONREPRESENTATION_H
#define SURGSIM_COLLISION_RIGIDSHAPECOLLISIONREPRESENTATION_H

#include <SurgSim/Collision/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidShape.h>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Collision
{

/// Use a RigidShape as a CollisionRepresentation, any SurgSim::Physics::Representation can
/// be used as a backing representation
class RigidShapeCollisionRepresentation : public CollisionRepresentation
{
public:
	RigidShapeCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<SurgSim::Physics::RigidShape> shape,
		std::shared_ptr<SurgSim::Physics::Representation> representation);

	virtual ~RigidShapeCollisionRepresentation();

	virtual int getShapeType() const override;

	virtual const std::shared_ptr<SurgSim::Physics::RigidShape> getShape() const override;

private:
	std::shared_ptr<SurgSim::Physics::RigidShape> m_shape;
};


}; // namespace Collision
}; // namespace SurgSim

#endif