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

#ifndef SURGSIM_PHYSICS_RIGIDSHAPECOLLISIONREPRESENTATION_H
#define SURGSIM_PHYSICS_RIGIDSHAPECOLLISIONREPRESENTATION_H

#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidShape.h>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Physics
{

/// Class to wrap a plain RigidShape for use as a CollisionRepresentation, just delegate
/// the access to the various RigidShape methods
class RigidShapeCollisionRepresentation : public CollisionRepresentation
{
public:
	RigidShapeCollisionRepresentation(
		const std::shared_ptr<RigidShape>& shape,
		const SurgSim::Math::Quaterniond& quat,
		const SurgSim::Math::Vector3d& translation);

	RigidShapeCollisionRepresentation(
		const std::shared_ptr<RigidShape>& shape,
		const SurgSim::Math::RigidTransform3d& pose);

	virtual ~RigidShapeCollisionRepresentation();

	virtual int getShapeType() const;

	virtual const std::shared_ptr<SurgSim::Physics::RigidShape> getShape() const;

	virtual const SurgSim::Math::RigidTransform3d& getCurrentPose() const;

private:
	std::shared_ptr<RigidShape> m_shape;
	SurgSim::Math::RigidTransform3d m_transform;
};


}; // Physics
}; // SurgSim

#endif