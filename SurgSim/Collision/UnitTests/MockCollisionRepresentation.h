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

#ifndef SURGSIM_COLLISION_UNITTESTS_MOCKCOLLISIONREPRESENTATION_H
#define SURGSIM_COLLISION_UNITTESTS_MOCKCOLLISIONREPRESENTATION_H

#include <SurgSim/Collision/CollisionRepresentation.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Shape.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

using SurgSim::Math::Shape;

namespace SurgSim
{
namespace Collision
{

class Representation;

/// Class to wrap a plain Shape for use as a CollisionRepresentation, just delegate
/// the access to the various Shape methods
class MockCollisionRepresentation : public CollisionRepresentation
{
public:
	MockCollisionRepresentation(
		const std::string& name,
		const std::shared_ptr<Shape>& shape,
		const SurgSim::Math::Quaterniond& quat,
		const SurgSim::Math::Vector3d& translation,
		std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr);

	MockCollisionRepresentation(
		const std::string& name,
		const std::shared_ptr<Shape>& shape,
		const SurgSim::Math::RigidTransform3d& pose,
		std::shared_ptr<SurgSim::Physics::Representation> representation = nullptr);

	virtual ~MockCollisionRepresentation();

	virtual int getShapeType() const override;

	virtual const std::shared_ptr<Shape> getShape() const override;

	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;

private:
	std::shared_ptr<Shape> m_shape;
	SurgSim::Math::RigidTransform3d m_transform;
};


}; // namespace Collision
}; // namespace SurgSim

#endif
