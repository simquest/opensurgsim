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

#ifndef SURGSIM_COLLISION_RIGIDCOLLISIONREPRESENTATION_H
#define SURGSIM_COLLISION_RIGIDCOLLISIONREPRESENTATION_H

#include <memory>

#include <SurgSim/Collision/Representation.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Shape.h>

namespace SurgSim
{
namespace Collision
{

/// Collision Representation class that wraps a RigidRepresentation, this can be used to strictly tie the
/// Collision Representation to the Rigid, so even if the shape of the rigid changes, the collision representation
/// will use the appropriate shape.
class RigidCollisionRepresentation : public Representation
{
public:

	/// Constructor
	RigidCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> representation);

	/// Destructor
	virtual ~RigidCollisionRepresentation();

	///@{
	/// Implementations of virtual functions from Collision Representation
	virtual int getShapeType() const override;
	virtual const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override;
	virtual std::shared_ptr<SurgSim::Physics::Representation> getPhysicsRepresentation() override;
	///@}

private:

	std::weak_ptr<SurgSim::Physics::RigidRepresentationBase> m_physicsRepresentation;
};

}; // namespace Collision
}; // namespace SurgSim

#endif
