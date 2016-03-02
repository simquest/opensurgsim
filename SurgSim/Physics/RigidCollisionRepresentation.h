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

#ifndef SURGSIM_PHYSICS_RIGIDCOLLISIONREPRESENTATION_H
#define SURGSIM_PHYSICS_RIGIDCOLLISIONREPRESENTATION_H

#include <memory>

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{
namespace Physics
{
class RigidRepresentationBase;

SURGSIM_STATIC_REGISTRATION(RigidCollisionRepresentation);

/// Collision Representation class that wraps a RigidRepresentation, this can be used to strictly tie the
/// Collision Representation to the Rigid, so even if the shape of the rigid changes, the collision representation
/// will use the appropriate shape.
class RigidCollisionRepresentation : public SurgSim::Collision::Representation
{
public:
	/// Constructor
	/// \param	name	The name of rigid collision representation
	explicit RigidCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~RigidCollisionRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Physics::RigidCollisionRepresentation);

	/// Get the pose of the representation
	/// \return The pose of this representation
	SurgSim::Math::RigidTransform3d getPose() const override;

	/// Get the shape type id
	/// \return The unique type of the shape, used to determine which calculation to use.
	int getShapeType() const override;

	/// Get the shape
	/// \return The actual shape used for collision.
	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;

	/// Set the shape
	/// The default is to use the shape of the Rigid Representation, this
	/// will override that shape.
	/// \param shape The actual shape used for collision, if nullptr the Rigid Representation shape will be used.
	void setShape(std::shared_ptr<SurgSim::Math::Shape> shape);

	/// Set rigid representation
	/// \param	representation	The rigid representation
	void setRigidRepresentation(std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> representation);

	/// Gets physics representation.
	/// \return	The physics representation.
	std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> getRigidRepresentation();

	void updateShapeData() override;
	void updateDcdData() override;
	void updateCcdData() override;

private:
	std::weak_ptr<SurgSim::Physics::RigidRepresentationBase> m_physicsRepresentation;
	std::shared_ptr<SurgSim::Math::Shape> m_shape;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_PHYSICS_RIGIDCOLLISIONREPRESENTATION_H
