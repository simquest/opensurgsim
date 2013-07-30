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

#include <SurgSim/Framework/Representation.h>

#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{

namespace Physics
{

class RigidShape;
class Representation;

/// Wrapper class to use for the collision operation, handles its enclosed shaped
/// and a possible local to global coordinate system transform, if the physics representation
/// is a nullptr or a has gone out of scope ASSERT's will be triggered
class CollisionRepresentation : public SurgSim::Framework::Representation
{
public:

	///@{
	/// Constructors
	explicit CollisionRepresentation(const std::string& name);
	CollisionRepresentation(const std::string& name, std::shared_ptr<SurgSim::Physics::Representation> representation);
	///@}

	virtual ~CollisionRepresentation();

	/// \return The unique type of the shape, used to determine which calculation to use.
	virtual int getShapeType() const = 0;

	/// \return The actual shape used for collision.
	virtual const std::shared_ptr<SurgSim::Physics::RigidShape> getShape() const = 0;

	/// Overridden from Representation, this is not applicable for a CollisionRepresentation
	/// the program will abort if this function is called
	/// \param pose will be ignored
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// \return Transformation to transform the shape into world coordinates
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;

	/// Overridden from Representation, this is not applicable for a CollisionRepresentation
	/// the program will abort if this function is called
	/// \param pose will be ignored
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose) override;


	/// Overridden from Representation, this will delegate to the Physics::Representation contained
	/// in this class
	/// \return Transformation of the contained Representation
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const override;

	/// Gets physics representation.
	/// \return	The physics representation.
	std::shared_ptr<SurgSim::Physics::Representation> getPhysicsRepresentation();

protected:
	std::weak_ptr<SurgSim::Physics::Representation> m_physicsRepresentation;

	/// Sets the physics representation for this collision representation.
	/// \param	physicsRepresentation	The physics representation.
	void setPhysicsRepresentation(const std::shared_ptr<SurgSim::Physics::Representation>& physicsRepresentation);
	
};


}; // namespace Physics
}; // namespace SurgSim

#endif