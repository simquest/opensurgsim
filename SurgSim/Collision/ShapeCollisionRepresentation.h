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

#ifndef SURGSIM_COLLISION_SHAPECOLLISIONREPRESENTATION_H
#define SURGSIM_COLLISION_SHAPECOLLISIONREPRESENTATION_H

#include <SurgSim/Collision/Representation.h>
#include <SurgSim/Math/Shape.h>


namespace SurgSim
{
namespace Collision
{

/// Use a Shape as a Collision Representation, any SurgSim::Physics::Representation can
/// be used as a backing representation
class ShapeCollisionRepresentation : public Representation
{
public:
	/// Constructor
	ShapeCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<SurgSim::Math::Shape> shape,
		const SurgSim::Math::RigidTransform3d& pose);

	/// Destructor
	virtual ~ShapeCollisionRepresentation();

	/// Overridden from Representation
	/// \param pose
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// \return Transformation to transform the shape into world coordinates
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;

	///@{
	/// Implementations of virtual functions from Collision Representation
	virtual int getShapeType() const override;
	virtual const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;
	virtual std::shared_ptr<SurgSim::Physics::Representation> getPhysicsRepresentation() override;
	///@}

private:
	std::shared_ptr<SurgSim::Math::Shape> m_shape;
	SurgSim::Math::RigidTransform3d m_pose;
};


}; // namespace Collision
}; // namespace SurgSim

#endif
