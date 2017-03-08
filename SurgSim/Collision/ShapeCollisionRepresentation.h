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

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Framework/ObjectFactory.h"

namespace SurgSim
{
namespace Math
{
class Shape;
}

namespace Collision
{
SURGSIM_STATIC_REGISTRATION(ShapeCollisionRepresentation);

/// Use a Shape as a Collision Representation, any SurgSim::Physics::Representation can
/// be used as a backing representation
class ShapeCollisionRepresentation : public Representation
{
public:
	/// Constructor
	explicit ShapeCollisionRepresentation(const std::string& name);

	/// Destructor
	virtual ~ShapeCollisionRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Collision::ShapeCollisionRepresentation);

	int getShapeType() const override;

	void setLocalPose(const SurgSim::Math::UnalignedRigidTransform3d& pose) override;

	// Set the shape to be used in this representation
	// \param shape Shape to be used in this representation.
	void setShape(const std::shared_ptr<SurgSim::Math::Shape>& shape);

	const std::shared_ptr<SurgSim::Math::Shape> getShape() const override;

protected:
	bool doInitialize() override;

private:
	// Shape used by this representation
	std::shared_ptr<SurgSim::Math::Shape> m_shape;
};

}; // namespace Collision
}; // namespace SurgSim

#endif // SURGSIM_COLLISION_SHAPECOLLISIONREPRESENTATION_H
