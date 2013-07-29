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

#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{
namespace Physics
{

/// CollisionRepresentation class that wraps a RigidRepresentation, this can be used to strictly tie the
/// CollisionRepresentation to the Rigid, so even if the shape of the rigid changes, the collision representation
/// will use the appropriate shape.
class RigidCollisionRepresentation : public CollisionRepresentation
{
public:

	/// Constructor
	explicit RigidCollisionRepresentation(
		const std::string& name,
		std::shared_ptr<RigidRepresentation> representation);

	/// Destructor
	virtual ~RigidCollisionRepresentation();

	///@{
	/// Implementations of virtual functions from CollisionRepresentation
	virtual int getShapeType() const override;
	virtual const std::shared_ptr<RigidShape> getShape() const override;
	virtual const SurgSim::Math::RigidTransform3d& getPose() const override;
	///@}

private:

	/// \note HS-2013-may-30 Should this be a std::weak_ptr ?
	std::shared_ptr<RigidRepresentation> m_localRepresentation;

};

}; // Physics
}; // SurgSim

#endif
