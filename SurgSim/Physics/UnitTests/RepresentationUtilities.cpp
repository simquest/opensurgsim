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

#include <SurgSim/Physics/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Physics/UnitTests/MockCollisionRepresentation.h>
#include <SurgSim/Physics/RigidShapeCollisionRepresentation.h>

#include <SurgSim/Physics/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>

#include <SurgSim/Physics/Shapes.h>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

std::shared_ptr<CollisionRepresentation> makeSphereRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const double& radius,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<RigidShape> sphere = std::make_shared<SphereShape>(radius);
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestSphereShapeCollisionRep",
		sphere,
		rotation,
		position,
		representation);
}

std::shared_ptr<CollisionRepresentation> makeDoubleSidedPlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<RigidShape> plane = std::make_shared<DoubleSidedPlaneShape>();
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestDoubleSidedPlaneCollisionRep",
		plane,
		rotation,
		position,
		representation);
}

std::shared_ptr<CollisionRepresentation> makePlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<RigidShape> plane = std::make_shared<PlaneShape>();
	std::shared_ptr<CollisionRepresentation> rep =
		std::make_shared<MockCollisionRepresentation>(
		"TestPlaneRepresentation",
		plane,
		rotation,
		position,
		representation);

	return rep;
}

std::shared_ptr<CollisionRepresentation> makeCapsuleRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const double& radius,
	const double& length,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<RigidShape> capsule= std::make_shared<CapsuleShape>(radius, length);
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestCapsuleShapeCollisionRep",
		capsule,
		rotation,
		position,
		representation);
}

}; // Physics
}; // SurgSim
