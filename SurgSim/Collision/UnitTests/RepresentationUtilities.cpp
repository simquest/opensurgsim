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

#include <SurgSim/Collision/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Collision/UnitTests/MockCollisionRepresentation.h>
#include <SurgSim/Collision/ShapeCollisionRepresentation.h>

#include <SurgSim/Collision/CollisionRepresentation.h>
#include <SurgSim/Physics/RigidRepresentation.h>

#include <SurgSim/Math/Shapes.h>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::shared_ptr<SurgSim::Collision::CollisionRepresentation> makeSphereRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const double& radius,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<Shape> sphere = std::make_shared<SurgSim::Math::SphereShape>(radius);
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestSphereShapeCollisionRep",
		sphere,
		rotation,
		position,
		representation);
}

std::shared_ptr<SurgSim::Collision::CollisionRepresentation> makeDoubleSidedPlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<Shape> plane = std::make_shared<SurgSim::Math::DoubleSidedPlaneShape>();
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestDoubleSidedPlaneCollisionRep",
		plane,
		rotation,
		position,
		representation);
}

std::shared_ptr<SurgSim::Collision::CollisionRepresentation> makePlaneRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<Shape> plane = std::make_shared<SurgSim::Math::PlaneShape>();
	return  std::make_shared<MockCollisionRepresentation>(
		"TestPlaneRepresentation",
		plane,
		rotation,
		position,
		representation);
}

std::shared_ptr<SurgSim::Collision::CollisionRepresentation> makeCapsuleRepresentation(
	std::shared_ptr<SurgSim::Physics::Representation> representation,
	const double& length,
	const double& radius,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<Shape> capsule = std::make_shared<SurgSim::Math::CapsuleShape>(length, radius);
	return 	std::make_shared<MockCollisionRepresentation>(
		"TestCapsuleShapeCollisionRep",
		capsule,
		rotation,
		position,
		representation);
}

}; // Collision
}; // SurgSim
