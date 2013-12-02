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
#include <SurgSim/Collision/ShapeCollisionRepresentation.h>

#include <SurgSim/Collision/Representation.h>
#include <SurgSim/Physics/RigidRepresentation.h>

#include <SurgSim/Math/Shapes.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::Quaterniond;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::shared_ptr<SurgSim::Collision::Representation> makeSphereRepresentation(
	const double& radius,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<SurgSim::Math::Shape> sphere = std::make_shared<SurgSim::Math::SphereShape>(radius);
	return 	std::make_shared<ShapeCollisionRepresentation>(
		"TestSphereShapeCollisionRep", sphere, SurgSim::Math::makeRigidTransform(rotation, position));
}

std::shared_ptr<SurgSim::Collision::Representation> makeDoubleSidedPlaneRepresentation(
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<SurgSim::Math::Shape> plane = std::make_shared<SurgSim::Math::DoubleSidedPlaneShape>();
	return 	std::make_shared<ShapeCollisionRepresentation>(
		"TestDoubleSidedPlaneCollisionRep", plane, SurgSim::Math::makeRigidTransform(rotation, position));
}

std::shared_ptr<SurgSim::Collision::Representation> makePlaneRepresentation(
	const Quaterniond& rotation,
	const Vector3d& position)
{
	std::shared_ptr<SurgSim::Math::Shape> plane = std::make_shared<SurgSim::Math::PlaneShape>();
	return  std::make_shared<ShapeCollisionRepresentation>(
		"TestPlaneRepresentation", plane, SurgSim::Math::makeRigidTransform(rotation, position));
}

std::shared_ptr<SurgSim::Collision::Representation> makeCapsuleRepresentation(
	const double& length,
	const double& radius,
	const Quaterniond& rotation,
	const Vector3d& position)
{

	std::shared_ptr<SurgSim::Math::Shape> capsule = std::make_shared<SurgSim::Math::CapsuleShape>(length, radius);
	return 	std::make_shared<ShapeCollisionRepresentation>(
		"TestCapsuleShapeCollisionRep", capsule, SurgSim::Math::makeRigidTransform(rotation, position));
}

}; // Collision
}; // SurgSim
