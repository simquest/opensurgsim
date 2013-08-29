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

#include <gtest/gtest.h>
#include <SurgSim/Physics/UnitTests/RepresentationUtilities.h>
#include <SurgSim/Collision/UnitTests/MockCollisionRepresentation.h>
#include <memory>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Physics/RigidRepresentationState.h>
#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/Physics/SphereShape.h>
#include <SurgSim/Collision/CollisionRepresentation.h>
#include <SurgSim/Collision/ContactCalculation.h>
#include <SurgSim/Collision/CollisionPair.h>

#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Collision/RigidShapeCollisionRepresentation.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Collision
{


namespace
{
std::shared_ptr<RigidShape> sphereShape = std::make_shared<SphereShape>(1.0);
std::shared_ptr<RigidShape> doubleSidedPlaneShape = std::make_shared<DoubleSidedPlaneShape>();

std::shared_ptr<CollisionRepresentation> rep0 = std::make_shared<MockCollisionRepresentation>
	("TestSphere 1", sphereShape, Quaterniond::Identity(), Vector3d(1.0,0.0,0.0));
std::shared_ptr<CollisionRepresentation> rep1 = std::make_shared<MockCollisionRepresentation>
	("TestSphere 2", sphereShape, Quaterniond::Identity(), Vector3d(0.5,0.0,0.0));

std::shared_ptr<CollisionPair> pair01 = std::make_shared<CollisionPair>(rep0, rep1);
}





























}; // namespace Collision
}; // namespace SurgSim
