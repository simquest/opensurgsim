// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include <memory>

#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Particles/RandomPointGenerator.h"

using SurgSim::Math::BoxShape;
using SurgSim::Math::MeshShape;
using SurgSim::Math::Shape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Particles::RandomPointGenerator;

TEST(RandomPointGeneratorTest, ConstructorTest)
{
	ASSERT_NO_THROW(RandomPointGenerator());
}

TEST(RandomPointGeneratorTest, GenerationTest)
{
	auto pointGenerator = std::make_shared<RandomPointGenerator>();

	auto boxShape = std::make_shared<BoxShape>(1.0, 2.0, 3.0);
	EXPECT_NO_THROW(pointGenerator->pointInShape(boxShape));

	auto sphereShape = std::make_shared<SphereShape>(6.0);
	EXPECT_NO_THROW(pointGenerator->pointOnShape(sphereShape));

	auto meshShape = std::make_shared<MeshShape>();
	EXPECT_NO_THROW(pointGenerator->pointOnShape(meshShape));

	std::shared_ptr<Shape> shape;
	EXPECT_THROW(pointGenerator->pointOnShape(shape), SurgSim::Framework::AssertionFailure);
}
