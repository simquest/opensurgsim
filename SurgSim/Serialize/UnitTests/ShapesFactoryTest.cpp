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
#include "SurgSim/Math/BoxShape.h"
#include "SurgSim/Math/CapsuleShape.h"
#include "SurgSim/Math/CylinderShape.h"
#include "SurgSim/Math/DoubleSidedPlaneShape.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Shape.h"

#include "SurgSim/Serialize/ShapesFactory.h"


TEST(ShapesFactoryTest, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory =
		std::make_shared<SurgSim::Serialize::ShapesFactory>();});
}

TEST(ShapesFactoryTest, CreateShapeInstancesTest)
{
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory =
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	std::string boxName = "SurgSim::Physics::BoxShape";

	/// Register derived object classes
	rigidShapesFactory->registerShape<SurgSim::Math::BoxShape>(boxName);


	/// Instantiate derived objects from class name.
	std::shared_ptr<SurgSim::Math::Shape> boxShape1 = rigidShapesFactory->createShape(boxName);
	std::shared_ptr<SurgSim::Math::Shape> boxShape2 = rigidShapesFactory->createShape(boxName);

	// Expect all derived objects from the same class are not the same!
	EXPECT_NE(boxShape1, boxShape2);
}

TEST(ShapesFactoryTest, CreateShapesTest)
{
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory =
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	std::string boxName = "SurgSim::Physics::BoxShape";
	std::string capsuleName = "SurgSim::Physics::CapsuleShape";
	std::string cylinderName = "SurgSim::Physics::CylinderShape";
	std::string doublesideplaneName = "SurgSim::Physics::DoubleSidedPlaneShape";
	std::string planeName = "SurgSim::Physics::PlaneShape";
	std::string sphereName = "SurgSim::Physics::SphereShape";

	/// Register derived object classes
	rigidShapesFactory->registerShape<SurgSim::Math::BoxShape>(boxName);

	rigidShapesFactory->registerShape<SurgSim::Math::CapsuleShape>(capsuleName);
	rigidShapesFactory->registerShape<SurgSim::Math::CylinderShape>(cylinderName);
	rigidShapesFactory->registerShape<SurgSim::Math::DoubleSidedPlaneShape>(doublesideplaneName);
	rigidShapesFactory->registerShape<SurgSim::Math::PlaneShape>(planeName);
	rigidShapesFactory->registerShape<SurgSim::Math::SphereShape>(sphereName);

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> boxShape = rigidShapesFactory->createShape(boxName);
	std::shared_ptr<SurgSim::Math::Shape> capsuleShape = rigidShapesFactory->createShape(capsuleName);
	std::shared_ptr<SurgSim::Math::Shape> cylinderShape = rigidShapesFactory->createShape(cylinderName);
	std::shared_ptr<SurgSim::Math::Shape> doublesidedplaneShape =
		rigidShapesFactory->createShape(doublesideplaneName);
	std::shared_ptr<SurgSim::Math::Shape> planeShape = rigidShapesFactory->createShape(planeName);
	std::shared_ptr<SurgSim::Math::Shape> sphereShape = rigidShapesFactory->createShape(sphereName);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_BOX, boxShape->getType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CAPSULE, capsuleShape->getType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CYLINDER, cylinderShape->getType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_DOUBLESIDEDPLANE, doublesidedplaneShape->getType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_PLANE, planeShape->getType());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, sphereShape->getType());

}

TEST(ShapesFactoryTest, NonRegisterTest)
{
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory =
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	std::string boxName = "SurgSim::Physics::BoxShape";
	std::string capsuleName = "SurgSim::Physics::CapsuleShape";
	std::string cylinderName = "SurgSim::Physics::CylinderShape";
	std::string doublesideplaneName = "SurgSim::Physics::DoubleSidedPlaneShape";
	std::string planeName = "SurgSim::Physics::PlaneShape";
	std::string sphereName = "SurgSim::Physics::SphereShape";

	/// Not register any derived object classes


	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> boxShape = rigidShapesFactory->createShape(boxName);
	std::shared_ptr<SurgSim::Math::Shape> capsuleShape = rigidShapesFactory->createShape(capsuleName);
	std::shared_ptr<SurgSim::Math::Shape> cylinderShape = rigidShapesFactory->createShape(cylinderName);
	std::shared_ptr<SurgSim::Math::Shape> doublesidedplaneShape =
		rigidShapesFactory->createShape(doublesideplaneName);
	std::shared_ptr<SurgSim::Math::Shape> planeShape = rigidShapesFactory->createShape(planeName);
	std::shared_ptr<SurgSim::Math::Shape> sphereShape = rigidShapesFactory->createShape(sphereName);

	/// Expect a null pointer form the Shape objects
	EXPECT_EQ(nullptr, boxShape);
	EXPECT_EQ(nullptr, capsuleShape);
	EXPECT_EQ(nullptr, cylinderShape);
	EXPECT_EQ(nullptr, doublesidedplaneShape);
	EXPECT_EQ(nullptr, planeShape);
	EXPECT_EQ(nullptr, sphereShape);

}
