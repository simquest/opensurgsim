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

#include <fstream>
#include <gtest/gtest.h>
#include <SurgSim/Math/BoxShape.h>
#include <SurgSim/Math/CapsuleShape.h>
#include <SurgSim/Math/CylinderShape.h>
#include <SurgSim/Math/DoubleSidedPlaneShape.h>
#include <SurgSim/Math/PlaneShape.h>
#include <SurgSim/Math/SphereShape.h>
#include <SurgSim/Math/Shape.h>

#include <SurgSim/Serialize/Convert.h>

#include <SurgSim/Serialize/ShapesFactory.h>


class ShapesConvertTest : public ::testing::Test
{
protected:

	// Setup out stream file
	void SetUp()
	{
		datafile = "ShapesConvertTest.yaml";
		fout.open(datafile);
	}

	// Remove the config testing file
	void TearDown()
	{
		remove(datafile.c_str());
	}

	// Stream file
	std::ofstream fout;

	// Datafile name
	std::string datafile;
};

TEST_F(ShapesConvertTest, BoxConvertTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::BoxShape> shape =
		std::make_shared<SurgSim::Math::BoxShape>(1.0, 2.0, 3.0);

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::BoxShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_BOX, actualShape->getType());
	EXPECT_EQ(shape->getSizeX(), std::static_pointer_cast<SurgSim::Math::BoxShape>(actualShape)->getSizeX());
	EXPECT_EQ(shape->getSizeY(), std::static_pointer_cast<SurgSim::Math::BoxShape>(actualShape)->getSizeY());
	EXPECT_EQ(shape->getSizeZ(), std::static_pointer_cast<SurgSim::Math::BoxShape>(actualShape)->getSizeZ());
}

TEST_F(ShapesConvertTest, CapsuleConvertTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::CapsuleShape> shape =
		std::make_shared<SurgSim::Math::CapsuleShape>(1.0, 0.5);

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::CapsuleShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CAPSULE, actualShape->getType());
	EXPECT_EQ(shape->getLength(), std::static_pointer_cast<SurgSim::Math::CapsuleShape>(actualShape)->getLength());
	EXPECT_EQ(shape->getRadius(), std::static_pointer_cast<SurgSim::Math::CapsuleShape>(actualShape)->getRadius());
}

TEST_F(ShapesConvertTest, CylinderConvertTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::CylinderShape> shape =
		std::make_shared<SurgSim::Math::CylinderShape>(1.0, 0.5);

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::CylinderShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_CYLINDER, actualShape->getType());
	EXPECT_EQ(shape->getLength(), std::static_pointer_cast<SurgSim::Math::CylinderShape>(actualShape)->getLength());
	EXPECT_EQ(shape->getRadius(), std::static_pointer_cast<SurgSim::Math::CylinderShape>(actualShape)->getRadius());
}

TEST_F(ShapesConvertTest, SphereConvertTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::SphereShape> shape =
		std::make_shared<SurgSim::Math::SphereShape>(2.0);

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::SphereShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SPHERE, actualShape->getType());
	EXPECT_EQ(shape->getRadius(), std::static_pointer_cast<SurgSim::Math::SphereShape>(actualShape)->getRadius());
}

TEST_F(ShapesConvertTest, DoubleSidedPlaneShapeTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::DoubleSidedPlaneShape> shape =
		std::make_shared<SurgSim::Math::DoubleSidedPlaneShape>();

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::DoubleSidedPlaneShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_DOUBLESIDEDPLANE, actualShape->getType());
}

TEST_F(ShapesConvertTest, PlaneConvertTest)
{
	YAML::Node node;
	YAML::Emitter emitter(fout);

	std::shared_ptr<SurgSim::Math::PlaneShape> shape =
		std::make_shared<SurgSim::Math::PlaneShape>();

	/// Registering classes
	std::shared_ptr<SurgSim::Serialize::ShapesFactory> rigidShapesFactory=
		std::make_shared<SurgSim::Serialize::ShapesFactory>();

	rigidShapesFactory->registerShape<SurgSim::Math::PlaneShape>(shape->getClassName());


	/// Encoding box shape
	node = shape->encode();
	emitter << node;
	fout.close();

	/// Decoding box shape
	YAML::Node innode = YAML::LoadFile(datafile);
	std::string className = innode["ClassName"].as<std::string>();

	/// Instantiate derived objected from class name.
	std::shared_ptr<SurgSim::Math::Shape> actualShape =
		rigidShapesFactory->createShape(className);

	actualShape->decode(innode);

	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_PLANE, actualShape->getType());
}