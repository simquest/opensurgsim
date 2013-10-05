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
#include <SurgSim/Serialize/Converter.h>
#include <SurgSim/Math/Valid.h>
#include <limits>
#include <SurgSim/Graphics/SphereRepresentation.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>


#include <random>

using SurgSim::Math::isValid;

class ConverterTest : public ::testing::Test
{
protected:

	// Setup out stream file
	void SetUp()
	{
		datafile = "convertertest.yaml";
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

TEST_F(ConverterTest, ConverterVector3dInvalidTest)
{
	YAML::Node outnode;
	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);

	// Add NaN value
	vector3(1)= std::numeric_limits<SurgSim::Math::Vector3d::Scalar>::quiet_NaN();

	outnode = vector3;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d expectedv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_TRUE(! isValid(expectedv3d));
}

TEST_F(ConverterTest, ConverterVector3dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);
	outnode = vector3;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d expectedv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_EQ(expectedv3d, vector3);
}


TEST_F(ConverterTest, ConverterVector3dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);
	outnode << vector3;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d expectedv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_EQ(expectedv3d, vector3);
}

TEST_F(ConverterTest, ConverterVector4dInvalidTest)
{
	YAML::Node outnode;
	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);

	// Add NaN value
	vector4(1)= std::numeric_limits<SurgSim::Math::Vector4d::Scalar>::quiet_NaN();

	outnode = vector4;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d expectedv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_TRUE(! isValid(expectedv4d));
}


TEST_F(ConverterTest, ConverterVector4dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);
	outnode = vector4;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d expectedv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_EQ(expectedv4d, vector4);
}

TEST_F(ConverterTest, ConverterVector4dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);
	outnode <<  vector4;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d expectedv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_EQ(expectedv4d, vector4);
}
TEST_F(ConverterTest, ConverterQuaterniondInvalidTest)
{
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	quat.x() = std::numeric_limits<SurgSim::Math::Quaterniond::Scalar>::quiet_NaN();

	outnode = quat;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond expectedQuat = innode.as<SurgSim::Math::Quaterniond>();
	EXPECT_TRUE(! isValid(expectedQuat));
}

TEST_F(ConverterTest, ConverterQuaterniondNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode = quat;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond expectedQuat = innode.as<SurgSim::Math::Quaterniond>();

	EXPECT_EQ(expectedQuat.x(), quat.x());
	EXPECT_EQ(expectedQuat.y(), quat.y());
	EXPECT_EQ(expectedQuat.z(), quat.z());
	EXPECT_EQ(expectedQuat.w(), quat.w());
}

TEST_F(ConverterTest, ConverterQuaterniondEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode << quat;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond expectedQuat = innode.as<SurgSim::Math::Quaterniond>();

	EXPECT_EQ(expectedQuat.x(), quat.x());
	EXPECT_EQ(expectedQuat.y(), quat.y());
	EXPECT_EQ(expectedQuat.z(), quat.z());
	EXPECT_EQ(expectedQuat.w(), quat.w());
}

TEST_F(ConverterTest, ConverterMatrix33dInvalidTest)
{
	YAML::Node outnode;

	SurgSim::Math::Matrix33d mat33d;
	mat33d <<	1, 2, 3,
		3, 2, 1,
		1, 2, 3;

	mat33d(1,1) = std::numeric_limits<SurgSim::Math::Matrix33d::Scalar>::quiet_NaN();

	outnode = mat33d;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix33d expectedm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_TRUE(! isValid(expectedm33d));

}

TEST_F(ConverterTest, ConverterMatrix33dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Matrix33d mat33d;
	mat33d <<	1, 2, 3,
		3, 2, 1,
		1, 2, 3;

	outnode = mat33d;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix33d expectedm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_EQ(expectedm33d, mat33d);
}

TEST_F(ConverterTest, ConverterMatrix33dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Matrix33d mat33d;
	mat33d <<	1, 2, 3,
		3, 2, 1,
		1, 2, 3;

	outnode << mat33d;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix33d expectedm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_EQ(expectedm33d, mat33d);
}

TEST_F(ConverterTest, ConverterMatrix44dInvalidTest)
{
	YAML::Node outnode;

	SurgSim::Math::Matrix44d mat44d;
	mat44d <<	1, 2, 3, 4,
		4, 3, 2, 1,
		1, 2, 3, 4,
		4, 3, 2, 1;

	mat44d(1,1) = std::numeric_limits<SurgSim::Math::Matrix44d::Scalar>::quiet_NaN();

	outnode = mat44d;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix44d expectedm44d = innode.as<SurgSim::Math::Matrix44d>();
	EXPECT_TRUE(! isValid(expectedm44d));
}

TEST_F(ConverterTest, ConverterMatrix44dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Matrix44d mat44d;
	mat44d <<	1, 2, 3, 4,
				4, 3, 2, 1,
				1, 2, 3, 4,
				4, 3, 2, 1;

	outnode = mat44d;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix44d expectedm44d = innode.as<SurgSim::Math::Matrix44d>();

	EXPECT_EQ(expectedm44d, mat44d);
}

TEST_F(ConverterTest, ConverterMatrix44dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Matrix44d mat44d;
	mat44d <<	1, 2, 3, 4,
		4, 3, 2, 1,
		1, 2, 3, 4,
		4, 3, 2, 1;

	outnode << mat44d;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix44d expectedm44d = innode.as<SurgSim::Math::Matrix44d>();

	EXPECT_EQ(expectedm44d, mat44d);
}

TEST_F(ConverterTest, ConverterRigidTransform3dInvalidTest)
{
	YAML::Node outnode;
	SurgSim::Math::Vector3d vec3(1.0, 2.0, 3.0);
	vec3[0] = std::numeric_limits<SurgSim::Math::RigidTransform3d::Scalar>::quiet_NaN();
	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		vec3);

	outnode = rigid;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d expectedRigid = innode.as<SurgSim::Math::RigidTransform3d>();
	EXPECT_TRUE(! isValid(expectedRigid));
}

TEST_F(ConverterTest, ConverterRigidTransform3dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	outnode = rigid;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d expectedRigid = innode.as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(expectedRigid.matrix(), rigid.matrix());
}

TEST_F(ConverterTest, ConverterRigidTransform3dEmitterTest)
{
	YAML::Node node;
	YAML::Emitter outnode(fout);

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	node = rigid;
	outnode << node;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d expectedRigid = innode.as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(expectedRigid.matrix(), rigid.matrix());
}

TEST_F(ConverterTest, ConvertSphereRepresentationTest)
{
	YAML::Node node;
	YAML::Emitter outnode(fout);

	std::shared_ptr<SurgSim::Graphics::SphereRepresentation> sphereRepresentation = 
		std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("Sphere_Obj");

	double sphereRadius = 5.0;
	sphereRepresentation->setRadius(sphereRadius);

	SurgSim::Math::RigidTransform3d spherePose = SurgSim::Math::makeRigidTransform(
		SurgSim::Math::Quaterniond(SurgSim::Math::Vector4d::Random()).normalized(),
		SurgSim::Math::Vector3d::Random());
	sphereRepresentation->setPose(spherePose);

	/// Encoding sphere representation
	node = YAML::convert<SurgSim::Graphics::SphereRepresentation>::encode(sphereRepresentation);
	outnode << node;
	fout.close();
	
	/// Decoding sphere representation
	YAML::Node innode = YAML::LoadFile(datafile);
	std::shared_ptr<SurgSim::Graphics::SphereRepresentation> expectedSphere =  std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("ImageSphere");
	YAML::convert<SurgSim::Graphics::SphereRepresentation>::decode(innode, expectedSphere);
	
	EXPECT_EQ(sphereRepresentation->getRadius(), expectedSphere->getRadius());
	EXPECT_TRUE(expectedSphere->getInitialPose().matrix().isApprox(sphereRepresentation->getInitialPose().matrix()));
	EXPECT_TRUE(expectedSphere->getPose().matrix().isApprox(sphereRepresentation->getPose().matrix()));
}