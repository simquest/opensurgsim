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

TEST_F(ConverterTest, ConverterQuaterniondNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode.push_back(quat);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond expectedQuat = innode[0].as<SurgSim::Math::Quaterniond>();

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
	YAML::Emitter outnode(fout);

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	outnode << rigid;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d expectedRigid = innode.as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(expectedRigid.matrix(), rigid.matrix());
}
