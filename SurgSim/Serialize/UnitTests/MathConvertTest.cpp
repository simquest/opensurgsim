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
#include <yaml-cpp/yaml.h>
#include <SurgSim/Serialize/MathConvert.h>
#include <SurgSim/Math/Valid.h>
#include <limits>

using SurgSim::Math::isValid;

class MathConvertTest : public ::testing::Test
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

TEST_F(MathConvertTest, ConvertVector3dInvalidTest)
{
	YAML::Node outnode;
	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);

	// Add NaN value
	vector3(1)= std::numeric_limits<SurgSim::Math::Vector3d::Scalar>::quiet_NaN();

	outnode = vector3;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d actualv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_TRUE(! isValid(actualv3d));
}

TEST_F(MathConvertTest, ConvertVector3dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);
	outnode = vector3;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d actualv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_EQ(vector3, actualv3d);
}


TEST_F(MathConvertTest, ConvertVector3dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);
	outnode << vector3;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector3d actualv3d = innode.as<SurgSim::Math::Vector3d>();
	EXPECT_EQ(vector3, actualv3d);
}

TEST_F(MathConvertTest, ConvertVector4dInvalidTest)
{
	YAML::Node outnode;
	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);

	// Add NaN value
	vector4(1)= std::numeric_limits<SurgSim::Math::Vector4d::Scalar>::quiet_NaN();

	outnode = vector4;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d actualv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_TRUE(! isValid(actualv4d));
}


TEST_F(MathConvertTest, ConvertVector4dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);
	outnode = vector4;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d actualv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_EQ(vector4, actualv4d);
}

TEST_F(MathConvertTest, ConvertVector4dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);
	outnode <<  vector4;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Vector4d actualv4d = innode.as<SurgSim::Math::Vector4d>();
	EXPECT_EQ(vector4, actualv4d);
}
TEST_F(MathConvertTest, ConvertQuaterniondInvalidTest)
{
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	quat.x() = std::numeric_limits<SurgSim::Math::Quaterniond::Scalar>::quiet_NaN();

	outnode = quat;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond actualQuat = innode.as<SurgSim::Math::Quaterniond>();
	EXPECT_TRUE(! isValid(actualQuat));
}

TEST_F(MathConvertTest, ConvertQuaterniondNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode = quat;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond actualQuat = innode.as<SurgSim::Math::Quaterniond>();

	EXPECT_EQ(quat.x(), actualQuat.x());
	EXPECT_EQ(quat.y(), actualQuat.y());
	EXPECT_EQ(quat.z(), actualQuat.z());
	EXPECT_EQ(quat.w(), actualQuat.w());
}

TEST_F(MathConvertTest, ConvertQuaterniondEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode << quat;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Quaterniond actualQuat = innode.as<SurgSim::Math::Quaterniond>();

	EXPECT_EQ(quat.x(), actualQuat.x());
	EXPECT_EQ(quat.y(), actualQuat.y());
	EXPECT_EQ(quat.z(), actualQuat.z());
	EXPECT_EQ(quat.w(), actualQuat.w());
}

TEST_F(MathConvertTest, ConvertMatrix33dInvalidTest)
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
	SurgSim::Math::Matrix33d actualm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_TRUE(! isValid(actualm33d));

}

TEST_F(MathConvertTest, ConvertMatrix33dNodeTest)
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
	SurgSim::Math::Matrix33d actualm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_EQ(mat33d, actualm33d);
}

TEST_F(MathConvertTest, ConvertMatrix33dEmitterTest)
{
	YAML::Emitter outnode(fout);

	SurgSim::Math::Matrix33d mat33d;
	mat33d <<	1, 2, 3,
		3, 2, 1,
		1, 2, 3;

	outnode << mat33d;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::Matrix33d actualm33d = innode.as<SurgSim::Math::Matrix33d>();

	EXPECT_EQ(mat33d, actualm33d);
}

TEST_F(MathConvertTest, ConvertMatrix44dInvalidTest)
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
	SurgSim::Math::Matrix44d actualm44d = innode.as<SurgSim::Math::Matrix44d>();
	EXPECT_TRUE(! isValid(actualm44d));
}

TEST_F(MathConvertTest, ConvertMatrix44dNodeTest)
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
	SurgSim::Math::Matrix44d actualm44d = innode.as<SurgSim::Math::Matrix44d>();

	EXPECT_EQ(mat44d, actualm44d);
}

TEST_F(MathConvertTest, ConvertMatrix44dEmitterTest)
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
	SurgSim::Math::Matrix44d actualm44d = innode.as<SurgSim::Math::Matrix44d>();

	EXPECT_EQ(mat44d, actualm44d);
}

TEST_F(MathConvertTest, ConvertRigidTransform3dInvalidTest)
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
	SurgSim::Math::RigidTransform3d actualRigid = innode.as<SurgSim::Math::RigidTransform3d>();
	EXPECT_TRUE(! isValid(actualRigid));
}

TEST_F(MathConvertTest, ConvertRigidTransform3dNodeTest)
{
	YAML::Node outnode;

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	outnode = rigid;
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d actualRigid = innode.as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(rigid.matrix(), actualRigid.matrix());
}

TEST_F(MathConvertTest, ConvertRigidTransform3dEmitterTest)
{
	YAML::Node node;
	YAML::Emitter outnode(fout);

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	node = rigid;
	outnode << node;
	fout.close();

	YAML::Node innode = YAML::LoadFile(datafile);
	SurgSim::Math::RigidTransform3d actualRigid = innode.as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(rigid.matrix(), actualRigid.matrix());
}
