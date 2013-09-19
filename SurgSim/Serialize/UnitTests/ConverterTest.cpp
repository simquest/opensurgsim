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

TEST(SerializeTest, ConverterVector3dTest)
{
	std::ofstream fout("config_vector3d.yaml");
	YAML::Node outnode;

	SurgSim::Math::Vector3d vector3(1.000001, 2.000001, 3.000001);
	outnode.push_back(vector3);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile("config_vector3d.yaml");
	SurgSim::Math::Vector3d expectedv3d = innode[0].as<SurgSim::Math::Vector3d>();
	EXPECT_EQ(expectedv3d, vector3);
}

TEST(SerializeTest, ConverterVector4dTest)
{
	std::ofstream fout("config_vector4d.yaml");
	YAML::Node outnode;

	SurgSim::Math::Vector4d vector4(1.000001, 2.000001, 3.000001, 4.000001);
	outnode.push_back(vector4);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile("config_vector4d.yaml");
	SurgSim::Math::Vector4d expectedv4d = innode[0].as<SurgSim::Math::Vector4d>();
	EXPECT_EQ(expectedv4d, vector4);
}


TEST(SerializeTest, ConverterQuaterniondTest)
{
	std::ofstream fout("config_quaterniond.yaml");
	YAML::Node outnode;

	SurgSim::Math::Quaterniond quat(3, 2, 1, 1);
	outnode.push_back(quat);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile("config_quaterniond.yaml");
	SurgSim::Math::Quaterniond expectedQuat = innode[0].as<SurgSim::Math::Quaterniond>();

	EXPECT_EQ(expectedQuat.x(), quat.x());
	EXPECT_EQ(expectedQuat.y(), quat.y());
	EXPECT_EQ(expectedQuat.z(), quat.z());
	EXPECT_EQ(expectedQuat.w(), quat.w());
}


TEST(SerializeTest, ConverterMatrix44dTest)
{
	std::ofstream fout("config_matrix44d.yaml");
	YAML::Node outnode;

	SurgSim::Math::Matrix44d mat44d;
	mat44d <<	1, 2, 3, 4,
				4, 3, 2, 1,
				1, 2, 3, 4,
				4, 3, 2, 1;

	outnode.push_back(mat44d);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile("config_matrix44d.yaml");
	SurgSim::Math::Matrix44d expectedm44d = innode[0].as<SurgSim::Math::Matrix44d>();

	EXPECT_EQ(expectedm44d, mat44d);
}

TEST(SerializeTest, ConverterRigidTransform3dTest)
{
	std::ofstream fout("config_rigidtransform3d.yaml");
	YAML::Node outnode;

	SurgSim::Math::RigidTransform3d rigid = SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
		SurgSim::Math::Vector3d(1.0, 2.0, 3.0));

	outnode.push_back(rigid);
	fout << outnode;
	fout.close();

	YAML::Node innode = YAML::LoadFile("config_rigidtransform3d.yaml");
	SurgSim::Math::RigidTransform3d expectedRigid = innode[0].as<SurgSim::Math::RigidTransform3d>();

	EXPECT_EQ(expectedRigid.matrix(), rigid.matrix());
}
