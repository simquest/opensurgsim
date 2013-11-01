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
#include <SurgSim/Serialize/Convert.h>
#include <SurgSim/Graphics/SphereRepresentation.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>

class GraphicsConvertTest : public ::testing::Test
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


TEST_F(GraphicsConvertTest, ConvertSphereRepresentationTest)
{
	YAML::Node node;
	YAML::Emitter outnode(fout);

	std::shared_ptr<SurgSim::Graphics::SphereRepresentation> sphereRepresentation =
		std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("Sphere_Obj");

	double sphereRadius = 5.0;
	sphereRepresentation->setRadius(sphereRadius);

	SurgSim::Math::RigidTransform3d spherePose = SurgSim::Math::makeRigidTransform(
		SurgSim::Math::Quaterniond(SurgSim::Math::Vector4d::Identity()).normalized(),
		SurgSim::Math::Vector3d::Identity());
	sphereRepresentation->setPose(spherePose);

	/// Encoding sphere representation
	node = YAML::convert<SurgSim::Graphics::SphereRepresentation>::encode(*sphereRepresentation);
	outnode << node;
	fout.close();

	/// Decoding sphere representation
	YAML::Node innode = YAML::LoadFile(datafile);
	std::shared_ptr<SurgSim::Graphics::SphereRepresentation> actualSphere =
		std::make_shared<SurgSim::Graphics::OsgSphereRepresentation>("ImageSphere");
	YAML::convert<SurgSim::Graphics::SphereRepresentation>::decode(innode, actualSphere);

	EXPECT_EQ(sphereRepresentation->getRadius(), actualSphere->getRadius());
	EXPECT_TRUE(actualSphere->getInitialPose().matrix().isApprox(sphereRepresentation->getInitialPose().matrix()));
	EXPECT_TRUE(actualSphere->getPose().matrix().isApprox(sphereRepresentation->getPose().matrix()));
}