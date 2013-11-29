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

#include <memory>
#include <vector>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/PointCloudRepresentation.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgPointCloudRepresentation.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/DataStructures/Vertices.h>

#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Testing/MathUtilities.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

namespace SurgSim
{
namespace Physics
{

	typedef SurgSim::DataStructures::Vertices<void> CloudMesh;

	std::vector<Vector3d> ReadTruthCubeData(std::string filename)
	{
		std::ifstream datafile(filename);
		std::vector<Vector3d> cubeData;
		std::string line;
		Vector3d position;
		char comma;

		// Parsing header line
		std::getline(datafile, line); 

		while (std::getline(datafile, line))
		{
			std::stringstream strstream(line);
			strstream >> position.x() >> comma >> position.y() >> comma >> position.z();
			cubeData.push_back(position);
		}

		return cubeData;

	};

	struct RenderTests : public ::testing::Test
	{
		virtual void SetUp()
		{
			runtime = std::make_shared<SurgSim::Framework::Runtime>();
			graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();
			
			runtime->addManager(graphicsManager);

			scene = std::make_shared<SurgSim::Framework::Scene>();
			
			runtime->setScene(scene);
	
			viewElement = std::make_shared<SurgSim::Graphics::OsgViewElement>("Truth Cube");
			scene->addSceneElement(viewElement);

		}

		virtual void TearDown()
		{
			runtime->stop();
		}

		std::shared_ptr<SurgSim::Framework::Runtime> runtime;
		std::shared_ptr<SurgSim::Graphics::OsgManager> graphicsManager;
		std::shared_ptr<SurgSim::Framework::Scene> scene;
		std::shared_ptr<SurgSim::Graphics::OsgViewElement> viewElement;

	};

	TEST_F(RenderTests, LoadTruthCubeData_Uncompressed)
	{
		std::vector<Vector3d> vertices = ReadTruthCubeData("Data/uncompressed.csv");
		
		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>("TruthCube uncompressred");
		auto pointCloud = representation->getVertices();
		
		/// Add vertices 
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(vertices[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0), SurgSim::Math::Vector3d(0.0, 0.0, 0.0));
		
		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

	TEST_F(RenderTests, LoadTruthCubeData_Compressed_5)
	{
		std::vector<Vector3d> vertices = ReadTruthCubeData("Data/compressed_5.csv");

		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>("TruthCube 5% strain");
		auto pointCloud = representation->getVertices();

		/// Add vertices 
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(vertices[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0), SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

	TEST_F(RenderTests, LoadTruthCubeData_Compressed_12_5)
	{
		std::vector<Vector3d> vertices = ReadTruthCubeData("Data/compressed_12_5.csv");

		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>("TruthCube 12.5 strain");
		auto pointCloud = representation->getVertices();

		/// Add vertices 
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(vertices[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0), SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}
			
	TEST_F(RenderTests, LoadTruthCubeData_Compressed_18_25)
	{
		std::vector<Vector3d> vertices = ReadTruthCubeData("Data/compressed_18_25.csv");

		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>("TruthCube 18.25%");
		auto pointCloud = representation->getVertices();

		/// Add vertices 
		for (size_t i = 0; i < vertices.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(vertices[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0), SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

}; // namespace Physics
}; // namespace SurgSim
