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

#include <SurgSim/Framework/ApplicationData.h>
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

	struct TruthCube
	{
		// Data positions of uncompressed data
		std::vector<Vector3d> cubeData0;

		// Data positions of 5% strain 
		std::vector<Vector3d> cubeData1;

		// Data positions of  12.5% strain
		std::vector<Vector3d> cubeData2;

		// Data positions of  18.25% strain
		std::vector<Vector3d> cubeData3;
	};

	TruthCube truthCube;

	/// Parsing Truth Cube data from an external file
	/// The uniaxial compression experimental dataset is obtained from:
	/// "Establishing Physical Standards for Real Time Soft Tissue Simulation" project.
	///  URL: http://biorobotics.harvard.edu/truthcube/truthcube.html 
	/// \param filename The filename
	/// \param cubeData0 The positions of uncompressed cube
	/// \param cubeData1 The positions of 5% strain cube
	/// \param cubeData2 The positions of 12.5% strain cube
	/// \param cubeData3 The positions of 18.25% strain cube

	bool ParseTruthCubeData(TruthCube& truthCube)
	{
		// Position of uncompressed data
		Vector3d position0;

		// Position of 5% strain 
		Vector3d position1;

		// Position of  12.5% strain
		Vector3d position2;

		// Position of  18.25% strain
		Vector3d position3;

		std::string line;

		char comma;
		int i,j,k;

		// The number of lines for Truth cube header files
		const int commentIndex = 7;

		const SurgSim::Framework::ApplicationData data("config.txt");
		std::string filename = data.findFile("uniaxial_positions.csv");

		std::ifstream datafile(filename);

		if (! datafile.good())
		{
			return false;
		}

		int index = 0;

		while (std::getline(datafile, line))
			if (++index > commentIndex)
			{
				std::stringstream strstream(line);
				strstream >> i >> comma >> j >> comma >> k >> comma 
				>> position0.x() >> comma >> position0.y() >> comma >> position0.z() >> comma
				>> position1.x() >> comma >> position1.y() >> comma >> position1.z() >> comma
				>> position2.x() >> comma >> position2.y() >> comma >> position2.z() >> comma
				>> position3.x() >> comma >> position3.y() >> comma >> position3.z();

				// Store proper strains for each cubeData
				truthCube.cubeData0.push_back(position0);
				truthCube.cubeData1.push_back(position1);
				truthCube.cubeData2.push_back(position2);
				truthCube.cubeData3.push_back(position3);
			}

		return true;

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

			// Parse Truth cube data
			ParseTruthCubeData(truthCube);
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
		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
			("TruthCube uncompressed");
		auto pointCloud = representation->getVertices();

		/// Loading the Truth Cube data
		for (size_t i = 0; i < truthCube.cubeData0.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(truthCube.cubeData0[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0),
			SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

	TEST_F(RenderTests, LoadTruthCubeData_Compressed_5)
	{
		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
			("TruthCube 5% strain");
		auto pointCloud = representation->getVertices();

		/// Load Truth Cube data
		for (size_t i = 0; i < truthCube.cubeData1.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(truthCube.cubeData1[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0),
			SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

	TEST_F(RenderTests, LoadTruthCubeData_Compressed_12_5)
	{
		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
			("TruthCube 12.5 strain");
		auto pointCloud = representation->getVertices();

		/// Load Truth Cube data
		for (size_t i = 0; i < truthCube.cubeData2.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(truthCube.cubeData2[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0),
			SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

	TEST_F(RenderTests, LoadTruthCubeData_Compressed_18_25)
	{
		auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
			("TruthCube 18.25%");
		auto pointCloud = representation->getVertices();

		/// Load Truth Cube data
		for (size_t i = 0; i < truthCube.cubeData3.size(); ++i)
		{
			pointCloud->addVertex(CloudMesh::VertexType(truthCube.cubeData3[i]));
		}

		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(SurgSim::Math::Quaterniond::Identity(), Vector3d(0.0,0.0,0.0));
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(SurgSim::Math::Vector3d(0.0, 140.0, 0.0),
			SurgSim::Math::Vector3d(0.0, 0.0, 0.0));

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));		
	}

}; // namespace Physics
}; // namespace SurgSim
