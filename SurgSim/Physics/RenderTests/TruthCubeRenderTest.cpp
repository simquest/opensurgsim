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

/// This test is based on Truth Cube experimental dataset that is obtained from:
/// "Establishing Physical Standards for Real Time Soft Tissue Simulation" project.
///  URL: http://biorobotics.harvard.edu/truthcube/truthcube.html

#include "SurgSim/Physics/RenderTests/RenderTest.h"

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;


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

struct TruthCubeRenderTests : public RenderTests
{
	/// Parsing Truth Cube data from an external file
	/// \param truthCube a container of cube data for all strains
	/// \return True if the Truth Cube Data is successful loaded, otherwise false
	bool ParseTruthCubeData(std::shared_ptr<TruthCube> truthCube)
	{
		// Position of uncompressed data, 5% strain, 12.5% strain, 18.25% strain
		Vector3d position0, position1, position2, position3;

		const int numCommentLine = 7;
		std::string lineId;
		char comma;
		int i,j,k, index = 0;

		const SurgSim::Framework::ApplicationData data("config.txt");

		std::string filename = data.findFile("uniaxial_positions.csv");

		std::ifstream datafile(filename);

		if (! datafile.good())
		{
			return false;
		}

		while (std::getline(datafile, lineId))
			if (++index > numCommentLine)
			{
				std::stringstream strstream(lineId);
				strstream >> i >> comma >> j >> comma >> k >> comma
					>> position0.x() >> comma >> position0.y() >> comma >> position0.z() >> comma
					>> position1.x() >> comma >> position1.y() >> comma >> position1.z() >> comma
					>> position2.x() >> comma >> position2.y() >> comma >> position2.z() >> comma
					>> position3.x() >> comma >> position3.y() >> comma >> position3.z();

				// Store proper strains for each cubeData
				truthCube->cubeData0.push_back(position0);
				truthCube->cubeData1.push_back(position1);
				truthCube->cubeData2.push_back(position2);
				truthCube->cubeData3.push_back(position3);
			}

			return true;

	};

	void doInitialize()
	{
		// Initialize truthCube variable
		truthCube = std::make_shared<TruthCube>();

		// Parsing TruthCube data
		ParseTruthCubeData(truthCube);
	};

	void runTest(std::shared_ptr<SurgSim::Graphics::OsgPointCloudRepresentation<void>> representation)
	{
		representation->setPointSize(4.0);
		RigidTransform3d pose = makeRigidTransform(Quaterniond::Identity(), Vector3d::Zero());
		representation->setInitialPose(pose);

		viewElement->addComponent(representation);
		viewElement->enableManipulator(true);
		viewElement->setManipulatorParameters(Vector3d(0.0, 140.0, 0.0), Vector3d::Zero());

		/// Run the thread
		runtime->start();

		boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
	}

	std::shared_ptr<TruthCube> truthCube;
};

TEST_F(TruthCubeRenderTests, LoadTruthCubeData_Uncompressed)
{
	doInitialize();

	auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("TruthCube uncompressed");
	auto pointCloud = representation->getVertices();

	/// Loading the Truth Cube uncompressed data into point cloud
	for (size_t i = 0; i < truthCube->cubeData0.size(); ++i)
	{
		pointCloud->addVertex(CloudMesh::VertexType(truthCube->cubeData0[i]));
	}

	runTest(representation);
}

TEST_F(TruthCubeRenderTests, LoadTruthCubeData_Compressed_5)
{
	doInitialize();

	auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("TruthCube 5% strain");
	auto pointCloud = representation->getVertices();

	/// Loading the Truth Cube data into point cloud
	for (size_t i = 0; i < truthCube->cubeData1.size(); ++i)
	{
		pointCloud->addVertex(CloudMesh::VertexType(truthCube->cubeData1[i]));
	}

	runTest(representation);
}

TEST_F(TruthCubeRenderTests, LoadTruthCubeData_Compressed_12_5)
{
	doInitialize();

	auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("TruthCube 12.5 strain");
	auto pointCloud = representation->getVertices();

	/// Loading the Truth Cube data into point cloud
	for (size_t i = 0; i < truthCube->cubeData2.size(); ++i)
	{
		pointCloud->addVertex(CloudMesh::VertexType(truthCube->cubeData2[i]));
	}

	runTest(representation);
}

TEST_F(TruthCubeRenderTests, LoadTruthCubeData_Compressed_18_25)
{
	doInitialize();

	auto representation = std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation<void>>
		("TruthCube 18.25%");
	auto pointCloud = representation->getVertices();

	/// Loading the Truth Cube data into point cloud
	for (size_t i = 0; i < truthCube->cubeData3.size(); ++i)
	{
		pointCloud->addVertex(CloudMesh::VertexType(truthCube->cubeData3[i]));
	}

	runTest(representation);
}

}; // namespace Physics
}; // namespace SurgSim
