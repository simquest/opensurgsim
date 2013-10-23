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

#include <SurgSim/DataStructures/Vertices.h>
#include <SurgSim/Graphics/RenderTests/RenderTest.h>
#include <SurgSim/Graphics/VectorFieldRepresentation.h>
#include <SurgSim/Graphics/OsgVectorFieldRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Testing/MathUtilities.h>

namespace SurgSim
{
namespace Graphics
{

using SurgSim::DataStructures::Vertices;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

struct OsgVectorFieldRepresentationRenderTests : public SurgSim::Graphics::RenderTest
{
protected:
	std::vector<Vector3d> makeVertices()
	{
		std::vector<Vector3d> result;
		result.emplace_back(Vector3d(0.00, 0.01, 0.00));
		result.emplace_back(Vector3d(-0.01, 0.00, 0.00));
		
		result.emplace_back(Vector3d(0.00, 0.00, -0.01));
		result.emplace_back(Vector3d(0.00, -0.01, 0.00));
		return result;
	}

	std::shared_ptr<VectorFieldRepresentation<void>> makeCloud(const std::vector<Vector3d>& vertices)
	{
		auto vectorRepresentation =	
			std::make_shared<OsgVectorFieldRepresentation<void>>("vectorRepresentation representation");

		auto mesh = std::make_shared<Vertices<void>>();
		for (auto it = std::begin(vertices); it != std::end(vertices); ++it)
		{
			mesh->addVertex(Vertices<void>::VertexType(*it));
		}

		vectorRepresentation->setVertices(mesh);
		vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0,0.0,-0.05)));

		viewElement->addComponent(vectorRepresentation);

		return vectorRepresentation;
	}
};

TEST_F(OsgVectorFieldRepresentationRenderTests, LineWidth)
{
	std::shared_ptr<VectorFieldRepresentation<void>> vectorRepresentation = makeCloud(makeVertices());

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 100;
	double startWidth = 0;
	double endWidth = 10.0;

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		vectorRepresentation->setLineWidth(interpolate(startWidth, endWidth, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

TEST_F(OsgVectorFieldRepresentationRenderTests, ColorTest)
{
	std::shared_ptr<VectorFieldRepresentation<void>> vectorRepresentation = makeCloud(makeVertices());
	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 10;

	SurgSim::Math::Vector4d startColor(0.0, 0.0, 0.0, 0.0);
	SurgSim::Math::Vector4d endColor(1.0, 1.0, 0.0, 0.0);

	SurgSim::Math::Vector4d startColor2(0.0, 0.0, 1.0, 0.0);
	SurgSim::Math::Vector4d endColor2(0.0, 1.0, 0.0, 0.0);

	SurgSim::Math::Vector4d startColor3(0.0, 0.0, 1.0, 0.0);
	SurgSim::Math::Vector4d endColor3(0.0, 0.0, 1.0, 1.0);

	SurgSim::Math::Vector4d startColor4(0.0, 0.0, 0.0, 1.0);
	SurgSim::Math::Vector4d endColor4(1.0, 1.0, 1.0, 0.0);
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;

		std::vector<Vector4d> vColor;
		vColor.emplace_back(interpolate(startColor,endColor,t));
		vColor.emplace_back(interpolate(startColor2,endColor2,t));
		vColor.emplace_back(interpolate(startColor3,endColor3,t));
		vColor.emplace_back(interpolate(startColor4,endColor4,t));
		vectorRepresentation->setColors(vColor);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}

}; // namespace Graphics
}; // namespace SurgSim
