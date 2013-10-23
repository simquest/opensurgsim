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
		result.emplace_back(Vector3d(0.0, 0.005,  0.01));
		result.emplace_back(Vector3d(0.0, 0.01, 0.01));

		result.emplace_back(Vector3d(0.0, 0.015, 0.01));
		result.emplace_back(Vector3d(0.01,0.02, 0.01));

		result.emplace_back(Vector3d(0.015,0.02, 0.01));
		result.emplace_back(Vector3d(0.02, 0.02, 0.01));

		result.emplace_back(Vector3d(0.025, 0.02, 0.01));
		result.emplace_back(Vector3d(0.03, 0.03, 0.01));
		return result;
	}

	std::shared_ptr<VectorFieldRepresentation<void>> makeCloud(const std::vector<Vector3d>& vertices)
	{
		auto vectorRepresentation =	std::make_shared<OsgVectorFieldRepresentation<void>>("vectorRepresentation representation");

		auto mesh = std::make_shared<Vertices<void>>();
		for (auto it = std::begin(vertices); it != std::end(vertices); ++it)
		{
			mesh->addVertex(Vertices<void>::VertexType(*it));
		}

		vectorRepresentation->setVertices(mesh);
		vectorRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0,0.0,-0.2)));

		viewElement->addComponent(vectorRepresentation);

		return vectorRepresentation;
	}
};

TEST_F(OsgVectorFieldRepresentationRenderTests, StaticRotate)
{
	std::shared_ptr<VectorFieldRepresentation<void>> vectorRepresentation = makeCloud(makeVertices());

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int numSteps = 100;

	Vector3d startAngles(0.0,0.0,0.0);
	Vector3d endAngles(M_PI_4, M_PI_2, M_PI_2);

	Vector3d startPosition (-0.1, 0.0, -0.0);
	Vector3d endPosition(0.1, 0.0, -0.4);

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		vectorRepresentation->setPose(interpolatePose(startAngles, endAngles, startPosition, endPosition, t));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}
//
//TEST_F(OsgVectorFieldRepresentationRenderTests, DynamicRotate)
//{
//	auto mesh = std::make_shared<std::vector<Vector3d>>(makeVertices());
//	auto vectorRepresentation = makeCloud((*mesh.get()));
//
//	/// Run the thread
//	runtime->start();
//	EXPECT_TRUE(graphicsManager->isInitialized());
//	EXPECT_TRUE(viewElement->isInitialized());
//	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//
//	int numSteps = 100;
//
//	Vector3d startAngles(0.0,0.0,0.0);
//	Vector3d endAngles(M_PI_4, M_PI_2, M_PI_2);
//	Vector3d startPosition (-0.1, 0.0, 0.2);
//	Vector3d endPosition(0.1, 0.0, -0.2);
//
//	for (int i = 0; i < numSteps; ++i)
//	{
//		/// Calculate t in [0.0, 1.0]
//		double t = static_cast<double>(i) / numSteps;
//		RigidTransform3d currentPose =
//			interpolatePose(startAngles, endAngles, startPosition, endPosition, t);
//
//		int id = 0;
//		auto vs = std::make_shared<SurgSim::DataStructures::Vertices<void>>();
//		for (auto it = std::begin((*mesh.get())); it != std::end((*mesh.get())); ++it)
//		{
//			vs->addVertex(SurgSim::DataStructures::Vertices<void>::VertexType(*it));
//		}
//		for (auto it = std::begin((*mesh.get())); it != std::end((*mesh.get())); ++it, ++id)
//		{
//			vs->setVertexPosition(id, currentPose * (*it));
//		}
//		vectorRepresentation->setVertices(vs);
//		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
//	}
//}
//
//TEST_F(OsgVectorFieldRepresentationRenderTests, LineWidth)
//{
//	std::shared_ptr<VectorFieldRepresentation<void>> vectorRepresentation = makeCloud(makeVertices());
//	
//	/// Run the thread
//	runtime->start();
//	EXPECT_TRUE(graphicsManager->isInitialized());
//	EXPECT_TRUE(viewElement->isInitialized());
//	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//
//	int numSteps = 100;
//	double startWidth = 0;
//	double endWidth = 20.0;
//
//	for (int i = 0; i < numSteps; ++i)
//	{
//		/// Calculate t in [0.0, 1.0]
//		double t = static_cast<double>(i) / numSteps;
//		vectorRepresentation->setLineWidth(interpolate(startWidth, endWidth, t));
//		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
//	}
//}
//
//TEST_F(OsgVectorFieldRepresentationRenderTests, ColorTest)
//{
//	std::shared_ptr<VectorFieldRepresentation<void>> vectorRepresentation = makeCloud(makeVertices());
//	/// Run the thread
//	runtime->start();
//	EXPECT_TRUE(graphicsManager->isInitialized());
//	EXPECT_TRUE(viewElement->isInitialized());
//	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//
//	vectorRepresentation->setLineWidth(4.0);
//
//	int numSteps = 100;
//
//	SurgSim::Math::Vector4d startColor(0.0, 1.0, 0.0, 1.0);
//	SurgSim::Math::Vector4d endColor(1.0, 1.0, 1.0, 1.0);
//	for (int i = 0; i < numSteps; ++i)
//	{
//		/// Calculate t in [0.0, 1.0]
//		double t = static_cast<double>(i) / numSteps;
//
//		std::vector<Vector4d> vColor;
//		vColor.push_back(interpolate(startColor,endColor,t));
//	
//
//		vectorRepresentation->setColors(vColor);
//		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
//	}
//
//	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//}

}; // namespace Graphics
}; // namespace SurgSim
