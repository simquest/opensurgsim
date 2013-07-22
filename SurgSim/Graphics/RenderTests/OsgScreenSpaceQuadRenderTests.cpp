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
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/View.h>

#include <SurgSim/Testing/MathUtilities.h>

#include <SurgSim/Graphics/OsgScreenSpaceQuadRepresentation.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

namespace SurgSim 
{
namespace Graphics
{

struct OsgScreenSpaceQuadRenderTests : public ::testing::Test
{
	virtual void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		graphicsManager = std::make_shared<SurgSim::Graphics::OsgManager>();

		runtime->addManager(graphicsManager);

		scene = std::make_shared<SurgSim::Framework::Scene>();
		runtime->setScene(scene);

		viewElement = std::make_shared<OsgViewElement>("view element");
		scene->addSceneElement(viewElement);

	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<OsgManager> graphicsManager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;

protected:


};


TEST_F(OsgScreenSpaceQuadRenderTests, InitTest)
{
	std::shared_ptr<OsgScreenSpaceQuadRepresentation> quad = 
		std::make_shared<OsgScreenSpaceQuadRepresentation>("Screen Quad", viewElement->getView());

	viewElement->addComponent(quad);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	quad->setSize(100,100);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	int width,height;
	viewElement->getView()->getDimensions(&width, &height);


	SurgSim::Math::Vector3d startPosition(0.0,0.0,0.0);
	SurgSim::Math::Vector3d endPosition(width,height,0.0);

	SurgSim::Math::Vector2d startSize(0.0,0.0);
	SurgSim::Math::Vector2d endSize(200,200);

	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		RigidTransform3d currentPose =
			SurgSim::Testing::interpolatePose(Vector3d::Identity(), Vector3d::Identity(), startPosition, endPosition, t);

		quad->setPose(currentPose);

		SurgSim::Math::Vector2d size = SurgSim::Testing::interpolate(startSize,endSize,t);
		quad->setSize(size.x(), size.y());
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

}

TEST_F(OsgScreenSpaceQuadRenderTests, TextureTest)
{

}

}; // namespace Graphics
}; // namespace SurgSim
