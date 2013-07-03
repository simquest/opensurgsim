// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

/// \file
/// Render Tests for the OsgShader class.

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

#include <random>


using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

std::shared_ptr<Shader> loadExampleShader()
{
	std::shared_ptr<Shader> shader = std::make_shared<OsgShader>();

	std::vector<std::string> paths;
	paths.push_back("Data/OsgShaderRenderTests");
	SurgSim::Framework::ApplicationData data(paths);

	std::string vertexShaderPath = data.findFile("shader.vert");
	std::string geometryShaderPath = data.findFile("shader.geom");
	std::string fragmentShaderPath = data.findFile("shader.frag");

	EXPECT_NE("", vertexShaderPath) << "Could not find vertex shader!";
	EXPECT_NE("", geometryShaderPath) << "Could not find geometry shader!";
	EXPECT_NE("", fragmentShaderPath) << "Could not find fragment shader!";

	shader->loadVertexShaderSource(vertexShaderPath);
	shader->loadGeometryShaderSource(geometryShaderPath);
	shader->loadFragmentShaderSource(fragmentShaderPath);

	return shader;
}

/// Pops up a window with a sphere colored by its normals and its mirror along the x-axis is also drawn using the
/// geometry shader
TEST(OsgShaderRenderTests, SphereShaderTest)
{
	ASSERT_TRUE(boost::filesystem::exists("Data"));

	/// Enable OSG info notifications to see the shader compilation results
	osg::NotifySeverity previousNotifyLevel = osg::getNotifyLevel();
	osg::setNotifyLevel(osg::INFO);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics view element to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view element");
	scene->addSceneElement(viewElement);

	/// Add the sphere representation to the view element, no need to make another scene element
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	sphereRepresentation->setRadius(0.25);
	sphereRepresentation->setInitialPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.25, 0.0, -1.0)));

	/// Add a shader to the sphere
	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	std::shared_ptr<Shader> shader = loadExampleShader();

	material->setShader(shader);
	sphereRepresentation->setMaterial(material);

	viewElement->addComponent(sphereRepresentation);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	runtime->stop();

	/// Reset notify level
	osg::setNotifyLevel(previousNotifyLevel);
}

};  // namespace Graphics

};  // namespace SurgSim
