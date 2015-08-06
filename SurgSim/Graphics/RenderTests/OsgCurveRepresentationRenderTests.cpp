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

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgCurveRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"

#include <osg/LineWidth>
#include <osg/Hint>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

namespace SurgSim
{
namespace Graphics
{

struct OsgCurveRepresentationRenderTests : public RenderTest
{


protected:
	DataStructures::VerticesPlain makeCube()
	{
		typedef DataStructures::VerticesPlain::VertexType Vertex;
		DataStructures::VerticesPlain result;
		result.addVertex(Vertex(Vector3d(0.1, -0.1, 0.1)));
		result.addVertex(Vertex(Vector3d(0.1, -0.1, 0.1)));
		result.addVertex(Vertex(Vector3d(-0.1, -0.1, 0.1)));
		result.addVertex(Vertex(Vector3d(-0.1, -0.1, -0.1)));
		result.addVertex(Vertex(Vector3d(0.1, -0.1, -0.1)));

		result.addVertex(Vertex(Vector3d(0.1, 0.1, 0.1)));
		result.addVertex(Vertex(Vector3d(-0.1, 0.1, 0.1)));
		result.addVertex(Vertex(Vector3d(-0.1, 0.1, -0.1)));
		result.addVertex(Vertex(Vector3d(0.1, 0.1, -0.1)));
		return result;
	}
};

TEST_F(OsgCurveRepresentationRenderTests, DynamicRotate)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("Element");
	auto vertices = makeCube();
	auto representation = std::make_shared<OsgCurveRepresentation>("Curve");
	element->addComponent(representation);
	scene->addSceneElement(element);
	viewElement->enableManipulator(true);
	viewElement->setPose(makeRigidTransform(
							 Math::Vector3d(0.0, 0.0, -2.0),
							 Math::Vector3d(0.0, 0.0, 0.0),
							 Math::Vector3d(0.0, 1.0, 0.0)));

	auto material = buildMaterial("Shaders/material_curve.vert", "Shaders/material.frag");
	material->addUniform("vec4", "diffuseColor");
	material->setValue("diffuseColor", Vector4f(0.2, 0.2, 0.9, 1.0));

	material->addUniform("vec4", "specularColor");
	material->setValue("specularColor", Vector4f(0.4, 0.4, 0.4, 1.0));

	material->addUniform("float", "shininess");
	material->setValue("shininess", 64.0f);

	representation->setMaterial(material);
	element->addComponent(material);

	auto light = std::make_shared<Graphics::OsgLight>("Light");
	// light->setLocalPose(Math::makeRigidTranslation(Math::Vector3d(0.5, 0.5, 0.5)));
	viewElement->addComponent(light);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	EXPECT_TRUE(viewElement->isInitialized());
	int numSteps = 10 * 60;

	auto widths = std::make_pair(0.1, 5.0);
	auto tensions = std::make_pair(0.1, 0.9);

	RigidTransform3d transform = makeRigidTransform(makeRotationQuaternion(0.02, Vector3d(0.0, 1.0, 1.0)),
								 Vector3d(0.0, -0.0, 0.0));

	for (int i = 0; i < numSteps; ++i)
	{

		double t = static_cast<double>(i) / numSteps;
		for (size_t i = 0; i < vertices.getNumVertices(); ++i)
		{
			vertices.setVertexPosition(i, transform * vertices.getVertexPosition(i));
		}


		representation->setWidth(interpolate(widths.first, widths.second, t));
		representation->setTension(interpolate(tensions.first, tensions.second, t));
		representation->updateControlPoints(vertices);

		boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0 / 30.0));
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

}

		for (size_t i = 0; i < vertices.getNumVertices(); ++i)
		{
			vertices.setVertexPosition(i, transform * vertices.getVertexPosition(i));
		}

		representation->updateControlPoints(vertices);

		boost::this_thread::sleep(boost::posix_time::milliseconds(10000 / numSteps));
	}
	boost::this_thread::sleep(boost::posix_time::milliseconds(100000));

}


}; // namespace Graphics
}; // namespace SurgSim
