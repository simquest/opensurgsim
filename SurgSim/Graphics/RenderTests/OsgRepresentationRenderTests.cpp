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
/// Render Tests for the OsgBoxRepresentation class.

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgCylinderRepresentation.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

#include <gtest/gtest.h>

#include <random>

#include <osgUtil/SmoothingVisitor>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{

struct OsgRepresentationRenderTests : public RenderTest
{

};

/// This test will put all shape one by one along the X-axis
/// To make sure all shapes are aligned.
/// X-axis points horizontally to the right
/// Y-axis points vertically up
/// Z-axis is perpendicular to the screen and points out
TEST_F(OsgRepresentationRenderTests, RepresentationTest)
{
	///	Box position
	Vector3d boxPosition(0.05, 0.0, -0.2);
	/// Capsule position
	Vector3d capsulePosition(-0.05, 0.0, -0.2);
	/// Cylinder position
	Vector3d cylinderPosition(-0.025, 0.0, -0.2);
	/// Sphere position
	Vector3d spherePosition(0.025, 0.0, -0.2);
	/// Size of the box
	Vector3d boxSize(0.01, 0.015, 0.01);
	/// Size of the capsule (radius, height)
	Vector2d capsuleSize(0.005, 0.015);
	/// Size of the cylinder
	Vector2d cylinderSize(0.005, 0.015);
	/// Radius of the sphere
	double sphereRadius = 0.005;

	/// Add representations to the view element so we don't need to make another concrete scene element
	std::shared_ptr<BoxRepresentation> boxRepresentation =
		std::make_shared<OsgBoxRepresentation>("box representation");
	viewElement->addComponent(boxRepresentation);

	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("capsule representation");
	viewElement->addComponent(capsuleRepresentation);

	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation");
	viewElement->addComponent(cylinderRepresentation);

	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	viewElement->addComponent(sphereRepresentation);


	std::shared_ptr<AxesRepresentation> axesRepresentation =
		std::make_shared<OsgAxesRepresentation>("axes");
	viewElement->addComponent(axesRepresentation);

	/// Run the thread
	runtime->start();

	boxRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), boxPosition));
	capsuleRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), capsulePosition));
	cylinderRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), cylinderPosition));
	sphereRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), spherePosition));
	axesRepresentation->setLocalPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0, 0.0, -0.2)));

	/// Set the size of box
	boxRepresentation->setSizeXYZ(boxSize.x(), boxSize.y(), boxSize.z());
	/// Set the size of capsule
	/// Capsule should use Y-axis as its axis
	capsuleRepresentation->setSize(capsuleSize.x(), capsuleSize.y());
	/// Set the size of cylinder
	/// Cylinder should use Y-axis as its axis
	cylinderRepresentation->setSize(cylinderSize.x(), cylinderSize.y());
	/// Set the size of sphere
	sphereRepresentation->setRadius(sphereRadius);

	axesRepresentation->setSize(0.01);

	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

	boxRepresentation->setDrawAsWireFrame(true);
	capsuleRepresentation->setDrawAsWireFrame(true);
	cylinderRepresentation->setDrawAsWireFrame(true);
	sphereRepresentation->setDrawAsWireFrame(true);

	boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
}

class  LineGeometryVisitor : public osg::NodeVisitor
{
public :
	LineGeometryVisitor() :
		m_normalsScale(0.1),
		NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
	};

	virtual ~LineGeometryVisitor() {};

	void apply(osg::Node& node)
	{
		traverse(node);
	}

	void apply(osg::Geode& geode)
	{
		osg::StateSet* state = nullptr;
		unsigned int vertNum = 0;
		unsigned int numGeoms = geode.getNumDrawables();

		// Only deal with 1 geometry for now ...
		if (geode.getNumDrawables() > 0)
		{
			osg::Geometry* curGeom = geode.getDrawable(0)->asGeometry();
			if (curGeom)
			{
				osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(curGeom->getVertexArray());

				auto normals = curGeom->getNormalArray();
				geode.addDrawable(buildGeometry(vertices, normals, osg::Vec4(0.0, 0.0, 1.0, 1.0)));

				auto tangents = curGeom->getVertexAttribArray(7);

				auto cast = dynamic_cast<osg::Vec4Array*>(tangents);
				ASSERT_NE(nullptr, cast);
				ASSERT_EQ(vertices->size(), cast->size());
				geode.addDrawable(buildGeometry(vertices, tangents, osg::Vec4(1.0, 0.0, 0.0, 1.0)));

				auto bitangents = curGeom->getVertexAttribArray(8);
				cast = dynamic_cast<osg::Vec4Array*>(bitangents);
				ASSERT_NE(nullptr, cast);
				ASSERT_EQ(vertices->size(), cast->size());
				geode.addDrawable(buildGeometry(vertices, bitangents, osg::Vec4(0.0, 1.0, 0.0, 1.0)));

			}
		}
	}

	osg::Geometry* buildGeometry(osg::Vec3Array* geomVertices, osg::Array* directions, osg::Vec4 color)
	{
		// create Geometry object to store all the vertices and lines primitive.
		osg::Geometry* linesGeom = new osg::Geometry();

		osg::Vec3Array* vertices = new osg::Vec3Array(geomVertices->size() * 2);
		osg::Vec3 direction;
		for (int i = 0; i < geomVertices->size(); ++i)
		{

			(*vertices)[i * 2] = (*geomVertices)[i];
			switch (directions->getType())
			{
				case osg::Array::Vec3ArrayType:
					direction = static_cast<const osg::Vec3Array&>(*directions)[i];
					break;
				case osg::Array::Vec4ArrayType:
					for (int j = 0; j < 3; ++j)
					{
						direction[j] = static_cast<const osg::Vec4Array&>(*directions)[i].ptr()[j];
					}
					break;
				default:
					SURGSIM_FAILURE() << "Unhandled Array type.";
			}
			(*vertices)[i * 2 + 1] = (*geomVertices)[i] + direction * m_normalsScale;
		}

		// pass the created vertex array to the points geometry object.
		linesGeom->setVertexArray(vertices);

		// set the colors as before, plus using the above
		osg::Vec4Array* colors = new osg::Vec4Array;
		colors->push_back(color);
		linesGeom->setColorArray(colors, osg::Array::BIND_OVERALL);

		// 	set the normal in the same way color.
		osg::Vec3Array* normals = new osg::Vec3Array;
		normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
		linesGeom->setNormalArray(normals, osg::Array::BIND_OVERALL);

		linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, vertices->size()));

		osg::StateSet* state = linesGeom->getOrCreateStateSet();
		state->setMode(GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF);

		return linesGeom;
	}

private:
	float m_normalsScale;
};


TEST_F(OsgRepresentationRenderTests, TangentTest)
{
	auto element = std::make_shared<Framework::BasicSceneElement>("sphere");
	auto graphics = std::make_shared<OsgSceneryRepresentation>("sphere");
	graphics->loadModel("OsgRepresentationRenderTests/sphere0_5.obj");
	//graphics->setDrawAsWireFrame(true);

	viewElement->enableManipulator(true);

	camera->setLocalPose(Math::makeRigidTransform(
							 Vector3d(0.0, 0.0, -4.0),
							 Vector3d(0.0, 0.0, 0.0),
							 Vector3d(0.0, 1.0, 0.0)));


	bool done = false;
	osg::ref_ptr<osg::Node> node = graphics->getOsgNode();
	// Generate normals
	osgUtil::SmoothingVisitor sv;
	node->accept(sv);

	graphics->setGenerateTangents(true);

	LineGeometryVisitor visitor;
	node->accept(visitor);

	element->addComponent(graphics);
	scene->addSceneElement(element);

	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	runtime->stop();
}


};  // namespace Graphics

};  // namespace SurgSim
