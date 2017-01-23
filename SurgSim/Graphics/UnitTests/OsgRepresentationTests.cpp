// This file is a part of the OpenSurgSim project.
// Copyright 2012-2016, SimQuest Solutions Inc.
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
/// Tests for the OsgRepresentation class.

#include <gtest/gtest.h>
#include <osg/Geode>
#include <osg/Geometry>
#include <random>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/UnitTests/MockOsgObjects.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;


namespace SurgSim
{
namespace Graphics
{

TEST(OsgRepresentationTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Representation> representation =
						 std::make_shared<MockOsgRepresentation>("test name");
					});

	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");

	EXPECT_EQ("test name", representation->getName());
	EXPECT_TRUE(representation->isActive());
}

TEST(OsgRepresentationsTests, ParameterTests)
{
	auto representation = std::make_shared<MockOsgRepresentation>("name");

	ASSERT_NO_THROW(representation->setValue("GroupReferences", std::vector<std::string>()));
	ASSERT_NO_THROW(representation->getValue("GroupReferences"));

	ASSERT_NO_THROW(representation->setValue("DrawAsWireFrame", true));
	ASSERT_NO_THROW(representation->getValue("DrawAsWireFrame"));

	ASSERT_NO_THROW(representation->setValue("GenerateTangents", true));
	ASSERT_NO_THROW(representation->getValue("GenerateTangents"));
}

TEST(OsgRepresentationTests, OsgNodeTest)
{
	std::shared_ptr<OsgRepresentation> representation = std::make_shared<MockOsgRepresentation>("test name");

	EXPECT_NE(nullptr, representation->getOsgNode());

	// Check that the OSG node is a group (MockOsgRepresentation passes a new group as the node into the
	// OsgRepresentation constructor)
	osg::ref_ptr<osg::Group> osgGroup = dynamic_cast<osg::Group*>(representation->getOsgNode().get());
	EXPECT_TRUE(osgGroup.valid()) << "Representation's OSG node should be a group!";
}

TEST(OsgRepresentationTests, ActivityTest)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::make_shared<MockOsgRepresentation>("test name");
	std::shared_ptr<Representation> representation = osgRepresentation;

	osg::Switch* switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
	ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";
	ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";

	EXPECT_TRUE(representation->isActive());
	representation->update(0.0);
	EXPECT_TRUE(switchNode->getChildValue(switchNode->getChild(0)));

	representation->setLocalActive(false);
	EXPECT_FALSE(representation->isActive());
	representation->update(0.0);
	EXPECT_FALSE(switchNode->getChildValue(switchNode->getChild(0)));

	representation->setLocalActive(true);
	EXPECT_TRUE(representation->isActive());
	representation->update(0.0);
	EXPECT_TRUE(switchNode->getChildValue(switchNode->getChild(0)));
}

TEST(OsgRepresentationTests, WireFrameTest)
{
	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");

	EXPECT_FALSE(representation->getDrawAsWireFrame());

	representation->setDrawAsWireFrame(true);
	EXPECT_TRUE(representation->getDrawAsWireFrame());
}

TEST(OsgRepresentationTests, PoseTest)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto scene = runtime->getScene();

	std::shared_ptr<Representation> representation = std::make_shared<MockOsgRepresentation>("test name");
	std::shared_ptr<BasicSceneElement> element = std::make_shared<BasicSceneElement>("element");
	element->addComponent(representation);
	scene->addSceneElement(element);
	representation->wakeUp();

	{
		SCOPED_TRACE("Check Initial Pose");
		EXPECT_TRUE(representation->getLocalPose().isApprox(RigidTransform3d::Identity()));
		EXPECT_TRUE(representation->getPose().isApprox(RigidTransform3d::Identity()));
	}

	RigidTransform3d localPose;
	{
		SCOPED_TRACE("Set Local Pose");
		localPose = Math::makeRigidTransform(Quaterniond(Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setLocalPose(localPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(localPose));
	}

	RigidTransform3d elementPose;
	{
		SCOPED_TRACE("Set Element Pose");
		elementPose = Math::makeRigidTransform(Quaterniond(Math::Vector4d::Random()).normalized(), Vector3d::Random());
		element->setPose(elementPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(elementPose * localPose));
	}

	{
		SCOPED_TRACE("Change Local Pose");
		localPose = Math::makeRigidTransform(Quaterniond(Math::Vector4d::Random()).normalized(), Vector3d::Random());
		representation->setLocalPose(localPose);
		EXPECT_TRUE(representation->getLocalPose().isApprox(localPose));
		EXPECT_TRUE(representation->getPose().isApprox(elementPose * localPose));
	}
}

TEST(OsgRepresentationTests, MaterialTest)
{
	std::shared_ptr<OsgRepresentation> osgRepresentation = std::make_shared<MockOsgRepresentation>("test name");
	std::shared_ptr<Representation> representation = osgRepresentation;

	std::shared_ptr<OsgMaterial> osgMaterial = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<Material> material = osgMaterial;
	{
		SCOPED_TRACE("Set material");
		EXPECT_TRUE(representation->setMaterial(material));
		EXPECT_EQ(material, representation->getMaterial());

		osg::Switch* switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
		ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";
		ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";
		EXPECT_EQ(osgMaterial->getOsgStateSet(), switchNode->getChild(0)->getStateSet()) <<
				"State set should be the material's state set!";
	}

	{
		SCOPED_TRACE("Clear material");
		representation->clearMaterial();
		EXPECT_EQ(nullptr, representation->getMaterial());

		osg::Switch* switchNode = dynamic_cast<osg::Switch*>(osgRepresentation->getOsgNode().get());
		ASSERT_NE(nullptr, switchNode) << "Could not get OSG switch node!";
		ASSERT_EQ(1u, switchNode->getNumChildren()) << "OSG switch node should have 1 child, the transform node!";
		EXPECT_NE(osgMaterial->getOsgStateSet(), switchNode->getChild(0)->getStateSet()) <<
				"State set should have been cleared!";
	}
}

TEST(OsgRepresentationTests, AddUniformTest)
{
	std::shared_ptr<OsgRepresentation> representation = std::make_shared<MockOsgRepresentation>("test name");
	{
		float value = 2.0;
		ASSERT_NO_THROW(representation->addUniform("float", "test_float_uniform", value));
	}
	{
		float value = 2.0;
		EXPECT_THROW(representation->addUniform("invalid", "test_float_uniform", value), Framework::AssertionFailure);
	}
	{
		double value = 2.0;
		EXPECT_THROW(representation->addUniform("float", "test_float_uniform", value), boost::bad_any_cast);
	}
	{
		Math::Vector4f vector(1.0, 2.0, 3.0, 4.0);
		ASSERT_NO_THROW(representation->addUniform("vec4", "test_vector_uniform", vector));
	}
	{
		Math::Vector4f vector(1.0, 2.0, 3.0, 4.0);
		EXPECT_THROW(representation->addUniform("dvec4", "test_vector_uniform", vector), boost::bad_any_cast);
	}
	{
		auto texture = std::make_shared<OsgTextureCubeMap>();
		ASSERT_NO_THROW(representation->addUniform("samplerCube", "test_texture_uniform", texture));
	}
}

TEST(OsgRepresentationTests, UpdateTest)
{
	std::shared_ptr<MockOsgRepresentation> mockRepresentation = std::make_shared<MockOsgRepresentation>("test name");
	std::shared_ptr<Representation> representation = mockRepresentation;

	EXPECT_EQ(0, mockRepresentation->getNumUpdates());
	EXPECT_EQ(0.0, mockRepresentation->getSumDt());

	double sumDt = 0.0;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	/// Do 10 updates with random dt and check each time that the number of updates and sum of dt are correct.
	for (int i = 1; i <= 10; ++i)
	{
		double dt = distribution(generator);
		sumDt += dt;

		representation->update(dt);
		EXPECT_EQ(i, mockRepresentation->getNumUpdates());
		EXPECT_LT(fabs(sumDt - mockRepresentation->getSumDt()), Eigen::NumTraits<double>::dummy_precision());
	}
}

TEST(OsgRepresentationTests, GroupTest)
{
	std::shared_ptr<Representation> rep = std::make_shared<MockOsgRepresentation>("TestRepresentation");

	rep->clearGroupReferences();

	EXPECT_TRUE(rep->addGroupReference("group1"));
	EXPECT_FALSE(rep->addGroupReference("group1"));

	EXPECT_TRUE(rep->addGroupReference("group2"));
	EXPECT_TRUE(rep->addGroupReference("group3"));

	std::vector<std::string> groups = rep->getGroupReferences();

	EXPECT_EQ(3U, groups.size());

	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group1"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group2"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group3"));
}

TEST(OsgRepresentationTests, GroupsTest)
{
	std::shared_ptr<Representation> rep = std::make_shared<MockOsgRepresentation>("TestRepresentation");

	rep->clearGroupReferences();

	std::vector<std::string> newGroups;
	newGroups.push_back("group1");
	newGroups.push_back("group1");
	newGroups.push_back("group2");
	newGroups.push_back("group3");

	rep->addGroupReferences(newGroups);
	std::vector<std::string> groups = rep->getGroupReferences();

	EXPECT_EQ(3U, groups.size());

	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group1"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group2"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group3"));

}

TEST(OsgRepresentationTests, SetGroupsTests)
{
	std::shared_ptr<Representation> rep = std::make_shared<MockOsgRepresentation>("TestRepresentation");

	std::vector<std::string> newGroups;
	newGroups.push_back("group1");
	newGroups.push_back("group1");
	newGroups.push_back("group2");
	newGroups.push_back("group3");

	rep->addGroupReference("OtherGroup");
	rep->setGroupReferences(newGroups);

	std::vector<std::string> groups = rep->getGroupReferences();

	EXPECT_EQ(3U, groups.size());

	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group1"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group2"));
	EXPECT_NE(std::end(groups), std::find(std::begin(groups), std::end(groups), "group3"));
}

class  CheckTangentsVisitor : public osg::NodeVisitor
{
public :
	CheckTangentsVisitor() :
		NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{

	}

	virtual ~CheckTangentsVisitor()
	{

	}

	void apply(osg::Node& node) // NOLINT
	{
		traverse(node);
	}

	void apply(osg::Geode& geode) // NOLINT
	{
		// Test object only has 1 geometry ...
		if (geode.getNumDrawables() > 0)
		{
			osg::Geometry* curGeom = geode.getDrawable(0)->asGeometry();
			if (curGeom)
			{
				osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(curGeom->getVertexArray());

				auto tangents =
					dynamic_cast<osg::Vec4Array*>(curGeom->getVertexAttribArray(TANGENT_VERTEX_ATTRIBUTE_ID));
				ASSERT_NE(nullptr, tangents)
						<< "Looks like no tangents where produced, or they are not of type osg::Vec4Array";
				ASSERT_EQ(vertices->size(), tangents->size())
						<< "Looks like number of tangents does not match number of vertices";

				auto bitangents =
					dynamic_cast<osg::Vec4Array*>(curGeom->getVertexAttribArray(BITANGENT_VERTEX_ATTRIBUTE_ID));
				ASSERT_NE(nullptr, bitangents)
						<< "Looks like no bitangents tangents where produced, or they are not of type osg::Vec4Array";
				ASSERT_EQ(vertices->size(), bitangents->size())
						<< "Looks like number of bitangents does not match number of vertices";

			}
		}
	}
};

TEST(OsgRepresentationTests, TangentGenerationTest)
{
	auto runtime = std::make_shared<Framework::Runtime>("config.txt");
	auto scenery = std::make_shared<OsgSceneryRepresentation>("scenery");
	scenery->loadModel("Geometry/sphere0_5.obj");

	EXPECT_FALSE(scenery->isGeneratingTangents());
	scenery->setGenerateTangents(true);
	EXPECT_TRUE(scenery->isGeneratingTangents());

	CheckTangentsVisitor visitor;

	scenery->getOsgNode()->accept(visitor);
}

TEST(OsgRepresentation, MaterialFromReference)
{
	auto runtime = std::make_shared<Framework::Runtime>();
	auto manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	auto scene = runtime->getScene();

	auto element = std::make_shared<Framework::BasicSceneElement>("element");
	auto material1  = std::make_shared<OsgMaterial>("material1");
	element->addComponent(material1);
	auto material2 = std::make_shared<OsgMaterial>("material2");
	element->addComponent(material2);


	auto rep1 = std::make_shared<MockOsgRepresentation>("representation1");
	element->addComponent(rep1);
	auto rep2 = std::make_shared<MockOsgRepresentation>("representation2");
	element->addComponent(rep2);
	auto rep3 = std::make_shared<MockOsgRepresentation>("representation3");
	element->addComponent(rep3);
	scene->addSceneElement(element);


	rep1->setMaterial(material1);
	rep2->setMaterialReference("element/material2");
	rep3->setMaterial(material1);
	rep3->setMaterialReference("element/material2");

	runtime->start();

	while (!rep1->isAwake() || !rep2->isAwake() || !rep3->isAwake())
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}
	EXPECT_EQ("element/material1", rep1->getMaterial()->getFullName());
	EXPECT_EQ("element/material2", rep2->getMaterial()->getFullName());
	EXPECT_EQ("element/material1", rep3->getMaterial()->getFullName());

	runtime->stop();
}

TEST(OsgRepresentation, Serialization)
{
	// Using the scenery representation here, but only testing base class functionality
	auto representation = std::make_shared<OsgBoxRepresentation>("representation1");

	representation->addUniform("float", "floatValue", 2.0f);
	representation->addUniform("double", "doubleValue", 2.0);
	representation->addUniform("bool", "boolValue", false);
	representation->addUniform("vec3", "vec3Value", Math::Vector3f(1.0, 2.0, 3.0));

	YAML::Node node;
	ASSERT_NO_THROW(node = *std::dynamic_pointer_cast<Framework::Component>(representation));
	auto result = node.as<std::shared_ptr<Framework::Component>>();

	EXPECT_EQ(2.0f, result->getValue<float>("floatValue"));
	EXPECT_EQ(2.0, result->getValue<double>("doubleValue"));
	EXPECT_EQ(false, result->getValue<bool>("boolValue"));
	EXPECT_TRUE(Math::Vector3f(1.0, 2.0, 3.0).isApprox(result->getValue<Math::Vector3f>("vec3Value")));

}


}  // namespace Graphics
}  // namespace SurgSim
