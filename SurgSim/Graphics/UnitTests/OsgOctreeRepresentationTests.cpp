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
/// Unit Tests for the OsgOctreeRepresentation class.

#include <gtest/gtest.h>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/Graphics/OsgOctreeRepresentation.h"
#include "SurgSim/Math/OctreeShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Framework::Runtime;
using SurgSim::DataStructures::EmptyData;
using SurgSim::Graphics::OsgOctreeRepresentation;
using SurgSim::Math::OctreeShape;
using SurgSim::Math::Vector3d;

TEST(OsgOctreeRepresentationTests, InitilizationTest)
{
	ASSERT_NO_THROW(std::make_shared<OsgOctreeRepresentation>("Test Octree"));
};

TEST(OsgOctreeRepresentationTests, GetSetUpdateTest)
{
	OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox(Vector3d::Zero(), Vector3d::Ones() * 4.0);
	auto octreeNode = std::make_shared<OctreeShape::NodeType>(boundingBox);
	auto octreeShape = std::make_shared<SurgSim::Math::OctreeShape>(*octreeNode);

	auto runtime = std::make_shared<Runtime>();
	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("Test Octree");
	octreeRepresentation->setOctreeShape(octreeShape);

	EXPECT_TRUE(octreeRepresentation->initialize(runtime));
	EXPECT_TRUE(octreeRepresentation->wakeUp());

	// Set the octree after wake up will cause a assertion failure.
	ASSERT_ANY_THROW({ octreeRepresentation->setOctreeShape(octreeShape); });

	ASSERT_NO_THROW(octreeRepresentation->update(0.1));
}

TEST(OsgOctreeRepresentationTests, SetNodeVisibilityTest)
{
	SurgSim::DataStructures::EmptyData emptyData;

	OctreeShape::NodeType::AxisAlignedBoundingBox boundingBox(Vector3d::Zero(), Vector3d::Ones() * 4.0);
	auto octreeNode = std::make_shared<OctreeShape::NodeType>(boundingBox);
	octreeNode->addData(Vector3d(0.0, 0.0, 0.0), emptyData, 2);
	octreeNode->addData(Vector3d(0.0, 0.0, 1.0), emptyData, 3);

	auto octreeShape = std::make_shared<SurgSim::Math::OctreeShape>(*octreeNode);
	auto octreeRepresentation = std::make_shared<OsgOctreeRepresentation>("TestOctree");

	// Path to leaf node
	SurgSim::DataStructures::OctreePath path;
	path.push_back(0);
	path.push_back(0);

	// Set node visibility when no octree is held by OsgOctreeRepresentation will throw.
	EXPECT_ANY_THROW(octreeRepresentation->setNodeVisible(path, true));

	octreeRepresentation->setOctreeShape(octreeShape);
	EXPECT_NO_THROW(octreeRepresentation->setNodeVisible(path, false));

	// Path to internal node
	SurgSim::DataStructures::OctreePath path2;
	path2.push_back(0);
	EXPECT_NO_THROW(octreeRepresentation->setNodeVisible(path2, true));

	// Invalid path
	SurgSim::DataStructures::OctreePath invalidPath;
	invalidPath.push_back(4);
	invalidPath.push_back(1);
	EXPECT_ANY_THROW(octreeRepresentation->setNodeVisible(invalidPath, true));
}

TEST(OsgOctreeRepresentationTests, SerializationTest)
{
	Runtime runtime("config.txt");
	std::shared_ptr<SurgSim::Math::Shape> octreeShape = std::make_shared<SurgSim::Math::OctreeShape>();
	std::string filename = "OctreeShapeData/staple.ply";
	std::static_pointer_cast<SurgSim::Math::OctreeShape>(octreeShape)->loadOctree(filename);

	std::shared_ptr<SurgSim::Framework::Component> osgOctree = std::make_shared<OsgOctreeRepresentation>("TestOctree");
	osgOctree->setValue("OctreeShape", octreeShape);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*osgOctree));
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Graphics::OsgOctreeRepresentation"];
	EXPECT_EQ(7u, data.size());

	std::shared_ptr<SurgSim::Graphics::OsgOctreeRepresentation> newOsgOctree;
	ASSERT_NO_THROW(newOsgOctree = std::dynamic_pointer_cast<OsgOctreeRepresentation>(
									   node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	EXPECT_EQ("SurgSim::Graphics::OsgOctreeRepresentation", newOsgOctree->getClassName());

	auto newOctree = newOsgOctree->getOctreeShape();
	auto oldOctree = std::static_pointer_cast<OctreeShape>(octreeShape)->getOctree();
	EXPECT_TRUE(newOctree->getOctree()->getBoundingBox().isApprox(oldOctree->getBoundingBox()));
	EXPECT_TRUE(newOsgOctree->getOctreeShape()->isValid());
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_OCTREE, newOctree->getType());
	EXPECT_THROW(newOctree->getVolume(), SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE((newOctree->getCenter() - Vector3d::Zero()).isZero());
	EXPECT_THROW(newOctree->getSecondMomentOfVolume(), SurgSim::Framework::AssertionFailure);
}