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

#include "SurgSim/DataStructures/Location.h"

namespace SurgSim
{

namespace DataStructures
{

TEST(LocationTests, Constructor)
{
	SurgSim::Math::Vector3d rigidLocalPosition = SurgSim::Math::Vector3d::Ones();
	SurgSim::DataStructures::OctreePath octreeNodePath;
	octreeNodePath.push_back(1);
	octreeNodePath.push_back(2);
	octreeNodePath.push_back(3);
	size_t index(3);
	SurgSim::DataStructures::IndexedLocalCoordinate triangleMeshLocalCoordinate(1, SurgSim::Math::Vector2d(4.0, 5.0));
	SurgSim::DataStructures::IndexedLocalCoordinate nodeMeshLocalCoordinate(1, SurgSim::Math::Vector());
	SurgSim::DataStructures::IndexedLocalCoordinate elementMeshLocalCoordinate(1, SurgSim::Math::Vector4d::Ones());

	EXPECT_NO_THROW({Location location(rigidLocalPosition);});
	EXPECT_NO_THROW({Location location(octreeNodePath);});
	EXPECT_NO_THROW({Location location(index);});
	EXPECT_NO_THROW({Location location(nodeMeshLocalCoordinate, SurgSim::DataStructures::Location::NODE);});
	EXPECT_NO_THROW({Location location(triangleMeshLocalCoordinate, SurgSim::DataStructures::Location::TRIANGLE);});
	EXPECT_NO_THROW({Location location(elementMeshLocalCoordinate, SurgSim::DataStructures::Location::ELEMENT);});
	EXPECT_THROW({Location location(elementMeshLocalCoordinate,\
		static_cast<SurgSim::DataStructures::Location::Type>(SurgSim::DataStructures::Location::ELEMENT + 10));},\
		SurgSim::Framework::AssertionFailure);

	{
		SCOPED_TRACE("Using rigid local position");

		Location location(rigidLocalPosition);
		EXPECT_TRUE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_TRUE(location.rigidLocalPosition.getValue().isApprox(rigidLocalPosition));

		Location location1(location);
		EXPECT_TRUE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_TRUE(location1.rigidLocalPosition.getValue().isApprox(rigidLocalPosition));
	}

	{
		SCOPED_TRACE("Using octree node path");
		Location location(octreeNodePath);

		EXPECT_EQ(octreeNodePath, location.octreeNodePath.getValue());
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_TRUE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_TRUE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(octreeNodePath, location1.octreeNodePath.getValue());
	}

	{
		SCOPED_TRACE("Using index");
		Location location(index);

		EXPECT_EQ(index, location.index.getValue());
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_TRUE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_TRUE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(index, location1.index.getValue());
	}

	{
		SCOPED_TRACE("Using triangle mesh local coordinate");
		Location location(triangleMeshLocalCoordinate, SurgSim::DataStructures::Location::TRIANGLE);
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_TRUE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(triangleMeshLocalCoordinate.index, location.triangleMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location.triangleMeshLocalCoordinate.getValue().coordinate.isApprox(\
			triangleMeshLocalCoordinate.coordinate));

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_TRUE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(triangleMeshLocalCoordinate.index, location1.triangleMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location1.triangleMeshLocalCoordinate.getValue().coordinate.isApprox(\
			triangleMeshLocalCoordinate.coordinate));
	}

	{
		SCOPED_TRACE("Using node mesh local coordinate");
		Location location(nodeMeshLocalCoordinate, SurgSim::DataStructures::Location::NODE);
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(nodeMeshLocalCoordinate.index, location.nodeMeshLocalCoordinate.getValue().index);

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(nodeMeshLocalCoordinate.index, location1.nodeMeshLocalCoordinate.getValue().index);
	}

	{
		SCOPED_TRACE("Using element mesh local coordinate");
		Location location(elementMeshLocalCoordinate, SurgSim::DataStructures::Location::ELEMENT);
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.nodeMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(elementMeshLocalCoordinate.index, location.elementMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location.elementMeshLocalCoordinate.getValue().coordinate.isApprox(\
			elementMeshLocalCoordinate.coordinate));

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.nodeMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(elementMeshLocalCoordinate.index, location1.elementMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location1.elementMeshLocalCoordinate.getValue().coordinate.isApprox(\
			elementMeshLocalCoordinate.coordinate));
	}
}

}; // namespace DataStructures

}; // namespace SurgSim
