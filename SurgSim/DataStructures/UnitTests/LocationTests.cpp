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
	SurgSim::DataStructures::IndexedLocalCoordinate meshLocalCoordinate(1, SurgSim::Math::Vector2d(4.0, 5.0));

	EXPECT_NO_THROW({Location location(rigidLocalPosition);});
	EXPECT_NO_THROW({Location location(octreeNodePath);});
	EXPECT_NO_THROW({Location location(meshLocalCoordinate);});

	{
		SCOPED_TRACE("Using rigid local position");
		Location location(rigidLocalPosition);
		EXPECT_FALSE(location.meshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_TRUE(location.rigidLocalPosition.hasValue());
		EXPECT_TRUE(location.rigidLocalPosition.getValue().isApprox(rigidLocalPosition));
		Location location1(location);
		EXPECT_FALSE(location1.meshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_TRUE(location1.rigidLocalPosition.hasValue());
		EXPECT_TRUE(location1.rigidLocalPosition.getValue().isApprox(rigidLocalPosition));
	}

	{
		SCOPED_TRACE("Using octree node path");
		Location location(octreeNodePath);
		EXPECT_FALSE(location.meshLocalCoordinate.hasValue());
		EXPECT_TRUE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_EQ(octreeNodePath, location.octreeNodePath.getValue());
		Location location1(location);
		EXPECT_FALSE(location1.meshLocalCoordinate.hasValue());
		EXPECT_TRUE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_EQ(octreeNodePath, location1.octreeNodePath.getValue());
	}

	{
		SCOPED_TRACE("Using mesh local coordinate");
		Location location(meshLocalCoordinate);
		EXPECT_TRUE(location.meshLocalCoordinate.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_EQ(meshLocalCoordinate.index, location.meshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location.meshLocalCoordinate.getValue().coordinate.isApprox(meshLocalCoordinate.coordinate));
		Location location1(location);
		EXPECT_TRUE(location1.meshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_EQ(meshLocalCoordinate.index, location1.meshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location1.meshLocalCoordinate.getValue().coordinate.isApprox(meshLocalCoordinate.coordinate));
	}
}

}; // namespace DataStructures

}; // namespace SurgSim
