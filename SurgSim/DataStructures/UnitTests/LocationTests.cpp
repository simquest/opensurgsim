// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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
	SurgSim::DataStructures::IndexedLocalCoordinate elementMeshLocalCoordinate(1, SurgSim::Math::Vector4d::Ones());

	EXPECT_NO_THROW({Location location(rigidLocalPosition);});
	EXPECT_NO_THROW({Location location(octreeNodePath);});
	EXPECT_NO_THROW({Location location(index);});
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
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_TRUE(location.rigidLocalPosition.getValue().isApprox(rigidLocalPosition));

		Location location1(location);
		EXPECT_TRUE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
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
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_TRUE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
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
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_TRUE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
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
		EXPECT_FALSE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(triangleMeshLocalCoordinate.index, location.triangleMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location.triangleMeshLocalCoordinate.getValue().coordinate.isApprox(\
			triangleMeshLocalCoordinate.coordinate));

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_TRUE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_FALSE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(triangleMeshLocalCoordinate.index, location1.triangleMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location1.triangleMeshLocalCoordinate.getValue().coordinate.isApprox(\
			triangleMeshLocalCoordinate.coordinate));
	}

	{
		SCOPED_TRACE("Using element mesh local coordinate");
		Location location(elementMeshLocalCoordinate, SurgSim::DataStructures::Location::ELEMENT);
		EXPECT_FALSE(location.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location.octreeNodePath.hasValue());
		EXPECT_FALSE(location.index.hasValue());
		EXPECT_FALSE(location.triangleMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(elementMeshLocalCoordinate.index, location.elementMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location.elementMeshLocalCoordinate.getValue().coordinate.isApprox(\
			elementMeshLocalCoordinate.coordinate));

		Location location1(location);
		EXPECT_FALSE(location1.rigidLocalPosition.hasValue());
		EXPECT_FALSE(location1.octreeNodePath.hasValue());
		EXPECT_FALSE(location1.index.hasValue());
		EXPECT_FALSE(location1.triangleMeshLocalCoordinate.hasValue());
		EXPECT_TRUE(location1.elementMeshLocalCoordinate.hasValue());

		EXPECT_EQ(elementMeshLocalCoordinate.index, location1.elementMeshLocalCoordinate.getValue().index);
		EXPECT_TRUE(location1.elementMeshLocalCoordinate.getValue().coordinate.isApprox(\
			elementMeshLocalCoordinate.coordinate));
	}
}

TEST(LocationTests, IsApprox)
{
	double epsilon = 1e-15;

	{
		SCOPED_TRACE("Rigid location");
		Location rigidLocationOnes(Math::Vector3d::Ones());
		Location rigidLocationZero(Math::Vector3d::Zero());
		Location rigidLocationAlmostOnes(Math::Vector3d(1.0, 1.0, 1 + epsilon * 0.5));
		Location rigidLocationAlmostZero(Math::Vector3d(0.0, 0.0, epsilon * 0.5));

		EXPECT_FALSE(rigidLocationOnes.isApprox(rigidLocationZero, epsilon));
		EXPECT_TRUE(rigidLocationOnes.isApprox(rigidLocationOnes, epsilon));
		EXPECT_TRUE(rigidLocationOnes.isApprox(rigidLocationAlmostOnes, epsilon));
		EXPECT_FALSE(rigidLocationOnes.isApprox(rigidLocationAlmostZero, epsilon));

		EXPECT_TRUE(rigidLocationZero.isApprox(rigidLocationZero, epsilon));
		EXPECT_FALSE(rigidLocationZero.isApprox(rigidLocationOnes, epsilon));
		EXPECT_FALSE(rigidLocationZero.isApprox(rigidLocationAlmostOnes, epsilon));
		EXPECT_TRUE(rigidLocationZero.isApprox(rigidLocationAlmostZero, epsilon));
	}

	{
		SCOPED_TRACE("Octree location");
		std::array<int, 3> path1 = {{0, 1, 2}};
		std::array<int, 3> path2 = {{2, 1, 0}};
		Location octreeLocation1(OctreePath(path1.begin(), path1.end()));
		Location octreeLocation2(OctreePath(path2.begin(), path2.end()));

		EXPECT_TRUE(octreeLocation1.isApprox(octreeLocation1));
		EXPECT_TRUE(octreeLocation2.isApprox(octreeLocation2));
		EXPECT_FALSE(octreeLocation1.isApprox(octreeLocation2));
	}

	{
		SCOPED_TRACE("Index location");
		Location indexLocationOne(1);
		Location indexLocationZero(0);

		EXPECT_TRUE(indexLocationOne.isApprox(indexLocationOne));
		EXPECT_TRUE(indexLocationZero.isApprox(indexLocationZero));
		EXPECT_FALSE(indexLocationZero.isApprox(indexLocationOne));
	}

	{
		SCOPED_TRACE("TriangleMesh location");
		auto vector001 = Math::Vector(3); vector001 << 0, 0, 1;
		auto vector100 = Math::Vector(3); vector100 << 1, 0, 0;
		auto vector010 = Math::Vector(3); vector010 << 0, 1, 1;
		IndexedLocalCoordinate triangleBarycentricCoord11(9, vector001);
		IndexedLocalCoordinate triangleBarycentricCoord12(9, vector100);
		IndexedLocalCoordinate triangleBarycentricCoord2(0, vector010);
		Location triangleMeshLocation11(triangleBarycentricCoord11, Location::TRIANGLE);
		Location triangleMeshLocation12(triangleBarycentricCoord12, Location::TRIANGLE);
		Location triangleMeshLocation2(triangleBarycentricCoord2, Location::TRIANGLE);

		EXPECT_TRUE(triangleMeshLocation11.isApprox(triangleMeshLocation11));
		EXPECT_FALSE(triangleMeshLocation11.isApprox(triangleMeshLocation12));
		EXPECT_FALSE(triangleMeshLocation11.isApprox(triangleMeshLocation2));
	}

	{
		SCOPED_TRACE("Element location");
		auto vector01 = Math::Vector(2); vector01 << 0, 1;
		auto vector10 = Math::Vector(2); vector10 << 1, 0;
		IndexedLocalCoordinate triangleBarycentricCoord11(9, vector01);
		IndexedLocalCoordinate triangleBarycentricCoord12(9, vector10);
		IndexedLocalCoordinate triangleBarycentricCoord2(0, vector10);
		Location triangleMeshLocation11(triangleBarycentricCoord11, Location::ELEMENT);
		Location triangleMeshLocation12(triangleBarycentricCoord12, Location::ELEMENT);
		Location triangleMeshLocation2(triangleBarycentricCoord2, Location::ELEMENT);

		EXPECT_TRUE(triangleMeshLocation11.isApprox(triangleMeshLocation11));
		EXPECT_FALSE(triangleMeshLocation11.isApprox(triangleMeshLocation12));
		EXPECT_FALSE(triangleMeshLocation11.isApprox(triangleMeshLocation2));
	}
}

}; // namespace DataStructures

}; // namespace SurgSim
