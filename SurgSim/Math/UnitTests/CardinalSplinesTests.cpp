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

/// \file
/// Tests for Cardinal Splines interpolation utilities
///

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Math/CardinalSplines.h"

TEST(CardinalSplinesTests, extension)
{
	{
		SurgSim::DataStructures::VerticesPlain emptyPoints;
		std::vector<SurgSim::Math::Vector3d> result;
		EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::extendControlPoints(emptyPoints, &result));
	}

	{
		SurgSim::DataStructures::VerticesPlain onePoints;
		onePoints.addVertex(SurgSim::DataStructures::VerticesPlain::VertexType(SurgSim::Math::Vector3d(0.0, 1.0, 2.0)));
		std::vector<SurgSim::Math::Vector3d> result;
		EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::extendControlPoints(onePoints, &result));
	}

	{
		SurgSim::DataStructures::VerticesPlain twoPoints;
		twoPoints.addVertex(SurgSim::DataStructures::VerticesPlain::VertexType(SurgSim::Math::Vector3d(0.0, 0.0, 0.0)));
		twoPoints.addVertex(SurgSim::DataStructures::VerticesPlain::VertexType(SurgSim::Math::Vector3d(1.0, 2.0, 3.0)));
		EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::extendControlPoints(twoPoints, nullptr));
	}

	{
		SurgSim::DataStructures::VerticesPlain twoPoints;
		twoPoints.addVertex(SurgSim::DataStructures::VerticesPlain::VertexType(SurgSim::Math::Vector3d(0.0, 0.0, 0.0)));
		twoPoints.addVertex(SurgSim::DataStructures::VerticesPlain::VertexType(SurgSim::Math::Vector3d(1.0, 2.0, 3.0)));
		std::vector<SurgSim::Math::Vector3d> result;
		EXPECT_NO_THROW(SurgSim::Math::CardinalSplines::extendControlPoints(twoPoints, &result));
		EXPECT_EQ(4u, result.size());
		EXPECT_TRUE(result[0].isApprox(SurgSim::Math::Vector3d(-1.0, -2.0, -3.0)));
		EXPECT_TRUE(result[3].isApprox(SurgSim::Math::Vector3d(2.0, 4.0, 6.0)));
	}
}

TEST(CardinalSplinesTests, interpolate)
{
	std::vector<SurgSim::Math::Vector3d> controlPoints;
	std::vector<SurgSim::Math::Vector3d> points;

	controlPoints.push_back(SurgSim::Math::Vector3d(0.0, 1.0, 2.0));
	controlPoints.push_back(SurgSim::Math::Vector3d(0.1, 1.1, 2.1));
	controlPoints.push_back(SurgSim::Math::Vector3d(0.2, 1.2, 2.2));

	// Less than 4 control points.
	EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::interpolate(1, controlPoints, &points, 0.5));

	controlPoints.push_back(SurgSim::Math::Vector3d(0.3, 1.3, 2.3));
	EXPECT_NO_THROW(SurgSim::Math::CardinalSplines::interpolate(0, controlPoints, &points, 0.5));
	EXPECT_EQ(4u, points.size());
	auto control = controlPoints.begin();
	for (auto point : points)
	{
		EXPECT_DOUBLE_EQ(point[0], (*control)[0]);
		EXPECT_DOUBLE_EQ(point[1], (*control)[1]);
		EXPECT_DOUBLE_EQ(point[2], (*control)[2]);
		control++;
	}
	points.clear();

	EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::interpolate(1, controlPoints, &points, -0.5));
	EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::interpolate(1, controlPoints, &points, 1.5));
	EXPECT_ANY_THROW(SurgSim::Math::CardinalSplines::interpolate(1, controlPoints, nullptr, 0.5));

	EXPECT_NO_THROW(SurgSim::Math::CardinalSplines::interpolate(10, controlPoints, &points, 0.5));
	EXPECT_EQ(10u, points.size());
}