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

#include <SurgSim/Collision/GjkSimplex.h>

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

TEST(GjkSimplexTest, UnitTests)
{
	{
		SCOPED_TRACE("Create an object.");
		EXPECT_NO_THROW( { GjkSimplex simplex; } );
		EXPECT_NO_THROW( { std::shared_ptr<GjkSimplex> simplex = std::make_shared<GjkSimplex>(); } );
	}

    GjkSimplex simplex;

	{
		SCOPED_TRACE("Add a vertex.");
		Vector3d vertexMinkowski, vertexA(1.0, 2.0, 3.0), vertexB(4.0, 3.0, 1.0);
		vertexMinkowski = vertexA - vertexB;
		EXPECT_NO_THROW(simplex.addVertex(vertexMinkowski, vertexA, vertexB));
		EXPECT_EQ(1, simplex.getNumberOfVertices());
		EXPECT_NO_THROW(simplex.addVertex(vertexMinkowski, vertexA, vertexB));
		EXPECT_EQ(2, simplex.getNumberOfVertices());
		EXPECT_NO_THROW(simplex.addVertex(vertexMinkowski, vertexA, vertexB));
		EXPECT_EQ(3, simplex.getNumberOfVertices());
		EXPECT_NO_THROW(simplex.addVertex(vertexMinkowski, vertexA, vertexB));
		EXPECT_EQ(4, simplex.getNumberOfVertices());
		EXPECT_ANY_THROW(simplex.addVertex(vertexMinkowski, vertexA, vertexB));
		EXPECT_EQ(4, simplex.getNumberOfVertices());
	}

	{
		SCOPED_TRACE("Clear simplex.");
		EXPECT_NO_THROW(simplex.clear());
		EXPECT_EQ(0, simplex.getNumberOfVertices());
	}
}

TEST(GjkSimplexTest, SinglePointAsSimplex)
{
    // The simplex is a single point.
    GjkSimplex simplex;
    Vector3d closestPointToOrigin, vertexA, vertexB;

    {
		SCOPED_TRACE("Case 1: The point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 2: The point is not the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.001, 0.0001, 0.001), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_FALSE(closestPointToOrigin.isZero());
	}
}

TEST(GjkSimplexTest, LineSegmentAsSimplex)
{
    // The simplex is a line segment.
    GjkSimplex simplex;
    Vector3d closestPointToOrigin, vertexA, vertexB;

    {
		SCOPED_TRACE("Case 1: The first simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d::Ones(), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 2: The second simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Ones(), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 3: The origin is within the line segment.");
		simplex.clear();
		simplex.addVertex(Vector3d::Ones(), vertexA, vertexB);
		simplex.addVertex(-Vector3d::Ones(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 4: The first simplex point is the closest point to origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Ones(), vertexA, vertexB);
		simplex.addVertex(Vector3d::Constant(5.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 5: The second simplex point is the closest point to origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Constant(5.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Ones(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d::Ones()));
	}

    {
		SCOPED_TRACE("Case 6: The closest point to origin is within the line segment.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 1.0, 0.0)));
	}
}

TEST(GjkSimplexTest, TriangleAsSimplex)
{
    // The simplex is a triangle => (a,b,c).
    GjkSimplex simplex;
    Vector3d closestPointToOrigin, vertexA, vertexB;

    {
		SCOPED_TRACE("Case 1: The first simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 2: The second simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 3: The third simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 4: The origin is within the line segment, ab.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 5: The origin is within the line segment, bc.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 6: The origin is within the line segment, ca.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 7: The first simplex point is the closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(3.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 3.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 8: The second simplex point is the closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(2.0, 3.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(3.0, 0.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 9: The third simplex point is the closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(3.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 3.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(1.0, 1.0, 0.0)));
	}

    {
		SCOPED_TRACE("Case 10: The origin is closest to the line segment, ab.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 3.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 11: The origin is closest to the line segment, bc.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 3.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 1.0, 0.0)));
	}

    {
		SCOPED_TRACE("Case 12: The origin is closest to the line segment, ca.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 3.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 1.0, 0.0)));
	}

    {
		SCOPED_TRACE("Case 13: The origin is within the triangle, abc.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 14: The origin is closest to the triangle, abc.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.1), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.0, 0.1)));
	}

    {
		SCOPED_TRACE("Case 15: The origin is closest to the triangle, abc.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, -1.0, -0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, -0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, -0.1), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.0, -0.1)));
	}
}

TEST(GjkSimplexTest, TetrahedronAsSimplex)
{
    // The simplex is a tetrahedron => (a,b,c,d).
    GjkSimplex simplex;
    Vector3d closestPointToOrigin, vertexA, vertexB;

    {
		SCOPED_TRACE("Case 1: The first simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 2: The second simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 3: The third simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 4: The fourth simplex point is the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(2.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 2.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d::Zero(), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 5: The origin is within the line segment, ab.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 6: The origin is within the line segment, bc.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 7: The origin is within the line segment, cd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 8: The origin is within the line segment, da.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 9: The origin is within the line segment, ac.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 10: The origin is within the line segment, bd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.5, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 11: The first simplex point is closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(3.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 3.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 2.0, 2.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 12: The second simplex point is closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(2.0, 2.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(3.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 3.0, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 13: The third simplex point is closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(2.0, 3.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 2.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(3.0, 1.0, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 14: The fourth simplex point is closest to the origin.");
		simplex.clear();
		simplex.addVertex(Vector3d(3.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 3.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(2.0, 2.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 1.0, 1.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(1.0, 1.0, 1.0)));
	}

    {
		SCOPED_TRACE("Case 15: The origin is closest to the line segment, ab.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 16: The origin is closest to the line segment, bc.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 17: The origin is closest to the line segment, cd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.5, 0.0)));
	}

    {
		SCOPED_TRACE("Case 18: The origin is closest to the line segment, da.");
		simplex.clear();
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.5, 0.0)));
	}

    {
		SCOPED_TRACE("Case 19: The origin is closest to the line segment, ac.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		EXPECT_ANY_THROW(simplex.update(&closestPointToOrigin));
	}

    {
		SCOPED_TRACE("Case 20: The origin is closest to the line segment, bd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.5, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, 0.5, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.5, 0.0)));
	}

    {
		SCOPED_TRACE("Case 21: The origin is within the triangle, abc.");
		simplex.clear();
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    // Case 22: The origin is closest to the triangle, abc.
    // This test case is not checked because this case will never occur
    // while running GJK algorithm.

    {
		SCOPED_TRACE("Case 23: The origin is within the triangle, bcd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 24: The origin is closest to the triangle, bcd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 1.0, 0.1), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.0, 0.1)));
	}

    {
		SCOPED_TRACE("Case 25: The origin is within the triangle, cda.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 26: The origin is closest to the triangle, cda.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.0, 0.1)));
	}

    {
		SCOPED_TRACE("Case 25: The origin is within the triangle, abd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}

    {
		SCOPED_TRACE("Case 26: The origin is closest to the triangle, abd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, 2.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 0.1), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isApprox(Vector3d(0.0, 0.0, 0.1)));
	}

    {
		SCOPED_TRACE("Case 27: The origin is within the tetrahedron, abcd.");
		simplex.clear();
		simplex.addVertex(Vector3d(0.0, 1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(-1.0, -1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(1.0, -1.0, 1.0), vertexA, vertexB);
		simplex.addVertex(Vector3d(0.0, 0.0, -1.0), vertexA, vertexB);
		simplex.update(&closestPointToOrigin);
		EXPECT_TRUE(closestPointToOrigin.isZero());
	}
}

} // namespace Collision
} // namespace SurgSim