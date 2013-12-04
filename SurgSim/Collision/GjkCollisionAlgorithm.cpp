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

#include <SurgSim/Collision/GjkCollisionAlgorithm.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Collision/GjkSimplex.h>

namespace SurgSim
{
namespace Collision
{

SurgSim::Math::Vector3d GjkCollisionAlgorithm::getSeparatingAxis() const
{
    return m_separatingAxis;
}

SurgSim::Math::Vector3d GjkCollisionAlgorithm::getVertexInA() const
{
    return m_vertexInA;
}

SurgSim::Math::Vector3d GjkCollisionAlgorithm::getVertexInB() const
{
    return m_vertexInB;
}

unsigned int GjkCollisionAlgorithm::getMaximumIterations() const
{
    return m_maximumIterations;
}

void GjkCollisionAlgorithm::setMaximumIterations(const unsigned int maximumIterations)
{
    m_maximumIterations = maximumIterations;
}

bool GjkCollisionAlgorithm::detectCollision(std::shared_ptr<SurgSim::Collision::CollisionPair> pair)
{
    using SurgSim::Math::Vector3d;
    using SurgSim::Math::RigidTransform3d;
    using SurgSim::Math::Geometry::ScalarEpsilon;
    using SurgSim::Math::Geometry::DistanceEpsilon;
    using SurgSim::Math::Geometry::SquaredDistanceEpsilon;

    // Get the relevant objects.
    std::shared_ptr<CollisionRepresentation> representationConvexA = pair->getFirst();
    std::shared_ptr<CollisionRepresentation> representationConvexB = pair->getSecond();

    std::shared_ptr<Shape> convexA = representationConvexA->getShape();
    std::shared_ptr<Shape> convexB = representationConvexB->getShape();

    // Minkowski sum:
    //  Let A be a set of n vertices and B be a set of m vertices.
    //  The Minkowski sum of A and B is the set of n.m vertices which is obtained by
    //  adding every vertex in B to every vertex in A.
    //  => Minkowski(A,B)[i][j] = A[i] + B[j], 0<i<n and 0<j<m.
    //
    // Minkowski difference:
    //  Let A be a set of n vertices and B be a set of m vertices.
    //  The Minkowski difference of A and B is the set of n.m vertices which is obtained by
    //  subtracting every vertex in B from every vertex in A.
    //  => Minkowski(A,-B)[i][j] = A[i] - B[j], 0<i<n and 0<j<m.
    //
    // Presence of collision in convex objects:
    //  Let A be a set of n vertices, which make a convex shape.
    //  Let B be a set of m vertices, which make a convex shape.
    //  If Minkowski(A,-B) is computed, it represents a convex shape, which is formed by
    //  sweeping the mirror image of B over the area of A.
    //  Every point in Minkowski(A,-B) represents a vector originating from within the shape of
    //  B and ending within the shape of A. It there is any overlap between A and B, then some of
    //  the terms in Minkowski(A,-B) will be zero.
    //
    // Algorithm to check if Minkowski(A,-B) contains the origin:
    //  Let {S#}, be the set of vertices, S0, S1, S2 and S3. This is referred to as the simplex.
    //  The objective of the algorithm is to find these 4 vertices in Minkowski(A,-B), which
    //  encloses the origin.
    //  1) Choose a point, P, in Minkowski(A,-B). Preferably Centeroid(A) - Centeroid(B).
    //     If that turns out to be zero, the shapes are in collision => Go to step 7.
    //  2) From P, find the direction, D, towards the origin => (0 - P) => -P.
    //  3) Find the farthest point, S, along D on the Minkowski(A,-B).
    //     If DotProduct(S, D) >= 0, add S to {S#}. Else, go to step 8.
    //  4) Check if {S#} encloses the origin. If it does, go to step 7.
    //  5) From {S#}, find the closest point, P, to the origin. Adjust {S#} accordingly.
    //  6) Go to step 2.
    //  7) There is a collision. Return true.
    //  8) There is no collision. Return false.

    // Flag to set if a collision is detected.
    bool collisionDetected = false;

    // The shapes transforms.
    RigidTransform3d convexTransformA = representationConvexA->getPose();
    RigidTransform3d convexTransformB = representationConvexB->getPose();

    // The inverse of the shapes transforms.
    // Used to transform the world search direction to local shape's coordinate system.
    RigidTransform3d convexTransformInverseA = convexTransformA.inverse();
    RigidTransform3d convexTransformInverseB = convexTransformB.inverse();

    // Check if the existing m_separatingAxis is zero.
    // If it is, create an initial guess for it based on center of mass of the shapes.
    // If not, use the existing m_separatingAxis as the initial guess.
    if (m_separatingAxis.isZero())
    {
        // Initial guess for a couple of vertices, one each in each shapes.
        m_vertexInA = convexTransformA * convexA->calculateMassCenter();
        m_vertexInB = convexTransformB * convexB->calculateMassCenter();

        // Find a vertex in the Minkowski(A,-B), which is also a vector in
        // the space containing the shapes, to use as initial search direction.
        m_separatingAxis = m_vertexInA - m_vertexInB;
    }

    // If this vector is zero, then there is a collision.
    if (m_separatingAxis.isZero())
    {
        collisionDetected = true;
    }
    else
    {
        // The simplex object.
        GjkSimplex gjkSimplex;
        // We have a valid intial search direction => m_separatingAxis.
        Vector3d direction = -m_separatingAxis;
        // Go into the GJK loop to find a simplex enclosing the origin.
        // The loop variables:
        Vector3d directionALocal, directionBLocal;
        std::pair<Vector3d, double> farthestPointOnALocal, farthestPointOnBLocal;
        Vector3d newSimplexVertex;
        Vector3d closestPointOnSimplexToOrigin;
        double squaredShortestDistanceToOrigin = 0.0;

        for (unsigned int currentIterationGJK = 0; currentIterationGJK < m_maximumIterations; ++currentIterationGJK)
        {
            // Transform the search direction to the local shape co-ordinate system.
            directionALocal = convexTransformInverseA.rotation() * direction;
            directionBLocal = convexTransformInverseB.rotation() * -direction;

            // Find the farthest vertex on the shapes, along the search direction.
            convexA->farthestPointAlongDirection(directionALocal, &farthestPointOnALocal);
            convexB->farthestPointAlongDirection(directionBLocal, &farthestPointOnBLocal);

            // Transform the farthest points on shapes to global co-ordinate system.
            // This is to find the Minkowski(A,-B) in global co-ordinates.
            m_vertexInA = convexTransformA * farthestPointOnALocal.first;
            m_vertexInB = convexTransformB * farthestPointOnBLocal.first;
            newSimplexVertex = m_vertexInA - m_vertexInB;

            // If DotProduct(newSimplexVertex, direction) < 0, there is no collision.
            if (direction.dot(newSimplexVertex) < 0.0)
            {
                collisionDetected = false;
                break;
            }

            // Add newSimplexVertex to simplex.
            gjkSimplex.addVertex(newSimplexVertex, m_vertexInA, m_vertexInB);

            // Update the simplex and get the new closest point to origin.
            gjkSimplex.update(&closestPointOnSimplexToOrigin);

            // Find the squaredDistance of the search direction.
            squaredShortestDistanceToOrigin = closestPointOnSimplexToOrigin.squaredNorm();

            // If squaredShortestDistanceToOrigin is zero, then there is collision.
            if (squaredShortestDistanceToOrigin < SquaredDistanceEpsilon)
            {
                collisionDetected = true;
                // The search direction which resulted in the simplex enclosing the origin.
                m_separatingAxis = -direction;
                // The vertex in convexA and convexB which are the penetration points.
                m_vertexInA.setZero();
                m_vertexInB.setZero();
                double barycentricCoordinate = 0.0;
                for (unsigned int i = 0; i < gjkSimplex.getNumberOfVertices(); ++i)
                {
                    barycentricCoordinate = gjkSimplex.getVertexBarycentricCoordinateInMinkowskiSpace(i);
                    m_vertexInA += gjkSimplex.getVertexInA(i) * barycentricCoordinate;
                    m_vertexInB += gjkSimplex.getVertexInB(i) * barycentricCoordinate;
                }
                break;
            }

            // Update the search direction.
            direction = -closestPointOnSimplexToOrigin;
        }
    }

    return collisionDetected;
}

} // namespace Collision
} // namespace SurgSim