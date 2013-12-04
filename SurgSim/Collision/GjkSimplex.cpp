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

#include <SurgSim/Collision/GjkSimplex.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Framework/Assert.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

namespace SurgSim
{
namespace Collision
{

GjkSimplex::GjkSimplex()
{
    m_vertexInMinkowskiSpace.clear();
    m_vertexInA.clear();
    m_vertexInB.clear();
    m_vertexBaryCenterInMinkowskiSpace.clear();
    m_numberOfActiveVertices = 0;
}

void GjkSimplex::addVertex(const SurgSim::Math::Vector3d& vertexInMinkowskiSpace,
                           const SurgSim::Math::Vector3d& vertexInA,
                           const SurgSim::Math::Vector3d& vertexInB)
{
    SURGSIM_ASSERT(m_numberOfActiveVertices < 4)
        << "Attempting to add too many vertices to GjkSimplex object, " << m_numberOfActiveVertices;

    m_vertexInMinkowskiSpace.push_back(vertexInMinkowskiSpace);
    m_vertexInA.push_back(vertexInA);
    m_vertexInB.push_back(vertexInB);
    m_vertexBaryCenterInMinkowskiSpace.push_back(0.0);

    ++m_numberOfActiveVertices;
}

void GjkSimplex::removeVertex(size_t index)
{
    SURGSIM_ASSERT(index < m_numberOfActiveVertices);

    m_vertexInMinkowskiSpace.erase(m_vertexInMinkowskiSpace.begin() + index);
    m_vertexInA.erase(m_vertexInA.begin() + index);
    m_vertexInB.erase(m_vertexInB.begin() + index);
    m_vertexBaryCenterInMinkowskiSpace.erase(m_vertexBaryCenterInMinkowskiSpace.begin() + index);

    --m_numberOfActiveVertices;
}

void GjkSimplex::update(SurgSim::Math::Vector3d *closestPointToOrigin)
{
    using SurgSim::Math::Vector3d;
    using SurgSim::Math::Geometry::DistanceEpsilon;

    // This method should not be called before at least a single vertex is added to the simplex.
    SURGSIM_ASSERT(m_numberOfActiveVertices > 0)
        << "Attempting to update an empty GjkSimplex object";

    // Find the closest point within the simplex to the origin.
    // Note: This point can be the origin itself.

    switch (m_numberOfActiveVertices)
    {
    case 1:
        {
            // The simplex is a single vertex => a.
            // Therefore, closestPointToOrigin is a.
            *closestPointToOrigin = m_vertexInMinkowskiSpace[0];
            m_vertexBaryCenterInMinkowskiSpace[0] = 1.0;
        }
        break;
    case 2:
        closestPointFromLineSegment(closestPointToOrigin);
        break;
    case 3:
        closestPointFromTriangle(closestPointToOrigin);
        break;
    case 4:
        closestPointFromTetrahedron(closestPointToOrigin);
        break;
    }
}

void GjkSimplex::clear()
{
    m_numberOfActiveVertices = 0;
    m_vertexInMinkowskiSpace.clear();
    m_vertexInA.clear();
    m_vertexInB.clear();
    m_vertexBaryCenterInMinkowskiSpace.clear();
}

size_t GjkSimplex::getNumberOfVertices() const
{
    return m_numberOfActiveVertices;
}

SurgSim::Math::Vector3d GjkSimplex::getVertexInMinkowskiSpace(size_t id) const
{
    SURGSIM_ASSERT(id < m_numberOfActiveVertices)
        << "Index out of bounds while retrieving vertex from GjkSimplex";

    return m_vertexInMinkowskiSpace[id];
}

SurgSim::Math::Vector3d GjkSimplex::getVertexInA(size_t id) const
{
    SURGSIM_ASSERT(id < m_numberOfActiveVertices)
        << "Index out of bounds while retrieving vertex from GjkSimplex";

    return m_vertexInA[id];
}

SurgSim::Math::Vector3d GjkSimplex::getVertexInB(size_t id) const
{
    SURGSIM_ASSERT(id < m_numberOfActiveVertices)
        << "Index out of bounds while retrieving vertex from GjkSimplex";

    return m_vertexInB[id];
}

double GjkSimplex::getVertexBarycentricCoordinateInMinkowskiSpace(size_t id) const
{
    SURGSIM_ASSERT(id < m_numberOfActiveVertices)
        << "Index out of bounds while retrieving barycentric coordinate from GjkSimplex";

    return m_vertexBaryCenterInMinkowskiSpace[id];
}

void GjkSimplex::closestPointFromLineSegment(SurgSim::Math::Vector3d *closestPointToOrigin)
{
    // The simplex is a line segment => (a, b).
    //
    // IMPORTANT NOTE:
    // b was added to the simplex by searching in the direction of ao.
    // From this information, we can be sure that the closest point to the origin is not a.
    //
    // Based on that, origin is
    // - ao denotes the vector from a to origin.
    // - bo denotes the vector from b to origin.
    // - ab denotes the line segment vector from a to b.
    //
    //      |             |
    //      |             |
    //      |      ab     |
    //   a  x-------------x  b
    //      |             |
    //      |             |
    //      |             |
    //
    // 2 cases:
    // 1) closestPointToOrigin is within ab => No reduction of simplex.
    // 2) closestPointToOrigin is one of the extremities of ab => The other
    //    extremity is removed from simplex.

    Vector3d ao = -m_vertexInMinkowskiSpace[0];
    Vector3d ab = m_vertexInMinkowskiSpace[1] - m_vertexInMinkowskiSpace[0];
    double aoProjectedOnab = ao.dot(ab);

	// a is NOT the closest point.
	SURGSIM_ASSERT(aoProjectedOnab > 0.0)
		<< "Vertex a in the line segment (simplex) should not be closest to origin";

    double abLengthSquared = ab.squaredNorm();
    if (aoProjectedOnab < abLengthSquared)
    {
        // Closest point is within the line segment.
        double aoProjectedOnNormalizedab = aoProjectedOnab / abLengthSquared;
        *closestPointToOrigin = m_vertexInMinkowskiSpace[0] + ab * aoProjectedOnNormalizedab;
        // The barycentric coordinate of the closestPointToOrigin.
        m_vertexBaryCenterInMinkowskiSpace[0] = 1.0 - aoProjectedOnNormalizedab;
        m_vertexBaryCenterInMinkowskiSpace[1] = aoProjectedOnNormalizedab;
    }
    else
    {
        // b is the closest point.
        // Remove a from simplex.
        removeVertex(0);
        // Closest point is b.
        *closestPointToOrigin = m_vertexInMinkowskiSpace[0];
        // The barycentric coordinate of the closestPointToOrigin.
        m_vertexBaryCenterInMinkowskiSpace[0] = 1.0;
    }
}

void GjkSimplex::closestPointFromTriangle(SurgSim::Math::Vector3d *closestPointToOrigin)
{
    // The simplex is a triangle => (a, b, c).
    //
    // /* Gughan TODO *****************
    // IMPORTANT NOTE:
    // There are two scenarios where a simplex will end up with 3 vertices.
    // 1) c was added to the simplex by searching towards the origin from
    // the line segment (a, b).
    // 2) A vertex was deleted from the tetrahedron.
    // And c is on the other side of origin along the search direction,
    // from the vertices a and b.
    // From this information, we can be sure that the closest point to the origin is
    // on the side of the line ab, where the vertex c is also present.
    // *******************************/
    //
    // - ao denotes the vector from a to origin.
    // - bo denotes the vector from b to origin.
    // - co denotes the vector from c to origin.
    // - ab denotes the triangle edge vector from a to b.
    // - ac denotes the triangle edge vector from a to c.
    // - bc denotes the triangle edge vector from b to c.
    //
    //       ,_       a       _,
    //         `*-=,_   _,=-*`
    //               `X`
    //           ab  / \ ac
    //              /   \
    //   '=.,_     /     \     _,.='
    //        `*-,X-------X,-*`
    //            |  bc   |
    //         b  |       |  c
    //            |       |
    //

    // 7 cases.
    // 1) The origin is closest to a.
    Vector3d ab = m_vertexInMinkowskiSpace[1] - m_vertexInMinkowskiSpace[0];
    Vector3d ac = m_vertexInMinkowskiSpace[2] - m_vertexInMinkowskiSpace[0];
    Vector3d ao = -m_vertexInMinkowskiSpace[0];
    double aoProjectedOnab = ao.dot(ab);
    double aoProjectedOnac = ao.dot(ac);

	SURGSIM_ASSERT(!(aoProjectedOnab < 0.0 && aoProjectedOnac < 0.0))
		<< "Vertex a in the triangle (simplex) should not be closest to origin";
    
	// 2) The origin is closest to b.
    Vector3d bc = m_vertexInMinkowskiSpace[2] - m_vertexInMinkowskiSpace[1];
    Vector3d bo = -m_vertexInMinkowskiSpace[1];
    double boProjectedOnab = bo.dot(ab);
    double boProjectedOnbc = bo.dot(bc);
    
	SURGSIM_ASSERT(!(boProjectedOnab >= 0.0 && boProjectedOnbc < 0.0))
		<< "Vertex b in the triangle (simplex) should not be closest to origin";
    
	// 3) The origin is closest to c.
    Vector3d co = -m_vertexInMinkowskiSpace[2];
    double coProjectedOnbc = co.dot(bc);
    double coProjectedOnac = co.dot(ac);
    if (coProjectedOnac >= 0.0 && coProjectedOnbc >= 0.0)
    {
        removeVertex(1);
        removeVertex(0);
        *closestPointToOrigin = m_vertexInMinkowskiSpace[0];
        // The barycentric coordinate of the closestPointToOrigin.
        m_vertexBaryCenterInMinkowskiSpace[0] = 1.0;
        return;
    }

    // 4) The origin is closest to ab.
    Vector3d triangleNormal = ab.cross(ac);
    double baryCoordC = triangleNormal.dot(ao.cross(bo));

	SURGSIM_ASSERT(!(baryCoordC <= 0.0 && aoProjectedOnab >= 0.0 && boProjectedOnab <= 0.0))
		<< "Edge ab in the triangle (simplex) should not be closest to origin";

    // 5) The origin is closest to ac.
    double baryCoordB = triangleNormal.dot(co.cross(ao));
    if (baryCoordB <= 0.0 && aoProjectedOnac >= 0.0 && coProjectedOnac <= 0.0)
    {
        removeVertex(1);
        // The barycentric coordinate of the closestPointToOrigin.
        m_vertexBaryCenterInMinkowskiSpace[1] = aoProjectedOnac / (aoProjectedOnac - coProjectedOnac);
        m_vertexBaryCenterInMinkowskiSpace[0] = 1.0 - m_vertexBaryCenterInMinkowskiSpace[1];
        // The closest point to origin.
        *closestPointToOrigin = m_vertexInMinkowskiSpace[0] +
                                ac * m_vertexBaryCenterInMinkowskiSpace[1];
        return;
    }

    // 6) The origin is closest to bc.
    double baryCoordA = triangleNormal.dot(bo.cross(co));
    if (baryCoordA <= 0.0 && boProjectedOnbc >= 0.0 && coProjectedOnbc <= 0.0)
    {
        removeVertex(0);
        // The barycentric coordinate of the closestPointToOrigin.
        m_vertexBaryCenterInMinkowskiSpace[1] = boProjectedOnbc / (boProjectedOnbc - coProjectedOnbc);
        m_vertexBaryCenterInMinkowskiSpace[0] = 1.0 - m_vertexBaryCenterInMinkowskiSpace[1];
        // The closest point to origin.
        *closestPointToOrigin = m_vertexInMinkowskiSpace[0] +
                                bc * m_vertexBaryCenterInMinkowskiSpace[1];
        return;
    }

    // 7) The origin is closest to the triangle face.
    // The barycentric coordinate of the closestPointToOrigin.
    double denom = 1.0 / (baryCoordA + baryCoordB + baryCoordC);
    m_vertexBaryCenterInMinkowskiSpace[0] = baryCoordA * denom;
    m_vertexBaryCenterInMinkowskiSpace[1] = baryCoordB * denom;
    m_vertexBaryCenterInMinkowskiSpace[2] = 1.0 - m_vertexBaryCenterInMinkowskiSpace[0] -
                                            m_vertexBaryCenterInMinkowskiSpace[1];
    // The closest point to origin.
    *closestPointToOrigin = m_vertexInMinkowskiSpace[0] * m_vertexBaryCenterInMinkowskiSpace[0] +
                            m_vertexInMinkowskiSpace[1] * m_vertexBaryCenterInMinkowskiSpace[1] +
                            m_vertexInMinkowskiSpace[2] * m_vertexBaryCenterInMinkowskiSpace[2];
}

void GjkSimplex::closestPointFromTetrahedron(SurgSim::Math::Vector3d *closestPointToOrigin)
{
    // The simplex is a tetrahedron => (a, b, c, d).
    //
    // IMPORTANT NOTE:
    // There is only one scenario as to how a simplex will end up with 4 vertices.
    // 1) d was added to the simplex by searching towards the origin from
    // the triangle (a, b, c).
    // From this information, we can be sure that the closest point to the origin is NOT
    // on the side of triangle (a, b, c) opposite to the side where 'd' is.
    //
    // - ao denotes the vector from a to origin.
    // - bo denotes the vector from b to origin.
    // - co denotes the vector from c to origin.
    // - d0 denotes the vector from b to origin.
    // - ab denotes the tetrahedron edge vector from a to b.
    // - ac denotes the tetrahedron edge vector from a to c.
    // - bc denotes the tetrahedron edge vector from b to c.
    // - ad denotes the tetrahedron edge vector from a to d.
    // - bd denotes the tetrahedron edge vector from b to d.
    // - cd denotes the tetrahedron edge vector from c to d.
    //
    //                a
    //
    //                X
    //               /|\
    //              / | \
    //             /  |  \
    //            /   |   \
    //           /    | ad \
    //          /     X     \
    //         /     / \     \
    //        /    /  d  \    \
    //       /   /         \   \
    //      /  / bd      cd \  \
    //     / /                 \ \
    //    X/---------------------\X
    //
    //   b                         c
    //

    Vector3d ad = m_vertexInMinkowskiSpace[0] - m_vertexInMinkowskiSpace[3];
    Vector3d bd = m_vertexInMinkowskiSpace[1] - m_vertexInMinkowskiSpace[3];
    Vector3d cd = m_vertexInMinkowskiSpace[2] - m_vertexInMinkowskiSpace[3];
    Vector3d d0 = -m_vertexInMinkowskiSpace[3];

    // 4 cases
    // 1) Origin is not inside the tetrahedron and closest to triangle, (a, b, d).
    Vector3d triangleNormal = bd.cross(ad);
    double distanceToOriginFromTriangle = triangleNormal.dot(d0);
    double distanceToFourthVertexFromTriangle =
        triangleNormal.dot(m_vertexInMinkowskiSpace[2] - m_vertexInMinkowskiSpace[3]);

    if ((distanceToOriginFromTriangle * distanceToFourthVertexFromTriangle) <= 0.0)
    {
        removeVertex(2);
        closestPointFromTriangle(closestPointToOrigin);
        return;
    }

    // 2) Origin is not inside the tetrahedron and closest to triangle, (b, c, d).
    triangleNormal = bd.cross(cd);
    distanceToOriginFromTriangle = triangleNormal.dot(d0);
    distanceToFourthVertexFromTriangle =
        triangleNormal.dot(m_vertexInMinkowskiSpace[0] - m_vertexInMinkowskiSpace[3]);

    if ((distanceToOriginFromTriangle * distanceToFourthVertexFromTriangle) <= 0.0)
    {
        removeVertex(0);
        closestPointFromTriangle(closestPointToOrigin);
        return;
    }

    // 3) Origin is not inside the tetrahedron and closest to triangle, (a, c, d).
    triangleNormal = cd.cross(ad);
    distanceToOriginFromTriangle = triangleNormal.dot(d0);
    distanceToFourthVertexFromTriangle =
        triangleNormal.dot(m_vertexInMinkowskiSpace[1] - m_vertexInMinkowskiSpace[3]);

    if ((distanceToOriginFromTriangle * distanceToFourthVertexFromTriangle) <= 0.0)
    {
        removeVertex(1);
        closestPointFromTriangle(closestPointToOrigin);
        return;
    }

    // 4) The origin is inside the tetrahedron.
    closestPointToOrigin->setZero();
    // The barycentric coordinate of the closestPointToOrigin.
    // m_vertexBaryCenterInMinkowskiSpace = (b0, b1, b2, b3)
    // [b0,b1,b2] = inverse([a-d b-d c-d]) * (-d)
    // where, b0+b1+b2+b3 = 1.
    Matrix33d barycenterMatrix;
	barycenterMatrix.block(0, 0, 3, 1) = m_vertexInMinkowskiSpace[0] - m_vertexInMinkowskiSpace[3];
    barycenterMatrix.block(0, 1, 3, 1) = m_vertexInMinkowskiSpace[1] - m_vertexInMinkowskiSpace[3];
    barycenterMatrix.block(0, 2, 3, 1) = m_vertexInMinkowskiSpace[2] - m_vertexInMinkowskiSpace[3];
    // Calculate [b0,b1,b2].
    Vector3d b0b1b2 = barycenterMatrix.inverse() * -m_vertexInMinkowskiSpace[3];
    // Calculate m_vertexBaryCenterInMinkowskiSpace = (b0, b1, b2, b3).
    m_vertexBaryCenterInMinkowskiSpace[0] = b0b1b2.x();
    m_vertexBaryCenterInMinkowskiSpace[1] = b0b1b2.y();
    m_vertexBaryCenterInMinkowskiSpace[2] = b0b1b2.z();
    // Now, b3 = 1-b0-b1-b2.
    m_vertexBaryCenterInMinkowskiSpace[3] = 1.0 - b0b1b2.x() - b0b1b2.y() - b0b1b2.z();
}

} // namespace Collision
} // namespace SurgSim