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

#ifndef SURGSIM_COLLISION_GJKSIMPLEX_H
#define SURGSIM_COLLISION_GJKSIMPLEX_H

#include <memory>
#include <vector>
#include <deque>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Geometry.h>

namespace SurgSim
{
namespace Collision
{

/// Class to maintain the list of (up to four) vertices which are a subset of the Miskowski(A,-B).
/// The objective is to enclose the origin with the tetahedron formed with the 4 vertices.
/// The vertices in the list are in the order in which they were added to the simplex. Maintaining
/// this order presents the opportunity of making some assumptions when searching for the
/// closest point to the origin from the simplex.
class GjkSimplex
{
public:
    /// Constructor.
    GjkSimplex();

    /// Add a vertex to the simplex.
    /// \param vertexInMinkowskiSpace The vertex in Minkowski(A,-B).
    /// \param vertexInA The vertex in A.
    /// \param vertexInB The vertex in B.
    void addVertex(const SurgSim::Math::Vector3d& vertexInMinkowskiSpace,
                   const SurgSim::Math::Vector3d& vertexInA,
                   const SurgSim::Math::Vector3d& vertexInB);

    /// Find the closest point from the simplex to the origin.
    /// \param closestPointToOrigin The closest vertex on the simplex to the origin.
    void update(SurgSim::Math::Vector3d *closestPointToOrigin);

    /// Clear the simplex.
    void clear();

    /// Get the number of vertices in this simplex.
    /// \return The number of vertices in this simplex.
    size_t getNumberOfVertices() const;

    /// Get the vertex in Minkowski difference space (A, -B) for the given id.
    /// \return The vertex in Minkowski difference space (A, -B).
    SurgSim::Math::Vector3d getVertexInMinkowskiSpace(size_t id) const;

    /// Get the vertex in A, given the id.
    /// \return The vertex in A.
    SurgSim::Math::Vector3d getVertexInA(size_t id) const;

    /// Get the vertex in B, given the id.
    /// \return The vertex in B.
    SurgSim::Math::Vector3d getVertexInB(size_t id) const;

    /// Get the barycentric coordinate for the given vertex id.
    /// \return The barycentric coordinate for the vertex id.
    double getVertexBarycentricCoordinateInMinkowskiSpace(size_t id) const;

private:
    /// Set the flag to remove the vertex whose index is specified.
    /// \param index The index of the vertex to be removed.
    void removeVertex(size_t index);

    /// Find the closest point to the origin from the simplex, which is a line segment.
    /// \param [out] closestPointToOrigin The closest point on the line segment to the origin.
    void closestPointFromLineSegment(SurgSim::Math::Vector3d *closestPointToOrigin);

    /// Find the closest point to the origin from the simplex, which is a triangle.
    /// \param [out] closestPointToOrigin The closest point on the triangle to the origin.
    void closestPointFromTriangle(SurgSim::Math::Vector3d *closestPointToOrigin);

    /// Find the closest point to the origin from the simplex, which is a tetrahedron.
    /// \param [out] closestPointToOrigin The closest point on the tetrahedron to the origin.
    /// \note The closest point from tetrahedron is assumed to be in the half space
    /// defined by the plane (a,b,c) containing the vertex d.
    void closestPointFromTetrahedron(SurgSim::Math::Vector3d *closestPointToOrigin);

private:
    /// The vertex in Minkowski(A,-B), which is also the vector from B to A.
    std::vector<SurgSim::Math::Vector3d> m_vertexInMinkowskiSpace;
    /// The vertex in shape A.
    std::vector<SurgSim::Math::Vector3d> m_vertexInA;
    /// The vertex in shape B.
    std::vector<SurgSim::Math::Vector3d> m_vertexInB;
    /// The barycentric coordinates of the closest vertex to the origin within the simplex.
    std::vector<double> m_vertexBaryCenterInMinkowskiSpace;
    /// The number of vertices being actively used.
    size_t m_numberOfActiveVertices;

};

}; // namespace Collision
}; // namespace SurgSim

#endif