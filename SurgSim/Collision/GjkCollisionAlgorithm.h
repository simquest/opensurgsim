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

#ifndef SURGSIM_COLLISION_GJKCOLLISIONALGORITHM_H
#define SURGSIM_COLLISION_GJKCOLLISIONALGORITHM_H

#include <memory>
#include <vector>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/Shape.h>
#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Collision/GjkSimplex.h>

using SurgSim::Math::Shape;

namespace SurgSim
{
namespace Collision
{

/// Class to perform the GJK algorithm between two convex meshes.
class GjkCollisionAlgorithm
{
public:
    /// Constructor
    /// \param maximumIterations The iterations count limit for the algorithm.
    explicit GjkCollisionAlgorithm(unsigned int maximumIterations = 1000)
                          : m_maximumIterations(maximumIterations)
    {
        m_separatingAxis.setZero();
        m_vertexInA.setZero();
        m_vertexInB.setZero();
    }

    /// Destructor
    virtual ~GjkCollisionAlgorithm()
    {
    }

    /// Get the separating axis.
    /// \return The separating axis.
    SurgSim::Math::Vector3d getSeparatingAxis() const;

    /// Get the vertex in shape A.
    /// \return The vertex in shape A.
    SurgSim::Math::Vector3d getVertexInA() const;

    /// Get the vertex in shape B.
    /// \return The vertex in shape B.
    SurgSim::Math::Vector3d getVertexInB() const;

    /// Get the maximum iterations.
    unsigned int getMaximumIterations() const;

    /// Set the maximum iterations.
    /// \param maximumIterations The maximumIterations value to be set.
    void setMaximumIterations(unsigned int maximumIterations);

    /// Detect collision.
    /// \param pair The collision pair which contains the two shapes, between which the algorithm is run.
    /// \return True, if a collision is detected. False, otherwise.
    bool detectCollision(std::shared_ptr<SurgSim::Collision::CollisionPair> pair);

private:
    /// The separating axis.
    SurgSim::Math::Vector3d m_separatingAxis;
    /// The vertex in shape A.
    SurgSim::Math::Vector3d m_vertexInA;
    /// The vertex in shape B.
    SurgSim::Math::Vector3d m_vertexInB;
    /// Maximum number of iterations for for the GJK algorithm.
    unsigned int m_maximumIterations;

};

}; // namespace Collision
}; // namespace SurgSim

#endif