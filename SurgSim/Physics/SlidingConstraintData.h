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

#ifndef SURGSIM_PHYSICS_SLIDINGCONSTRAINTDATA_H
#define SURGSIM_PHYSICS_SLIDINGCONSTRAINTDATA_H

#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{

namespace Physics
{

/// Class for Frictionless sliding constraint (stores two planes, the intersection of which, is the sliding direction)
class SlidingConstraintData : public ConstraintData
{
public:
	/// Default constructor
	SlidingConstraintData();

	/// Constructor
	/// \param point The point through which the sliding direction vector passes.
	/// \param direction The sliding direction vector.
	SlidingConstraintData(const SurgSim::Math::Vector3d& point, const SurgSim::Math::Vector3d& direction);

	/// Destructor
	virtual ~SlidingConstraintData();

	/// Calculate the two plane equations based on the given sliding direction.
	/// \param point The point through which the sliding direction vector passes.
	/// \param direction The sliding direction vector.
	void setSlidingDirection(const SurgSim::Math::Vector3d& point, const SurgSim::Math::Vector3d& direction);

	/// Set the friction coefficient for a frictional sliding constraint
	/// \param mu The friction coefficient
	void setFrictionCoefficient(double mu);

	/// \return The friction coefficient (default is 0.5)
	/// \note The friction coefficient is only used for frictional constraint, it is discarded othersise.
	double getFrictionCoefficient() const;

	/// \return The normals of the two planes.
	const std::array<Math::Vector3d, 2>& getNormals() const;

	/// /return The pose of constraint
	const Math::RigidTransform3d getPose();

	/// \return The tangent (direction defined by the two planes).
	const Math::Vector3d& getTangent() const;

	/// \return The distance from origin of the planes direction (tangent).
	const double getDistanceTangent() const;

private:
	/// The normals of the two planes.
	std::array<Math::Vector3d, 2> m_normals;

	/// The point of constraint.
	Math::Vector3d m_point;

	/// The sliding direction (intersection of the two planes).
	Math::Vector3d m_slidingDirection;

	/// The tangent of the two planes.
	Math::Vector3d m_tangent;

	/// The distance from origin of the tangential plane.
	double m_distanceTangent;

	/// Friction coefficient for frictional constraint (unused for frictionless constraint).
	double m_mu;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_SLIDINGCONSTRAINTDATA_H
