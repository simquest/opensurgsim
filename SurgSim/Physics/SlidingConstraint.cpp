// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#include <sstream>

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/FemRepresentation.h"
#include "SurgSim/Physics/SlidingConstraint.h"
#include "SurgSim/Physics/SlidingConstraintData.h"

namespace SurgSim
{

namespace Physics
{

SlidingConstraint::SlidingConstraint(ConstraintType constraintType,
	std::shared_ptr<ConstraintData> data,
	std::shared_ptr<Representation> representation0,
	const DataStructures::Location& location0,
	std::shared_ptr<Representation> representation1,
	const DataStructures::Location& location1,
	const Math::Vector3d& slidingDirection)
	: Constraint(constraintType, data, representation0, location0, representation1, location1)
{
	m_slidingConstraintData = std::dynamic_pointer_cast<SlidingConstraintData>(data);
	SURGSIM_ASSERT(m_slidingConstraintData != nullptr) <<
		"The data sent in for the sliding constraint is not of type SlidingConstraintData.";

	Math::Vector3d directionEnd = m_localizations.second->calculatePosition() + slidingDirection;
	m_directionEnd = m_localizations.second->getTransform().inverse() * directionEnd;
}

SlidingConstraint::~SlidingConstraint()
{
}

void SlidingConstraint::doBuild(double dt,
	const ConstraintData& data,
	MlcpPhysicsProblem* mlcp,
	size_t indexOfRepresentation0,
	size_t indexOfRepresentation1,
	size_t indexOfConstraint)
{
	// Update the SlidingConstraintData
	Math::Vector3d pointOfConstraint = m_localizations.second->calculatePosition();
	Math::Vector3d slidingDirection = m_localizations.second->getTransform() * m_directionEnd - pointOfConstraint;
	m_slidingConstraintData->setSlidingDirection(pointOfConstraint, slidingDirection);

	// Update the representation0's localization (representation1's remains the same).
	bool hasReachedEnd = false;
	m_localizations.first->moveClosestTo(pointOfConstraint, &hasReachedEnd);

	if (hasReachedEnd)
	{
		setActive(false);
	}
}

} // namespace Physics

} // namespace SurgSim
