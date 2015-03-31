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

#ifndef SURGSIM_PHYSICS_CONSTRAINTDATA3DDISTANCEPOINTS_H
#define SURGSIM_PHYSICS_CONSTRAINTDATA3DDISTANCEPOINTS_H

#include <array>

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"

namespace SurgSim
{

namespace Physics
{

/// CosntraintData for rotation vector constraint
class ConstraintDataFem3DDistancePoints : public ConstraintData
{
public:
	/// Default Constructor
	ConstraintDataFem3DDistancePoints() : ConstraintData() {}

	/// Destructor
	virtual ~ConstraintDataFem3DDistancePoints(){}

	void setDistance(double d)
	{
		m_distance = d;
	}

	double getDistance() const
	{
		return m_distance;
	}

	double getSquaredDistance() const
	{
		return m_distance * m_distance;
	}

	void setPoint(size_t id, std::shared_ptr<Fem3DRepresentationLocalization> pt)
	{
		SURGSIM_ASSERT(id < 2) << "invalid id (should be 0 or 1)";
		m_points[id] = pt;
	}

	std::shared_ptr<Fem3DRepresentationLocalization> getPoint(size_t id) const
	{
		SURGSIM_ASSERT(id < 2) << "invalid id (should be 0 or 1)";
		return m_points[id];
	}

private:
	std::array<std::shared_ptr<Fem3DRepresentationLocalization>, 2> m_points;
	double m_distance;
};

};  // namespace Physics
};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONSTRAINTDATA3DDISTANCEPOINTS_H
