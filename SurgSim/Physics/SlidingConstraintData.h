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

namespace SurgSim
{

namespace Physics
{

/// Class for Frictionless sliding constraint (stores two planes, the intersection of which, is the sliding direction)
class SlidingConstraintData : public ConstraintData
{
public:
	/// Default constructor
	SlidingConstraintData() :
		ConstraintData(), m_d1(0.0), m_d2(0.0)
	{
		m_normal1.setZero();
		m_normal2.setZero();
	}

	SlidingConstraintData(const Math::Vector3d& normal1, double d1, const Math::Vector3d& normal2, double d2) :
		ConstraintData(), m_normal1(normal1), m_d1(d1), m_normal2(normal2), m_d2(d2)
	{}

	/// Destructor
	virtual ~SlidingConstraintData()
	{
	}

	/// Sets the plane equation for the first plane
	/// \param n The plane normal (normalized vector)
	/// \param d The plane distance to the origin
	/// \note The plane is defined by { P | n.P + d = 0 }
	void setPlane1Equation(const SurgSim::Math::Vector3d& n, double d)
	{
		m_normal1 = n;
		m_d1 = d;
	}

	/// \return Normal of plane 1.
	const SurgSim::Math::Vector3d& getNormal1() const
	{
		return m_normal1;
	}

	/// \return d of plane 1.
	double getD1() const
	{
		return m_d1;
	}

	/// Sets the plane equation for the second plane
	/// \param n The plane normal (normalized vector)
	/// \param d The plane distance to the origin
	/// \note The plane is defined by { P | n.P + d = 0 }
	void setPlane2Equation(const SurgSim::Math::Vector3d& n, double d)
	{
		m_normal2 = n;
		m_d2 = d;
	}

	/// \return Normal of plane 2.
	const SurgSim::Math::Vector3d& getNormal2() const
	{
		return m_normal2;
	}

	/// \return d of plane 2.
	double getD2() const
	{
		return m_d2;
	}

private:
	/// Normal of plane 1.
	Math::Vector3d m_normal1;

	/// d of plane 1.
	double m_d1;

	/// Normal of plane 1.
	Math::Vector3d m_normal2;

	/// d of plane 1.
	double m_d2;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_SLIDINGCONSTRAINTDATA_H
