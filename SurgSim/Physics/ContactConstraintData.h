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

#ifndef SURGSIM_PHYSICS_CONTACTCONSTRAINTDATA_H
#define SURGSIM_PHYSICS_CONTACTCONSTRAINTDATA_H

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

/// Class for Frictionless contact (only needs a plane equation)
class ContactConstraintData : public ConstraintData
{
public:
	/// Default constructor
	ContactConstraintData() :
		ConstraintData(),
		m_distance(0.0)
	{
		m_normal.setZero();
	}

	/// Destructor
	virtual ~ContactConstraintData()
	{
	}

	/// Sets the plane equation of the frictionless contact
	/// \param n The plane normal (normalized vector)
	/// \param d The plane distance to the origin
	/// \note The plane is defined by { P | n.P + d = 0 }
	void setPlaneEquation(const SurgSim::Math::Vector3d& n, double d)
	{
		m_normal = n;
		m_distance = d;
	}

	/// Gets the plane normal vector
	/// \return The plane equation's normal vector (normalized vector)
	const SurgSim::Math::Vector3d& getNormal() const
	{
		return m_normal;
	}

	/// Gets the plane distance to the origin
	/// \return The plane equation's distance to the origin
	double getDistance() const
	{
		return m_distance;
	}

	/// Gets the time of contact for the collision
	/// \return The time of contact for the collision
	double getContactTime() const
	{
		return m_contact->time;
	}

	/// \return The contact that uses this constraint data.
	std::shared_ptr<Collision::Contact> getContact()
	{
		return m_contact;
	}

	/// \param contacts The contacts that use this constraint data.
	void setContact(const std::shared_ptr<Collision::Contact>& contacts)
	{
		m_contact = contacts;
	}

private:
	/// Plane equation normal vector (normalized vector)
	SurgSim::Math::Vector3d m_normal;

	/// Plane equation distance to origin
	double m_distance;

	/// The contact that uses this constraint data.
	std::shared_ptr<Collision::Contact> m_contact;
};

};  // namespace Physics

};  // namespace SurgSim

#endif  // SURGSIM_PHYSICS_CONTACTCONSTRAINTDATA_H
