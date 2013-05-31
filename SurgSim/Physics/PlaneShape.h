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

#ifndef SURGSIM_PHYSICS_PLANESHAPE_H
#define SURGSIM_PHYSICS_PLANESHAPE_H

#include <SurgSim/Physics/RigidShape.h>

namespace SurgSim
{

namespace Physics
{

/// PlaneShape a plane with the form nx + d = 0
class PlaneShape: public RigidShape
{
public:
	/// Constructor
	/// \param sizeX, sizeY, sizeZ the box sizes in all 3 directions (in m)
	PlaneShape(Vector3d normal, double d) :
		m_normal(normal.normalized()), m_d(d) 
	{
		
	}

	PlaneShape(Vector3d point0, Vector3d normal) : m_normal(normal.normalized())
	{
		m_d = - m_normal.dot(point0);
	}

	PlaneShape(Vector3d point0, Vector3d point1, Vector3d point2)
	{
		m_normal = (point1 - point0).cross(point2 - point0);
		m_normal.normalize();
		m_d = - m_normal.dot(point0);
	}

	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_PLANE;
	}

	/// Calculate the volume of the box
	/// \return The volume of the box (in m-3)
	double calculateVolume() const
	{
		return 0;
	}

	/// Calculate the mass center of the box
	/// \return The mass center of the box
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the box
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the box
	Matrix33d calculateInertia(double rho) const
	{
		return Matrix33d::Identity();
	}

	/// Gets the d of the plane equation.
	/// \return	The value of d.
	inline double getD()
	{
		return m_d;
	}

	/// Gets the normal of the plane equation.
	/// \return	The value of the normal.
	const Vector3d& getNormal()
	{
		return m_normal;
	}

private:

	/// The normal of the plane
	Vector3d m_normal;

	/// The d of the plane 
	double m_d;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_PLANESHAPE_H
