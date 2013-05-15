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

#ifndef SURGSIM_PHYSICS_RIGIDSHAPE_H
#define SURGSIM_PHYSICS_RIGIDSHAPE_H

#include <SurgSim/Framework/Assert.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using namespace SurgSim::Framework;

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

namespace SurgSim 
{

namespace Physics
{

/// Generic rigid shape class defining a shape for a rigid actor
/// \note This class gives the ability to analyse the shape and compute
/// \note physical information (volume, mass, mass center, inertia)
class RigidShape
{
public:
	/// Calculate the volume, mass, mass center and inertia from the shape
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 symmetric inertia matrix of the shape
	virtual void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const = 0;
};

/// Mesh shape
class Mesh;
class MeshShape: public RigidShape
{
public:
	/// Constructor
	/// \param mesh The mesh to build the shape from
	explicit MeshShape(const std::shared_ptr<Mesh> mesh)
	{
		m_mesh = mesh;
	};

	/// Get mesh
	/// \return The mesh associated to this MeshShape
	const std::shared_ptr<Mesh> getMesh() const
	{
		return m_mesh;
	};

	/// Calculate the volume, mass, mass center and inertia from the mesh
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 inertia matrix of the rigid shape
	void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const
	{
	};

private:
	/// Mesh associated to this MeshShape
	std::shared_ptr<Mesh> m_mesh;
};

/// Sphere shape: sphere centered on (0 0 0), with radius, solid or not
class SphereShape: public RigidShape
{
public:
	/// Constructor
	/// \param radius The sphere radius (in m)
	/// \param isSolid True if the sphere is solid, False otherwise
	SphereShape(double radius, bool isSolid = true)
	{
		m_radius = radius;
		m_isSolid  = isSolid;
	};

	/// Get the sphere radius
	/// \return The sphere radius
	double getRadius() const
	{
		return m_radius;
	};

	/// Query if the sphere is solid or not
	/// \return True if the sphere is solid, False otherwise
	bool isSolid() const
	{
		return m_isSolid;
	};

	/// Calculate the volume, mass, mass center and inertia from the sphere
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 inertia matrix of the rigid shape
	void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const
	{
		double r2 = m_radius * m_radius;

		double localVolume;
		if (m_isSolid)
		{
			localVolume = 4.0 / 3.0 * M_PI * m_radius * r2;
		}
		else
		{
			localVolume = 4.0 * M_PI * r2;
		}
		if (volume)
		{
			*volume = localVolume;
		}

		double localMass = localVolume * rho;
		if (mass)
		{
			*mass = localMass;
		}

		if (massCenter)
		{
			massCenter->setZero();
		}
		
		if (inertia)
		{
			inertia->setIdentity();
			(*inertia) *= (m_isSolid ? 2.0 / 5.0 : 2.0 / 3.0) * localMass * r2;
		}
	};

private:
	/// Sphere radius
	double m_radius;

	/// Is the sphere solid or hollow ?
	bool m_isSolid;
};

/// Box shape: box centered on (0 0 0), aligned with the axis
/// with different sizes along X, Y and Z
class BoxShape: public RigidShape
{
public:
	/// Constructor
	/// \param sizeX, sizeY, sizeZ the box sizes in all 3 directions (in m)
	BoxShape(double sizeX, double sizeY, double sizeZ)
	{
		m_size[0] = sizeX;
		m_size[1] = sizeY;
		m_size[2] = sizeZ;
	};

	/// Get size in X direction
	/// \return the size in the X direction (in m)
	double getSizeX() const
	{
		return m_size[0];
	};

	/// Get size in Y direction
	/// \return the size in the Y direction (in m)
	double getSizeY() const
	{
		return m_size[1];
	};

	/// Get size in Z direction
	/// \return the size in the Z direction (in m)
	double getSizeZ() const
	{
		return m_size[2];
	};

	/// Calculate the volume, mass, mass center and inertia from the box
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 inertia matrix of the rigid shape
	void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const
	{
		double localVolume = m_size[0] * m_size[1] * m_size[2];
		if (volume)
		{
			*volume = localVolume;
		}

		double localMass = localVolume * rho;
		if (mass)
		{
			*mass = localMass;
		}

		if (massCenter)
		{
			massCenter->setZero();
		}
		
		if (inertia)
		{
			const double SquarelengthX = m_size[0] * m_size[0];
			const double SquarelengthY = m_size[1] * m_size[1];
			const double SquarelengthZ = m_size[2] * m_size[2];
			const double coef = 1.0 / 12.0 * localMass;

			inertia->setZero();
			(*inertia)(0, 0) = coef * (SquarelengthY + SquarelengthZ);
			(*inertia)(1, 1) = coef * (SquarelengthX + SquarelengthZ);
			(*inertia)(2, 2) = coef * (SquarelengthX + SquarelengthY);
		}
	};

private:
	/// The box sizes along the 3 axis respectively {X,Y,Z}
	double   m_size[3];
};

/// Cylinder shape: centered on (0 0 0) with inner and outer radius
/// Cannot have inner radius = outer radius
/// /tParam direction MUST BE {0,1,2} for respectively {X,Y,Z} alignment
template <unsigned int direction>
class CylinderShape: public RigidShape
{
public:
	/// Constructor
	/// \param length The full length of the cylinder (in m)
	/// \param outerRadius, innerRadius The outer and inner radii (in m)
	/// \note outerRadius MUST BE DIFFERENT THAN inerRadius
	CylinderShape(double length, double outerRadius, double innerRadius = 0.0)
	{
		m_length      = length;
		m_outerRadius = outerRadius;
		m_innerRadius = innerRadius;
		SURGSIM_ASSERT(outerRadius != innerRadius);
	};

	/// Get the cylinder length
	/// \return The cylinder length (in m)
	double getLength() const
	{
		return m_length;
	};

	/// Get the cylinder outer radius
	/// \return The cylinder outer radius (in m)
	double getOuterRadius() const
	{
		return m_outerRadius;
	};

	/// Get the cylinder inner radius
	/// \return The cylinder inner radius (in m)
	double getInnerRadius() const
	{
		return m_innerRadius;
	};

	/// Calculate the volume, mass, mass center and inertia from the cylinder
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 inertia matrix of the rigid shape
	void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const
	{
		const double &L = m_length;
		const double squareORadius = m_outerRadius * m_outerRadius;
		const double squareIRadius = m_innerRadius * m_innerRadius;
		const double localVolume = M_PI * (squareORadius - squareIRadius) * L;

		if (volume)
		{
			*volume = localVolume;
		}

		double localMass = localVolume * rho;
		if (mass)
		{
			*mass = localMass;
		}

		if (massCenter)
		{
			massCenter->setZero();
		}

		if (inertia)
		{
			const double coef    = 1.0 / 12.0 * localMass;
			const double coefDir = 1.0 /  2.0 * localMass;
			const double squareL = m_length * m_length;
			const double squareRadii = squareORadius + squareIRadius;

			inertia->setIdentity();
			(*inertia) *= coef * (3.0 * squareRadii + squareL);
			(*inertia)(direction, direction) = coefDir * (squareRadii);
		}
	};

private:
	/// The cylinder outer radius
	double   m_outerRadius;

	/// The cylinder inner radius
	double   m_innerRadius;
	
	/// The cylinder length
	double   m_length;
};

/// Capsule shape: centered on (0 0 0), with length and radius
/// \tParam direction MUST BE IN {0,1,2} for respectively {X,Y,Z} alignment
template <unsigned int direction>
class CapsuleShape: public RigidShape
{
public:
	/// Constructor
	/// \param length The capsule length (i.e. of the cylinder) (in m)
	/// \param radius The capsule radius (i.e. of the cylinder/spheres) (in m)
	CapsuleShape(double length, double radius)
	{
		m_length = length;
		m_radius = radius;
	};

	/// Get the capsule length (i.e. cylinder length)
	/// \return The capsule length (in m)
	double getLength() const
	{
		return m_length;
	};

	/// Get the capsule radius (i.e. cylinder/spheres radius)
	/// \return The capsule radius (in m)
	double getRadius() const
	{
		return m_radius;
	};

	/// Calculate the volume, mass, mass center and inertia from the capsule
	/// \param rho The mass density (in Kg.m-3)
	/// \param [out] volume The volume of the rigid shape (in m-3)
	/// \param [out] mass The mass of the rigid shape (in Kg)
	/// \param [out] massCenter The mass center of the rigid shape
	/// \param [out] inertia The 3x3 inertia matrix of the rigid shape
	void calculateMassInertia(double rho, double *volume, double* mass,
		Vector3d* massCenter, Matrix33d* inertia) const
	{
		const double &r = m_radius;
		const double &l = m_length;
		const double r2 = r * r;
		const double l2 = l * l;
		const double localCylinderVolume = M_PI * (r2) * l;
		const double localSphereVolume   = 4.0 / 3.0 * M_PI * r2 * r;

		if (volume)
		{
			*volume = localCylinderVolume + localSphereVolume;
		}

		double localCylinderMass = localCylinderVolume * rho;
		double localSphereMass   = localSphereVolume * rho;
		double localMass         = localCylinderMass + localSphereMass;
		if (mass)
		{
			*mass = localMass;
		}

		if (massCenter)
		{
			massCenter->setZero();
		}

		if (inertia)
		{
			// The Inertia matrix is a combination of the cylinder inertia and
			// the 2 semi-spheres inertia:
			//
			// Inertia of cylinder along the X axis (direction = 0)
			// mc = PI.radius.radius.length (mass of the cylinder)
			// a = 1/2.mc.r^2
			// b = 1/12.mc.(3.r^2 + h^2)
			//               (a 0 0)
			// I(cylinder) = (0 b 0)
			//               (0 0 b)
			//
			// Inertia of the 2 semi-spheres along the X axis (direction = 0)
			// ms = 4/3 pi.radius.radius.radius (mass of the entire sphere)
			// c = 2/5.ms.r^2
			// d = 2/5.ms.r^2 + ms.h^2/4 + 3/8.ms.r.h
			//                     (c 0 0)
			// I(2 semi-spheres) = (0 d 0)
			//                     (0 0 d)

			double a = 1.0 / 2.0  * localCylinderMass * r2;
			double b = 1.0 / 12.0 * localCylinderMass * (3.0 * r2 + l2);
			double c = 2.0 / 5.0  * localSphereMass   * r2;
			double d = c + localSphereMass * l * (l / 4.0 + 3.0 / 8.0 * r);

			inertia->setIdentity();
			(*inertia) *= b + d;
			(*inertia)(direction, direction) = a + c;
		}
	};

private:
	/// Capsule radius
	double   m_radius;

	/// Capsule length
	double   m_length;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_RIGIDSHAPE_H
