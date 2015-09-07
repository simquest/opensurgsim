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

#ifndef SURGSIM_MATH_UNITTESTS_MOCKTRIANGLE_H
#define SURGSIM_MATH_UNITTESTS_MOCKTRIANGLE_H

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim
{
namespace Math
{

/// MockTriangle class used in the unit tests.
class MockTriangle
{
public:
	// Default constructor.
	MockTriangle() {}

	// Constructor.
	MockTriangle(const Vector3d& vertex0, const Vector3d& vertex1, const Vector3d& vertex2) :
		v0(vertex0), v1(vertex1), v2(vertex2), v0v1(vertex1 - vertex0), v0v2(vertex2 - vertex0),
		v1v2(vertex2 - vertex1)
	{
		n = v0v1.cross(v0v2);
		n.normalize();
	}

	// Find a point inside the triangle, given a pair of scaling factor for the edges (v0v1 and v0v2).
	Vector3d pointInTriangle(double a, double b) const
	{
		return v0 + a * v0v1 + b * v0v2;
	}

	// Move this triangle by the given vector.
	void translate(const Vector3d& v)
	{
		v0 += v;
		v1 += v;
		v2 += v;
	}

	// Rotate this triangle about the x-axis by the given angle.
	void rotateAboutXBy(double angle)
	{
		RigidTransform3d r(Eigen::AngleAxis<double>(angle * (M_PI / 180.0), Vector3d(1, 0, 0)));
		v0  = r * v0;
		v1  = r * v1;
		v2  = r * v2;
		n = r * n;
	}

	// Rotate this triangle about the y-axis by the given angle.
	void rotateAboutYBy(double angle)
	{
		SurgSim::Math::RigidTransform3d r(Eigen::AngleAxis<double>(angle * (M_PI / 180.0), Vector3d(0, 1, 0)));
		v0  = r * v0;
		v1  = r * v1;
		v2  = r * v2;
		n = r * n;
	}

	// Rotate this triangle about the z-axis by the given angle.
	void rotateAboutZBy(double angle)
	{
		SurgSim::Math::RigidTransform3d r(Eigen::AngleAxis<double>(angle * (M_PI / 180.0), Vector3d(0, 0, 1)));
		v0  = r * v0;
		v1  = r * v1;
		v2  = r * v2;
		n = r * n;
	}

	// Transform this triangle by the given matrix.
	void transform(SurgSim::Math::RigidTransform3d transform)
	{
		v0  = transform * v0;
		v1  = transform * v1;
		v2  = transform * v2;
		n = transform.linear() * n;
	}

	// Vertices of this triangle.
	Vector3d v0;
	Vector3d v1;
	Vector3d v2;

	// Edges of this triangle.
	Vector3d v0v1;
	Vector3d v0v2;
	Vector3d v1v2;

	// Normal of the triangle.
	Vector3d n;
};


} // namespace Math

} // namespace SurgSim

#endif // SURGSIM_MATH_UNITTESTS_MOCKTRIANGLE_H