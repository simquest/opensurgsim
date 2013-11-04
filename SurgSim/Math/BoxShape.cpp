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

#include <SurgSim/Math/BoxShape.h>

namespace SurgSim
{
namespace Math
{

BoxShape::BoxShape(double sizeX, double sizeY, double sizeZ)
{
	m_size[0] = sizeX;
	m_size[1] = sizeY;
	m_size[2] = sizeZ;

	calculateVertices();
}


int BoxShape::getType()
{
	return SHAPE_TYPE_BOX;
}

double BoxShape::getSizeX() const
{
	return m_size[0];
}

double BoxShape::getSizeY() const
{
	return m_size[1];
}

double BoxShape::getSizeZ() const
{
	return m_size[2];
}

double BoxShape::calculateVolume() const
{
	return m_size[0] * m_size[1] * m_size[2];
}

SurgSim::Math::Vector3d BoxShape::calculateMassCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d BoxShape::calculateInertia(double rho) const
{
	const double mass = calculateMass(rho);

	const double SquarelengthX = m_size[0] * m_size[0];
	const double SquarelengthY = m_size[1] * m_size[1];
	const double SquarelengthZ = m_size[2] * m_size[2];
	const double coef = 1.0 / 12.0 * mass;

	Matrix33d inertia;
	inertia.setZero();
	inertia(0, 0) = coef * (SquarelengthY + SquarelengthZ);
	inertia(1, 1) = coef * (SquarelengthX + SquarelengthZ);
	inertia(2, 2) = coef * (SquarelengthX + SquarelengthY);

	return inertia;
}

SurgSim::Math::Vector3d BoxShape::calculateGlobalVertex(const int i,
										 const SurgSim::Math::Quaterniond& quat,
										 const Vector3d& trans) const
{
	return quat * m_vertices[i] + trans;
}

SurgSim::Math::Vector3d BoxShape::getLocalVertex(const int i) const
{
	return m_vertices[i];
}

void BoxShape::calculateVertices()
{
	static const double multiplier[8][3] = {{-0.5, -0.5, -0.5}, {-0.5, -0.5, 0.5}, {-0.5, 0.5, 0.5},
		{-0.5, 0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {0.5, 0.5, -0.5}
	};

	for(int i = 0; i < 8; ++i)
	{
		m_vertices[i][0] = m_size[0] * multiplier[i][0];
		m_vertices[i][1] = m_size[1] * multiplier[i][1];
		m_vertices[i][2] = m_size[2] * multiplier[i][2];
	}
}

}; // namespace Math
}; // namespace SurgSim
