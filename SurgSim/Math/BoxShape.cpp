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

#include "SurgSim/Math/BoxShape.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::BoxShape, BoxShape);

BoxShape::BoxShape(double sizeX, double sizeY, double sizeZ) :
	m_size(Vector3d(sizeX, sizeY, sizeZ))
{
	calculateVertices();
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoxShape, double, SizeX, getSizeX, setSizeX);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoxShape, double, SizeY, getSizeY, setSizeY);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(BoxShape, double, SizeZ, getSizeZ, setSizeZ);
}


int BoxShape::getType() const
{
	return SHAPE_TYPE_BOX;
}

Vector3d BoxShape::getSize() const
{
	return m_size;
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

void BoxShape::setSizeX(double sizeX)
{
	m_size[0] = sizeX;
}

void BoxShape::setSizeY(double sizeY)
{
	m_size[1] = sizeY;
}

void BoxShape::setSizeZ(double sizeZ)
{
	m_size[2] = sizeZ;
}

double BoxShape::getVolume() const
{
	return m_size[0] * m_size[1] * m_size[2];
}

SurgSim::Math::Vector3d BoxShape::getCenter() const
{
	return Vector3d(0.0, 0.0, 0.0);
}

SurgSim::Math::Matrix33d BoxShape::getSecondMomentOfVolume() const
{
	const double volume = getVolume();

	const Vector3d sizeSquared = m_size.array() * m_size.array();
	const double coef = 1.0 / 12.0 * volume;
	Matrix33d inertia = Matrix33d::Zero();
	inertia.diagonal() = coef * Vector3d(sizeSquared[1] + sizeSquared[2],
										sizeSquared[0] + sizeSquared[2],
										sizeSquared[0] + sizeSquared[1]);
	return inertia;
}

SurgSim::Math::Vector3d BoxShape::getVertex(const int i) const
{
	return m_vertices[i];
}

const std::array<Vector3d, 8>& BoxShape::getVertices() const
{
	return m_vertices;
}

void BoxShape::calculateVertices()
{
	static const std::array<Vector3d, 8> multiplier = {{Vector3d(-0.5, -0.5, -0.5),
														Vector3d(-0.5, -0.5,  0.5),
														Vector3d(-0.5,  0.5,  0.5),
														Vector3d(-0.5,  0.5, -0.5),
														Vector3d( 0.5, -0.5, -0.5),
														Vector3d( 0.5, -0.5,  0.5),
														Vector3d( 0.5,  0.5,  0.5),
														Vector3d( 0.5,  0.5, -0.5)}};
	for(int i = 0; i < 8; ++i)
	{
		m_vertices[i] = m_size.array() * multiplier[i].array();
	}
}

bool BoxShape::isValid() const
{
	return (m_size.minCoeff() >= 0);
}

}; // namespace Math
}; // namespace SurgSim
