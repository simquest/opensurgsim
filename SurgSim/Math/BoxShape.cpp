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

BoxShape::BoxShape(double sizeX, double sizeY, double sizeZ) :
	m_size(Vector3d(sizeX, sizeY, sizeZ))
{
	calculateVertices();
}


int BoxShape::getType()
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

	const Vector3d sizeSquared = m_size.array() * m_size.array();
	const double coef = 1.0 / 12.0 * mass;
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
	static const std::array<Vector3d, 8> multiplier = {Vector3d(-0.5, -0.5, -0.5),
													   Vector3d(-0.5, -0.5,  0.5),
													   Vector3d(-0.5,  0.5,  0.5),
													   Vector3d(-0.5,  0.5, -0.5),
													   Vector3d( 0.5, -0.5, -0.5),
													   Vector3d( 0.5, -0.5,  0.5),
													   Vector3d( 0.5,  0.5,  0.5),
													   Vector3d( 0.5,  0.5, -0.5)};
	for(int i = 0; i < 8; ++i)
	{
		m_vertices[i] = m_size.array() * multiplier[i].array();
	}
}

YAML::Node SurgSim::Math::BoxShape::encode()
{
	YAML::Node node;
	node = SurgSim::Math::Shape::encode();
	node["SizeX"] = getSizeX();
	node["SizeY"] = getSizeY();
	node["SizeZ"] = getSizeZ();

	return node;
}

bool SurgSim::Math::BoxShape::decode(const YAML::Node& node)
{
	bool isSuccess = SurgSim::Math::Shape::decode(node);
	if (! isSuccess)
	{
		return false;
	}

	m_size[0] = node["SizeX"].as<double>();
	m_size[1] = node["SizeY"].as<double>();
	m_size[2] = node["SizeZ"].as<double>();

	return true;
}


}; // namespace Math
}; // namespace SurgSim
