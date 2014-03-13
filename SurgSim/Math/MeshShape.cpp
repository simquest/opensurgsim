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
//

#include "SurgSim/Math/MeshShape.h"

namespace SurgSim
{
namespace Math
{

int MeshShape::getType()
{
	return SHAPE_TYPE_MESH;
}

const std::shared_ptr<SurgSim::DataStructures::TriangleMesh> MeshShape::getMesh() const
{
	return m_mesh;
}

double MeshShape::getVolume() const
{
	return m_volume;
}

SurgSim::Math::Vector3d MeshShape::getCenter() const
{
	return m_center;
}

SurgSim::Math::Matrix33d MeshShape::getSecondMomentOfVolume() const
{
	return m_secondMomentOfVolume;
}

void MeshShape::computeIntegralTerms(const double w0, const double w1, const double w2,
									 double* f1, double* f2, double* f3, double* g0, double* g1, double* g2) const
{
	double temp0 = w0 + w1;
	double temp1 = w0 * w0;
	double temp2 = temp1 + w1 * temp0;
	*f1 = temp0 + w2;
	*f2 = temp2 + w2 * (*f1);
	*f3 = w0 * temp1 + w1 * temp2 + w2 * (*f2);
	*g0 = (*f2) + w0 * ((*f1) + w0);
	*g1 = (*f2) + w1 * ((*f1) + w1);
	*g2 = (*f2) + w2 * ((*f1) + w2);
}

void MeshShape::computeVolumeIntegrals()
{
	const double mult[10] = {1.0 / 6.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 60.0, 1.0 / 60.0, 1.0 / 60.0,
		1.0 / 120.0, 1.0 / 120.0, 1.0 / 120.0};
	// intg order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
	double intg[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	for (size_t t = 0; t < m_mesh->getNumTriangles(); t++)
	{
		// get vertices of triangle t
		const Vector3d& p0 = m_mesh->getVertexPosition(m_mesh->getTriangle(t).verticesId[0]);
		const Vector3d& p1 = m_mesh->getVertexPosition(m_mesh->getTriangle(t).verticesId[1]);
		const Vector3d& p2 = m_mesh->getVertexPosition(m_mesh->getTriangle(t).verticesId[2]);

		double x0 = p0.x(), y0 = p0.y(), z0 = p0.z();
		double x1 = p1.x(), y1 = p1.y(), z1 = p1.z();
		double x2 = p2.x(), y2 = p2.y(), z2 = p2.z();

		// get edges and cross product of edges
		double a1 = x1 - x0, b1 = y1 - y0, c1 = z1 - z0;
		double a2 = x2 - x0, b2 = y2 - y0, c2 = z2 - z0;
		double d0 = b1 * c2 - b2 * c1;
		double d1 = a2 * c1 - a1 * c2;
		double d2 = a1 * b2 - a2 * b1;

		// compute integral terms
		double f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z;
		double g0x, g0y, g0z, g1x, g1y, g1z, g2x, g2y, g2z;
		computeIntegralTerms(x0, x1, x2, &f1x, &f2x, &f3x, &g0x, &g1x, &g2x);
		computeIntegralTerms(y0, y1, y2, &f1y, &f2y, &f3y, &g0y, &g1y, &g2y);
		computeIntegralTerms(z0, z1, z2, &f1z, &f2z, &f3z, &g0z, &g1z, &g2z);

		// update integrals
		intg[0] += d0 * f1x;
		intg[1] += d0 * f2x;
		intg[2] += d1 * f2y;
		intg[3] += d2 * f2z;
		intg[4] += d0 * f3x;
		intg[5] += d1 * f3y;
		intg[6] += d2 * f3z;
		intg[7] += d0 * (y0 * g0x + y1 * g1x + y2 * g2x);
		intg[8] += d1 * (z0 * g0y + z1 * g1y + z2 * g2y);
		intg[9] += d2 * (x0 * g0z + x1 * g1z + x2 * g2z);
	}
	for (int i = 0; i < 10; i++)
	{
		intg[i] *= mult[i];
	}

	// volume
	m_volume = intg[0];
	// Test the volume validity
	SURGSIM_ASSERT(m_volume > 0.0) << "A MeshShape cannot have a negative or null volume, we found V = " << m_volume;

	// center of mass
	m_center.x() = intg[1] / m_volume;
	m_center.y() = intg[2] / m_volume;
	m_center.z() = intg[3] / m_volume;

	// inertia tensor relative to center of mass
	const Vector3d& C = m_center; // To improve readability
	m_secondMomentOfVolume(0, 0) = intg[5] + intg[6] - m_volume * (C.y() * C.y() + C.z() * C.z());
	m_secondMomentOfVolume(1, 1) = intg[4] + intg[6] - m_volume * (C.z() * C.z() + C.x() * C.x());
	m_secondMomentOfVolume(2, 2) = intg[4] + intg[5] - m_volume * (C.x() * C.x() + C.y() * C.y());
	m_secondMomentOfVolume(0, 1) = m_secondMomentOfVolume(1, 0) = -(intg[7] - m_volume * C.x() * C.y());
	m_secondMomentOfVolume(1, 2) = m_secondMomentOfVolume(2, 1) = -(intg[8] - m_volume * C.y() * C.z());
	m_secondMomentOfVolume(0, 2) = m_secondMomentOfVolume(2, 0) = -(intg[9] - m_volume * C.z() * C.x());
	// Test the second moment of volume validity
	// The diagonal element should all be positive
	SURGSIM_ASSERT(m_secondMomentOfVolume.diagonal().minCoeff() > 0.0) <<
		"A MeshShape cannot have a second moment of volume (used to compute the inertia matrix)" <<
		" with negative or null diagonal elements, we found" << std::endl << m_secondMomentOfVolume;
}

std::string MeshShape::getClassName()
{
	return std::string("SurgSim::Math::MeshShape");
}


}; // namespace Math
}; // namespace SurgSim
