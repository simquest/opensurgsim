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

#include "SurgSim/Math/SurfaceMeshShape.h"
#include "SurgSim/DataStructures/TriangleMeshUtilities.h"
#include "SurgSim/Framework/Log.h"

namespace
{
const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::SurfaceMeshShape, SurfaceMeshShape);

SurfaceMeshShape::SurfaceMeshShape() : m_volume(0.0), m_thickness(1e-2)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SurfaceMeshShape, std::string, FileName, getFileName, setFileName);
}

int SurfaceMeshShape::getType() const
{
	return SHAPE_TYPE_SURFACEMESH;
}


void SurfaceMeshShape::setFileName(const std::string& fileName)
{
	using SurgSim::DataStructures::TriangleMesh;
	m_fileName = fileName;
	m_mesh = SurgSim::DataStructures::loadTriangleMesh<SurgSim::DataStructures::TriangleMesh>(fileName);
}

std::string SurfaceMeshShape::getFileName() const
{
	return m_fileName;
}

std::shared_ptr<SurgSim::DataStructures::TriangleMesh> SurfaceMeshShape::getMesh()
{
	return m_mesh;
}

double SurfaceMeshShape::getVolume() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for SurfaceMeshShape, so it cannot compute volume.";
	}
	return m_volume;
}

SurgSim::Math::Vector3d SurfaceMeshShape::getCenter() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for SurfaceMeshShape, so it cannot compute center.";
	}
	return m_center;
}

SurgSim::Math::Matrix33d SurfaceMeshShape::getSecondMomentOfVolume() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for SurfaceMeshShape, so it cannot compute second moment of volume.";
	}
	return m_secondMomentOfVolume;
}

void SurfaceMeshShape::computeVolumeIntegrals()
{
	// Considering a surface mesh with thickness, the goal is to compute the following properties:
	// 1) volume = integral(over volume) dx dy dz
	// 2) center = integral(over volume) (x y z)^T dx dy dz / volume
	// 3) the inertia matrix without the mass density:
	// I = (Ixx Ixy Ixz) = integral(over volume) (y^2+z^2    -xy      -xz  ) dx dy dz
	//     (Iyx Iyy Iyz)                         (  -yx    x^2+z^2    -yz  )
	//     (Izx Izy Izz)                         (  -zx      -zy    x^2+y^2)
	// Each term of the matrix I can be evaluated independently and each monome can also be evaluated
	// independently and summed up after: integral(V) y^2+z^2 dV = integral(V) y^2 dV + integral(V) z^2 dV
	//
	// Therefore, we simply need to compute 10 different volume integral:
	// {1, x, y, z, x^2, y^2, z^2, xy, yz, zx}
	//
	// Example for the monome 'xy':
	// = integral(over volume) x.y dx dy dz = integral(over volume) x.y dV
	// = integral(over area) x.y dS * thickness
	// = [sum(over all triangles) integral(over triangle) x.y dS] * thickness
	//
	// Change the integration variables from cartesian coordinates to a parametrization of the
	// triangle ABC = {P | P = A + a.u + b.v}
	//   with
	//  { u=AB       the 1st triangle edge supporting the parametrization
	//  { v=AC       the 2nd triangle edge supporting the parametrization
	//  { 0<=a<=1    the triangle 1st coordinate in the new coordinate system (parametrized)
	//  { 0<=b<=1-a  the triangle 2nd coordinate in the new coordinate system (parametrized)
	//  => dS = |dP/db x dP/da|.db.da = |vxu|.db.da = |uxv|.db.da
	//  => x  = Px
	//  => y  = Py
	//
	// integral(over volume) x.y dx dy dz =
	// [sum(over all triangles) integral(over triangle) x.y dS] * thickness =
	// [sum(over all triangles) integral(from 0 to 1) integral(from 0 to 1-a) Px.Py db.da |uxv|] * thickness
	//
	// From here, the various integral terms can be found related to A, u and v using any formal integration tool.

	// Order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
	Eigen::VectorXd integral(10);
	integral.setZero();

	for (auto const& triangle : m_mesh->getTriangles())
	{
		if (!triangle.isValid)
		{
			continue;
		}

		auto A = m_mesh->getVertexPosition(triangle.verticesId[0]);
		auto B = m_mesh->getVertexPosition(triangle.verticesId[1]);
		auto C = m_mesh->getVertexPosition(triangle.verticesId[2]);

		// Triangle parametrization P(a, b) = A + u.a + v.b  with u=AB and v=AC
		const Vector3d u = B - A;
		const Vector3d v = C - A;
		const double area = u.cross(v).norm() / 2.0; // Triangle area

		integral[0] += area;
		integral[1] += (A[0] + (u[0] + v[0]) / 3.0) * area;
		integral[2] += (A[1] + (u[1] + v[1]) / 3.0) * area;
		integral[3] += (A[2] + (u[2] + v[2]) / 3.0) * area;
		integral[4] +=
			(A[0] * (A[0] + 2.0 * (u[0] + v[0]) / 3.0) + (u[0] * v[0] + v[0] * v[0] + u[0] * u[0]) / 6.0) * area;
		integral[5] +=
			(A[1] * (A[1] + 2.0 * (u[1] + v[1]) / 3.0) + (u[1] * v[1] + v[1] * v[1] + u[1] * u[1]) / 6.0) * area;
		integral[6] +=
			(A[2] * (A[2] + 2.0 * (u[2] + v[2]) / 3.0) + (u[2] * v[2] + v[2] * v[2] + u[2] * u[2]) / 6.0) * area;
		integral[7] += (A[0] * A[1] + (A[0] * (u[1] + v[1]) + A[1] * (u[0] + v[0])) / 3.0) * area;
		integral[7] += ((u[0] * u[1] + v[0] * v[1]) / 6.0 + (u[0] * v[1] + u[1] * v[0]) / 12.0) * area;
		integral[8] += (A[1] * A[2] + (A[1] * (u[2] + v[2]) + A[2] * (u[1] + v[1])) / 3.0) * area;
		integral[8] += ((u[1] * u[2] + v[1] * v[2]) / 6.0 + (u[1] * v[2] + u[2] * v[1]) / 12.0) * area;
		integral[9] += (A[2] * A[0] + (A[2] * (u[0] + v[0]) + A[0] * (u[2] + v[2])) / 3.0) * area;
		integral[9] += ((u[2] * u[0] + v[2] * v[0]) / 6.0 + (u[2] * v[0] + u[0] * v[2]) / 12.0) * area;
	}

	// integral[0] is the sum of all triangle's area
	double area = integral[0];

	// Center of mass
	m_center = integral.segment(1, 3);
	if (area > epsilon)
	{
		m_center /= area;
	}

	// second moment of volume relative to center
	Vector3d centerSquared = m_center.cwiseProduct(m_center);
	m_secondMomentOfVolume(0, 0) = integral[5] + integral[6] - area * (centerSquared.y() + centerSquared.z());
	m_secondMomentOfVolume(1, 1) = integral[4] + integral[6] - area * (centerSquared.z() + centerSquared.x());
	m_secondMomentOfVolume(2, 2) = integral[4] + integral[5] - area * (centerSquared.x() + centerSquared.y());
	m_secondMomentOfVolume(0, 1) = -(integral[7] - area * m_center.x() * m_center.y());
	m_secondMomentOfVolume(1, 0) = m_secondMomentOfVolume(0, 1);
	m_secondMomentOfVolume(1, 2) = -(integral[8] - area * m_center.y() * m_center.z());
	m_secondMomentOfVolume(2, 1) = m_secondMomentOfVolume(1, 2);
	m_secondMomentOfVolume(0, 2) = -(integral[9] - area * m_center.z() * m_center.x());
	m_secondMomentOfVolume(2, 0) = m_secondMomentOfVolume(0, 2);

	m_secondMomentOfVolume *=  m_thickness;
	m_volume = (area * m_thickness);
}

bool SurfaceMeshShape::isValid() const
{
	return (nullptr != m_mesh) && (m_thickness > 1e-5) && (m_mesh->isValid());
}

}; // namespace Math
}; // namespace SurgSim
