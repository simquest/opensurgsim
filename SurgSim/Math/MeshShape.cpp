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

const std::shared_ptr<typename MeshShape::TriMesh>
MeshShape::getMesh() const
{
	return m_mesh;
}

double MeshShape::getVolume() const
{
	return m_T0;
}

SurgSim::Math::Vector3d MeshShape::getCenter() const
{
	return Vector3d(m_T1[0] / m_T0, m_T1[1] / m_T0, m_T1[2] / m_T0);
}

SurgSim::Math::Matrix33d MeshShape::getSecondMomentOfVolume() const
{
	Matrix33d secondMoment;
	const Vector3d& r = getCenter();
	const double volume = getVolume();

	secondMoment(0, 0) = (m_T2[1] + m_T2[2]);
	secondMoment(1, 1) = (m_T2[2] + m_T2[0]);
	secondMoment(2, 2) = (m_T2[0] + m_T2[1]);
	secondMoment(0, 1) = secondMoment(1, 0) = - m_TP[0];
	secondMoment(1, 2) = secondMoment(2, 1) = - m_TP[1];
	secondMoment(2, 0) = secondMoment(0, 2) = - m_TP[2];

	// translate tensor to center
	secondMoment(0, 0) -= volume * (r[1]*r[1] + r[0]*r[0]);
	secondMoment(1, 1) -= volume * (r[2]*r[2] + r[0]*r[0]);
	secondMoment(2, 2) -= volume * (r[0]*r[0] + r[1]*r[1]);
	secondMoment(0, 1) = secondMoment(1, 0) += volume * r[0] * r[1];
	secondMoment(1, 2) = secondMoment(2, 1) += volume * r[1] * r[2];
	secondMoment(2, 0) = secondMoment(0, 2) += volume * r[2] * r[0];

	return secondMoment;
}

void MeshShape::computeProjectionIntegrals(const MeshShape::TriMesh::TriangleType& face)
{
	double a0, a1, da;
	double b0, b1, db;
	double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
	double a1_2, a1_3, b1_2, b1_3;
	double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
	double Cab, Kab, Caab, Kaab, Cabb, Kabb;
	int i;

	m_P1 = m_Pa = m_Pb = m_Paa = m_Pab = m_Pbb = m_Paaa = m_Paab = m_Pabb = m_Pbbb = 0.0;

	for (i = 0; i < 3; i++)
	{
		a0 = m_mesh->getVertexPosition(face.verticesId[i])[m_alpha];
		b0 = m_mesh->getVertexPosition(face.verticesId[i])[m_beta];
		a1 = m_mesh->getVertexPosition(face.verticesId[(i+1) % 3])[m_alpha];
		b1 = m_mesh->getVertexPosition(face.verticesId[(i+1) % 3])[m_beta];
		da = a1 - a0;
		db = b1 - b0;
		a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
		b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
		a1_2 = a1 * a1; a1_3 = a1_2 * a1;
		b1_2 = b1 * b1; b1_3 = b1_2 * b1;

		C1 = a1 + a0;
		Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
		Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
		Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
		Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
		Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
		Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

		m_P1 += db*C1;
		m_Pa += db*Ca;
		m_Paa += db*Caa;
		m_Paaa += db*Caaa;
		m_Pb += da*Cb;
		m_Pbb += da*Cbb;
		m_Pbbb += da*Cbbb;
		m_Pab += db*(b1*Cab + b0*Kab);
		m_Paab += db*(b1*Caab + b0*Kaab);
		m_Pabb += da*(a1*Cabb + a0*Kabb);
	}

	m_P1 /= 2.0;
	m_Pa /= 6.0;
	m_Paa /= 12.0;
	m_Paaa /= 20.0;
	m_Pb /= -6.0;
	m_Pbb /= -12.0;
	m_Pbbb /= -20.0;
	m_Pab /= 24.0;
	m_Paab /= 60.0;
	m_Pabb /= -60.0;
}

void MeshShape::computeFaceIntegrals(const MeshShape::TriMesh::TriangleType& face)
{
	double k1, k2, k3, k4;

	computeProjectionIntegrals(face);

	const Vector3d& ptA = m_mesh->getVertexPosition(face.verticesId[0]);
	const Vector3d& ptB = m_mesh->getVertexPosition(face.verticesId[1]);
	const Vector3d& ptC = m_mesh->getVertexPosition(face.verticesId[2]);
	Vector3d n = (ptB - ptA).cross(ptC - ptA);
	n.normalize();
	double w = -ptA.dot(n);

	const double alpha_squared = n[m_alpha] * n[m_alpha];
	const double alpha_cubed = alpha_squared * n[m_alpha];
	const double beta_squared = n[m_beta] * n[m_beta];
	const double beta_cubed = beta_squared * n[m_beta];

	k1 = 1 / n[m_gamma]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

	m_Fa = k1 * m_Pa;
	m_Fb = k1 * m_Pb;
	m_Fc = -k2 * (n[m_alpha]*m_Pa + n[m_beta]*m_Pb + w*m_P1);

	m_Faa = k1 * m_Paa;
	m_Fbb = k1 * m_Pbb;
	m_Fcc = k3 * (alpha_squared*m_Paa + 2*n[m_alpha]*n[m_beta]*m_Pab
				  + beta_squared*m_Pbb + w*(2*(n[m_alpha]*m_Pa + n[m_beta]*m_Pb) + w*m_P1));

	m_Faaa = k1 * m_Paaa;
	m_Fbbb = k1 * m_Pbbb;
	m_Fccc = -k4 * (alpha_cubed*m_Paaa + 3*alpha_squared*n[m_beta]*m_Paab
					+ 3*n[m_alpha]*beta_squared*m_Pabb + beta_cubed*m_Pbbb
					+ 3*w*(alpha_squared*m_Paa + 2*n[m_alpha]*n[m_beta]*m_Pab
						   + beta_squared*m_Pbb)
					+ w*w*(3*(n[m_alpha]*m_Pa + n[m_beta]*m_Pb) + w*m_P1));

	m_Faab = k1 * m_Paab;
	m_Fbbc = -k2 * (n[m_alpha]*m_Pabb + n[m_beta]*m_Pbbb + w*m_Pbb);
	m_Fcca = k3 * (alpha_squared*m_Paaa + 2*n[m_alpha]*n[m_beta]*m_Paab
				   + beta_squared*m_Pabb
				   + w*(2*(n[m_alpha]*m_Paa + n[m_beta]*m_Pab) + w*m_Pa));
}

void MeshShape::computeVolumeIntegrals()
{
	double nx, ny, nz;

	m_T0 = m_T1[0] = m_T1[1] = m_T1[2]
		= m_T2[0] = m_T2[1] = m_T2[2]
		= m_TP[0] = m_TP[1] = m_TP[2] = 0.0;

	for (size_t i = 0; i < m_mesh->getNumTriangles(); i++)
	{
		const TriMesh::TriangleType* f = &m_mesh->getTriangle(i);

		const Vector3d& ptA = m_mesh->getVertexPosition(f->verticesId[0]);
		const Vector3d& ptB = m_mesh->getVertexPosition(f->verticesId[1]);
		const Vector3d& ptC = m_mesh->getVertexPosition(f->verticesId[2]);
		Vector3d n = (ptB - ptA).cross(ptC - ptA);
		n.normalize();
		nx = fabs(n[0]);
		ny = fabs(n[1]);
		nz = fabs(n[2]);
		if (nx > ny && nx > nz)
		{
			m_gamma = 0;
		}
		else
		{
			m_gamma = (ny > nz) ? 1 : 2;
		}
		m_alpha = (m_gamma + 1) % 3;
		m_beta = (m_alpha + 1) % 3;

		computeFaceIntegrals(*f);

		m_T0 += n[0] * ((m_alpha == 0) ? m_Fa : ((m_beta == 0) ? m_Fb : m_Fc));

		m_T1[m_alpha] += n[m_alpha] * m_Faa;
		m_T1[m_beta] += n[m_beta] * m_Fbb;
		m_T1[m_gamma] += n[m_gamma] * m_Fcc;
		m_T2[m_alpha] += n[m_alpha] * m_Faaa;
		m_T2[m_beta] += n[m_beta] * m_Fbbb;
		m_T2[m_gamma] += n[m_gamma] * m_Fccc;
		m_TP[m_alpha] += n[m_alpha] * m_Faab;
		m_TP[m_beta] += n[m_beta] * m_Fbbc;
		m_TP[m_gamma] += n[m_gamma] * m_Fcca;
	}

	m_T1[0] /= 2; m_T1[1] /= 2; m_T1[2] /= 2;
	m_T2[0] /= 3; m_T2[1] /= 3; m_T2[2] /= 3;
	m_TP[0] /= 2; m_TP[1] /= 2; m_TP[2] /= 2;
}

std::string MeshShape::getClassName()
{
	return std::string("SurgSim::Math::MeshShape");
}


}; // namespace Math
}; // namespace SurgSim