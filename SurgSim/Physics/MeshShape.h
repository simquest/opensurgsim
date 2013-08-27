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

// Parts of the code in this file are based on research and
// public-domain code by Brian Mirtich.
// (http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html)


#ifndef SURGSIM_PHYSICS_MESHSHAPE_H
#define SURGSIM_PHYSICS_MESHSHAPE_H

#include <SurgSim/Physics/RigidShape.h>
#include <SurgSim/DataStructures/TriangleMesh.h>

namespace SurgSim
{

namespace Physics
{

/// Mesh shape: shape made of a triangle mesh
/// Various physical properties are computed from the triangle mesh using
/// work by Brian Mirtich.
/// http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html
template <class VertexData, class EdgeData, class TriangleData>
class MeshShape: public RigidShape
{
	/// Type TriMesh for convenience
	typedef SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData> TriMesh;

public:
	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	explicit MeshShape(const std::shared_ptr<TriMesh> mesh)
	{
		m_mesh = mesh;
		computeVolumeIntegrals();
	}

	/// \return the type of the shape
	int getType()
	{
		return RIGID_SHAPE_TYPE_MESH;
	}

	/// Get mesh
	/// \return The triangle mesh associated to this MeshShape
	const std::shared_ptr<TriMesh> getMesh() const
	{
		return m_mesh;
	}

	/// Calculate the volume of the mesh
	/// \return The volume of the mesh (in m-3)
	virtual double calculateVolume() const
	{
		return m_T0;
	}

	/// Calculate the mass center of the mesh
	/// \return The mass center of the mesh
	Vector3d calculateMassCenter() const
	{
		return Vector3d(m_T1[0] / m_T0, m_T1[1] / m_T0, m_T1[2] / m_T0);
	}

	/// Calculate the inertia of the mesh
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the mesh
	Matrix33d calculateInertia(double rho) const
	{
		Matrix33d inertia;
		const Vector3d& r = calculateMassCenter();
		const double mass = calculateMass(rho);

		inertia(0, 0) = rho * (m_T2[1] + m_T2[2]);
		inertia(1, 1) = rho * (m_T2[2] + m_T2[0]);
		inertia(2, 2) = rho * (m_T2[0] + m_T2[1]);
		inertia(0, 1) = inertia(1, 0) = - rho * m_TP[0];
		inertia(1, 2) = inertia(2, 1) = - rho * m_TP[1];
		inertia(2, 0) = inertia(0, 2) = - rho * m_TP[2];

		// translate inertia tensor to center of mass
		inertia(0, 0) -= mass * (r[1]*r[1] + r[0]*r[0]);
		inertia(1, 1) -= mass * (r[2]*r[2] + r[0]*r[0]);
		inertia(2, 2) -= mass * (r[0]*r[0] + r[1]*r[1]);
		inertia(0, 1) = inertia(1, 0) += mass * r[0] * r[1];
		inertia(1, 2) = inertia(2, 1) += mass * r[1] * r[2];
		inertia(2, 0) = inertia(0, 2) += mass * r[2] * r[0];

		return inertia;
	}

private:

	/// Calculate squared value
	/// \param x A real
	/// \return x^2
	static double square(double x)
	{
		return ((x)*(x));
	}

	/// Calculate cubic value
	/// \param x A real
	/// \return x^3
	static double cube(double x)
	{
		return ((x)*(x)*(x));
	}

	/// Compute various integrations over projection of face
	/// \param face A triangle
	void computeProjectionIntegrals(const typename TriMesh::TriangleType& face)
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
			a0 = m_mesh->getVertexPosition(face.vertices[i])[m_alpha];
			b0 = m_mesh->getVertexPosition(face.vertices[i])[m_beta];
			a1 = m_mesh->getVertexPosition(face.vertices[(i+1) % 3])[m_alpha];
			b1 = m_mesh->getVertexPosition(face.vertices[(i+1) % 3])[m_beta];
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

	/// Compute various integrations on a face
	/// \param face A triangle
	void computeFaceIntegrals(const typename TriMesh::TriangleType& face)
	{
		double k1, k2, k3, k4;

		computeProjectionIntegrals(face);

		const Vector3d& ptA = m_mesh->getVertexPosition(face.vertices[0]);
		const Vector3d& ptB = m_mesh->getVertexPosition(face.vertices[1]);
		const Vector3d& ptC = m_mesh->getVertexPosition(face.vertices[2]);
		Vector3d n = (ptB - ptA).cross(ptC - ptA);
		n.normalize();
		double w = -ptA.dot(n);

		k1 = 1 / n[m_gamma]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

		m_Fa = k1 * m_Pa;
		m_Fb = k1 * m_Pb;
		m_Fc = -k2 * (n[m_alpha]*m_Pa + n[m_beta]*m_Pb + w*m_P1);

		m_Faa = k1 * m_Paa;
		m_Fbb = k1 * m_Pbb;
		m_Fcc = k3 * (MeshShape::square(n[m_alpha])*m_Paa + 2*n[m_alpha]*n[m_beta]*m_Pab
					  + MeshShape::square(n[m_beta])*m_Pbb + w*(2*(n[m_alpha]*m_Pa + n[m_beta]*m_Pb) + w*m_P1));

		m_Faaa = k1 * m_Paaa;
		m_Fbbb = k1 * m_Pbbb;
		m_Fccc = -k4 * (MeshShape::cube(n[m_alpha])*m_Paaa + 3*MeshShape::square(n[m_alpha])*n[m_beta]*m_Paab
						+ 3*n[m_alpha]*MeshShape::square(n[m_beta])*m_Pabb + MeshShape::cube(n[m_beta])*m_Pbbb
						+ 3*w*(MeshShape::square(n[m_alpha])*m_Paa + 2*n[m_alpha]*n[m_beta]*m_Pab
							   + MeshShape::square(n[m_beta])*m_Pbb)
						+ w*w*(3*(n[m_alpha]*m_Pa + n[m_beta]*m_Pb) + w*m_P1));

		m_Faab = k1 * m_Paab;
		m_Fbbc = -k2 * (n[m_alpha]*m_Pabb + n[m_beta]*m_Pbbb + w*m_Pbb);
		m_Fcca = k3 * (MeshShape::square(n[m_alpha])*m_Paaa + 2*n[m_alpha]*n[m_beta]*m_Paab
					   + MeshShape::square(n[m_beta])*m_Pabb
					   + w*(2*(n[m_alpha]*m_Paa + n[m_beta]*m_Pab) + w*m_Pa));
	}

	/// Compute various volume integrals over the triangle mesh
	void computeVolumeIntegrals()
	{
		double nx, ny, nz;
		unsigned int i;

		m_T0 = m_T1[0] = m_T1[1] = m_T1[2]
			= m_T2[0] = m_T2[1] = m_T2[2]
			= m_TP[0] = m_TP[1] = m_TP[2] = 0.0;

		for (i = 0; i < m_mesh->getNumTriangles(); i++)
		{
			const typename TriMesh::TriangleType* f = &m_mesh->getTriangle(i);

			const Vector3d& ptA = m_mesh->getVertexPosition(f->vertices[0]);
			const Vector3d& ptB = m_mesh->getVertexPosition(f->vertices[1]);
			const Vector3d& ptC = m_mesh->getVertexPosition(f->vertices[2]);
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

	/// Data structure from Brian Mirtich
	int m_alpha;
	int m_beta;
	int m_gamma;

	//// projection integrals (Data structure from Brian Mirtich)
	double m_P1, m_Pa, m_Pb, m_Paa, m_Pab, m_Pbb, m_Paaa, m_Paab, m_Pabb, m_Pbbb;

	/// face integrals (Data structure from Brian Mirtich)
	double m_Fa, m_Fb, m_Fc, m_Faa, m_Fbb, m_Fcc, m_Faaa, m_Fbbb, m_Fccc, m_Faab, m_Fbbc, m_Fcca;

	/// volume integrals (Data structure from Brian Mirtich)
	double m_T0, m_T1[3], m_T2[3], m_TP[3];

	/// Triangle mesh associated to this MeshShape
	std::shared_ptr<TriMesh> m_mesh;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_MESHSHAPE_H
