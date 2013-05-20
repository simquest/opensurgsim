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

#ifndef SURGSIM_PHYSICS_MESHSHAPE_H
#define SURGSIM_PHYSICS_MESHSHAPE_H

#include <SurgSim/Physics/Actors/RigidShape.h>
#include <SurgSim/DataStructures/TriangleMesh.h>

namespace SurgSim
{

namespace Physics
{

/// Mesh shape: shape made of a triangle mesh
/// Various physical properties are computed from the triangle mesh using
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
		return T0;
	}

	/// Calculate the mass center of the mesh
	/// \return The mass center of the mesh
	Vector3d calculateMassCenter() const
	{
		return Vector3d(T1[0] / T0, T1[1] / T0, T1[2] / T0);
	}

	/// Calculate the inertia of the mesh
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the mesh
	Matrix33d calculateInertia(double rho) const
	{
		Matrix33d inertia;
		const Vector3d& r = calculateMassCenter();
		const double mass = calculateMass(rho);

		inertia(0, 0) = rho * (T2[1] + T2[2]);
		inertia(1, 1) = rho * (T2[2] + T2[0]);
		inertia(2, 2) = rho * (T2[0] + T2[1]);
		inertia(0, 1) = inertia(1, 0) = - rho * TP[0];
		inertia(1, 2) = inertia(2, 1) = - rho * TP[1];
		inertia(2, 0) = inertia(0, 2) = - rho * TP[2];

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
	double sqr(double x) const
	{
		return ((x)*(x));
	}

	/// Calculate cubic value
	/// \param x A real
	/// \return x^3
	double cube(double x) const
	{
		return ((x)*(x)*(x));
	}

	/// Compute various integrations over projection of face
	/// \param face A triangle
	void computeProjectionIntegrals(const typename TriMesh::Triangle& face)
	{
		double a0, a1, da;
		double b0, b1, db;
		double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
		double a1_2, a1_3, b1_2, b1_3;
		double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
		double Cab, Kab, Caab, Kaab, Cabb, Kabb;
		int i;

		P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

		for (i = 0; i < 3; i++)
		{
			a0 = m_mesh->getVertexPosition(face.vertices[i])[A];
			b0 = m_mesh->getVertexPosition(face.vertices[i])[B];
			a1 = m_mesh->getVertexPosition(face.vertices[(i+1) % 3])[A];
			b1 = m_mesh->getVertexPosition(face.vertices[(i+1) % 3])[B];
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

			P1 += db*C1;
			Pa += db*Ca;
			Paa += db*Caa;
			Paaa += db*Caaa;
			Pb += da*Cb;
			Pbb += da*Cbb;
			Pbbb += da*Cbbb;
			Pab += db*(b1*Cab + b0*Kab);
			Paab += db*(b1*Caab + b0*Kaab);
			Pabb += da*(a1*Cabb + a0*Kabb);
		}

		P1 /= 2.0;
		Pa /= 6.0;
		Paa /= 12.0;
		Paaa /= 20.0;
		Pb /= -6.0;
		Pbb /= -12.0;
		Pbbb /= -20.0;
		Pab /= 24.0;
		Paab /= 60.0;
		Pabb /= -60.0;
	}

	/// Compute various integrations on a face
	/// \param face A triangle
	void computeFaceIntegrals(const typename TriMesh::Triangle& face)
	{
		double k1, k2, k3, k4;

		computeProjectionIntegrals(face);

		const Vector3d& ptA = m_mesh->getVertexPosition(face.vertices[0]);
		const Vector3d& ptB = m_mesh->getVertexPosition(face.vertices[1]);
		const Vector3d& ptC = m_mesh->getVertexPosition(face.vertices[2]);
		Vector3d n = (ptB - ptA).cross(ptC - ptA);
		n.normalize();
		double w = -ptA.dot(n);

		k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

		Fa = k1 * Pa;
		Fb = k1 * Pb;
		Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

		Faa = k1 * Paa;
		Fbb = k1 * Pbb;
		Fcc = k3 * (sqr(n[A])*Paa + 2*n[A]*n[B]*Pab + sqr(n[B])*Pbb
			+ w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

		Faaa = k1 * Paaa;
		Fbbb = k1 * Pbbb;
		Fccc = -k4 * (cube(n[A])*Paaa + 3*sqr(n[A])*n[B]*Paab
			+ 3*n[A]*sqr(n[B])*Pabb + cube(n[B])*Pbbb
			+ 3*w*(sqr(n[A])*Paa + 2*n[A]*n[B]*Pab + sqr(n[B])*Pbb)
			+ w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

		Faab = k1 * Paab;
		Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
		Fcca = k3 * (sqr(n[A])*Paaa + 2*n[A]*n[B]*Paab + sqr(n[B])*Pabb
			+ w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
	}

	/// Compute various volume integrals over the triangle mesh
	void computeVolumeIntegrals()
	{
		double nx, ny, nz;
		unsigned int i;

		T0 = T1[0] = T1[1] = T1[2]
			= T2[0] = T2[1] = T2[2]
			= TP[0] = TP[1] = TP[2] = 0.0;

		for (i = 0; i < m_mesh->getNumTriangles(); i++)
		{
			const typename TriMesh::Triangle* f = &m_mesh->getTriangle(i);

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
				C = 0;
			}
			else
			{
				C = (ny > nz) ? 1 : 2;
			}
			A = (C + 1) % 3;
			B = (A + 1) % 3;

			computeFaceIntegrals(*f);

			T0 += n[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

			T1[A] += n[A] * Faa;
			T1[B] += n[B] * Fbb;
			T1[C] += n[C] * Fcc;
			T2[A] += n[A] * Faaa;
			T2[B] += n[B] * Fbbb;
			T2[C] += n[C] * Fccc;
			TP[A] += n[A] * Faab;
			TP[B] += n[B] * Fbbc;
			TP[C] += n[C] * Fcca;
		}

		T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
		T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
		TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;
	}

	/// Data structure from Brian Mirtich
	/// alpha
	int A;
	/// beta
	int B;
	/// gamma
	int C;

	//// projection integrals (Data structure from Brian Mirtich)
	double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

	/// face integrals (Data structure from Brian Mirtich)
	double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

	/// volume integrals (Data structure from Brian Mirtich)
	double T0, T1[3], T2[3], TP[3];

	/// Triangle mesh associated to this MeshShape
	std::shared_ptr<TriMesh> m_mesh;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_MESHSHAPE_H
