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

#include <SurgSim/Physics/FemElement3DTetrahedron.h>
#include <SurgSim/Physics/DeformableRepresentationState.h>

namespace SurgSim
{

namespace Physics
{

FemElement3DTetrahedron::FemElement3DTetrahedron(std::array<unsigned int, 4> nodeIds,
												 const DeformableRepresentationState& restState) :
	m_rho(0.0),
	m_E(0.0),
	m_nu(0.0)
{
	using SurgSim::Math::getSubVector;

	this->m_numDofPerNode = 3; // 3 dof per node (x, y, z)

	for (auto nodeId = nodeIds.cbegin(); nodeId != nodeIds.cend(); nodeId++)
	{
		this->m_nodeIds.push_back(*nodeId);
	}

	// Compute the fem tetrahedron shape functions Ni(x,y,z) = 1/6V ( ai + x.bi + y.ci + z.di )
	computeShapeFunctions(restState);

	// Store the rest state for this tetrahedron in m_x0
	getSubVector(m_x0, 0, 3) = getSubVector(restState.getPositions(), m_nodeIds[0], 3);
	getSubVector(m_x0, 1, 3) = getSubVector(restState.getPositions(), m_nodeIds[1], 3);
	getSubVector(m_x0, 2, 3) = getSubVector(restState.getPositions(), m_nodeIds[2], 3);
	getSubVector(m_x0, 3, 3) = getSubVector(restState.getPositions(), m_nodeIds[3], 3);
}

void FemElement3DTetrahedron::setMassDensity(double rho)
{
	m_rho = rho;
}

double FemElement3DTetrahedron::getMassDensity() const
{
	return m_rho;
}

void FemElement3DTetrahedron::setYoungModulus(double E)
{
	m_E = E;
}

double FemElement3DTetrahedron::getYoungModulus() const
{
	return m_E;
}

void FemElement3DTetrahedron::setPoissonRatio(double nu)
{
	m_nu = nu;
}

double FemElement3DTetrahedron::getPoissonRatio() const
{
	return m_nu;
}

void FemElement3DTetrahedron::addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::addSubVector;

	// K.U = F
	// K.(x - x0) = F
	Eigen::Matrix<double, 12, 1> x;
	Eigen::Matrix<double, 12, 1> f;
	Eigen::Matrix<double, 12, 12> k;
	computeStiffness(state, &k);
	getSubVector(x, 0, 3) = getSubVector(state.getPositions(), m_nodeIds[0], 3);
	getSubVector(x, 1, 3) = getSubVector(state.getPositions(), m_nodeIds[1], 3);
	getSubVector(x, 2, 3) = getSubVector(state.getPositions(), m_nodeIds[2], 3);
	getSubVector(x, 3, 3) = getSubVector(state.getPositions(), m_nodeIds[3], 3);
	f = k * (x - m_x0);
	addSubVector(f, m_nodeIds, 3, F);
}

void FemElement3DTetrahedron::addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M)
{
	using SurgSim::Math::getSubMatrix;

	// From Przemieniecki book
	// -> section 11 "Inertia properties of structural elements
	//  -> subsection 8 "Solid Tetrahedron"
	//
	// On each axis (x, y, z), the mass matrix is
	// m = rho * volume / 20 * (2 1 1 1)
	//                         (1 2 1 1)
	//                         (1 1 2 1)
	//                         (1 1 1 2)
	//
	//         x1 y1 z1 x2 y2 z2 x3 y3 z3 x4 y4 z4
	//     x1 ( 2        1        1        1       )
	//     y1 (    2        1        1        1    )
	//     z1 (       2        1        1        1 )
	//     x2 ( 1        2        1        1       )
	//     y2 (    1        2        1        1    )
	// M = z2 (       1        2        1        1 ) * rho * volume / 20
	//     x3 ( 1        1        2        1       )
	//     y3 (    1        1        2        1    )
	//     z3 (       1        1        2        1 )
	//     x4 ( 1        1        1        2       )
	//     y4 (    1        1        1        2    )
	//     z4 (       1        1        1        2 )

	double coef = getVolume(state) * m_rho / 20.0;
	for (unsigned int rowNodeId = 0; rowNodeId < 4; rowNodeId++)
	{
		for (unsigned int colNodeId = 0; colNodeId < 4; colNodeId++)
		{
			auto Mii = getSubMatrix(*M, m_nodeIds[rowNodeId], m_nodeIds[colNodeId], 3, 3);

			if (rowNodeId == colNodeId)
			{
				Mii += SurgSim::Math::Matrix33d::Identity() * 2.0 * coef;
			}
			else
			{
				Mii += SurgSim::Math::Matrix33d::Identity() * coef;
			}
		}
	}
}

void FemElement3DTetrahedron::addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D)
{
}

void FemElement3DTetrahedron::computeStiffness(const DeformableRepresentationState& state,
											   Eigen::Matrix<double, 12, 12>* k)
{
	m_Em.setZero();
	m_strain.setZero();

	// Compute the strain matrix
	double coef = 1.0 / (6.0 * m_restVolume);
	for(int i=0 ; i<4 ; i++)
	{
		m_strain(0, 3*i  ) = coef * m_bi[i];
		m_strain(1, 3*i+1) = coef * m_ci[i];
		m_strain(2, 3*i+2) = coef * m_di[i];
		m_strain(3, 3*i  ) = coef * m_ci[i]; m_strain(3, 3*i+1) = coef * m_bi[i];
		m_strain(4 ,3*i+1) = coef * m_di[i]; m_strain(4, 3*i+2) = coef * m_ci[i];
		m_strain(5 ,3*i  ) = coef * m_di[i]; m_strain(5, 3*i+2) = coef * m_bi[i];
	}

	// Compute the elasticity material matrix
	// which is commonly based on the Lame coefficients (1st = lambda, 2nd = mu = shear modulus):
	double lambda = m_E * m_nu / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));
	double mu = m_E / (2.0 * (1 + m_nu));
	m_Em(0, 0) = m_Em(1, 1) = m_Em(2, 2) = 2.0 * mu + lambda;
	m_Em(0, 1) = m_Em(0, 2) = m_Em(1, 0) = m_Em(1, 2) = m_Em(2, 0) = m_Em(2, 1) = lambda;
	m_Em(3, 3) = m_Em(4, 4) = m_Em(5, 5) = mu;

	// Compute the stress matrix
	m_stress = m_Em * m_strain;

	// Compute the stiffness matrix
	*k = m_restVolume * m_strain.transpose() * m_stress;

	// Ke is symmetric but numerical computation might introduce epsilon, we force the symmetry here
	*k = ((*k) + (*k).transpose()) * 0.5;
}

void FemElement3DTetrahedron::addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K)
{
	using SurgSim::Math::getSubMatrix;
	using SurgSim::Math::addSubMatrix;

	Eigen::Matrix<double, 12, 12> k;
	computeStiffness(state, &k);
	addSubMatrix(k, getNodeIds(), 3, K);
}

void FemElement3DTetrahedron::addFMDK(const DeformableRepresentationState& state,
	SurgSim::Math::Vector* F,
	SurgSim::Math::Matrix* M,
	SurgSim::Math::Matrix* D,
	SurgSim::Math::Matrix* K)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::getSubMatrix;
	using SurgSim::Math::addSubMatrix;
	using SurgSim::Math::addSubVector;

	addMass(state, M);
	addDamping(state, D);

	Eigen::Matrix<double, 12, 12> k;
	computeStiffness(state, &k);

	// Assemble the stiffness matrix (using k)
	addSubMatrix(k, m_nodeIds, 3, K);

	// Assemble the force vector (using k)
	// K.U = F
	// K.(x - x0) = F
	Eigen::Matrix<double, 12, 1> x;
	Eigen::Matrix<double, 12, 1> f;
	getSubVector(x, 0, 3) = getSubVector(state.getPositions(), m_nodeIds[0], 3);
	getSubVector(x, 1, 3) = getSubVector(state.getPositions(), m_nodeIds[1], 3);
	getSubVector(x, 2, 3) = getSubVector(state.getPositions(), m_nodeIds[2], 3);
	getSubVector(x, 3, 3) = getSubVector(state.getPositions(), m_nodeIds[3], 3);
	f = k * (x - m_x0);
	addSubVector(f, m_nodeIds, 3, F);
}

double FemElement3DTetrahedron::det(const Vector3d& a, const Vector3d& b, const Vector3d& c) const
{
	return a[0]*b[1]*c[2] + a[2]*b[0]*c[1] + a[1]*b[2]*c[0] - a[2]*b[1]*c[0] - a[1]*b[0]*c[2] - a[0]*b[2]*c[1];
}

double FemElement3DTetrahedron::getVolume(const DeformableRepresentationState& state) const
{
	using SurgSim::Math::Vector;
	using SurgSim::Math::getSubVector;

	/// Computes the tetrahedron volume 1/6 * | 1 p0x p0y p0z |
	///                                       | 1 p1x p1y p1z |
	///                                       | 1 p2x p2y p2z |
	///                                       | 1 p3x p3y p3z |

	const Vector& x = state.getPositions();
	const Eigen::VectorBlock<const Vector> p0 = getSubVector(x, m_nodeIds[0], 3);
	const Eigen::VectorBlock<const Vector> p1 = getSubVector(x, m_nodeIds[1], 3);
	const Eigen::VectorBlock<const Vector> p2 = getSubVector(x, m_nodeIds[2], 3);
	const Eigen::VectorBlock<const Vector> p3 = getSubVector(x, m_nodeIds[3], 3);

	// fabs is necessary if we don't pay attention to the indexing !
	// If the tetrahedron verify ABC counter clock wise viewed from D, this determinant is always positive = 6V
	// Otherwise, it can happen that this determinant is negative = -6V !!
	return (det(p1, p2, p3) - det(p0, p2, p3) + det(p0, p1, p3) - det(p0, p1, p2)) / 6.0;
}

void FemElement3DTetrahedron::computeShapeFunctions(const DeformableRepresentationState& restState)
{
	// The tetrahedron nodes 3D position {a,b,c,d}
	Vector3d a = SurgSim::Math::getSubVector(restState.getPositions(), m_nodeIds[0], 3);
	Vector3d b = SurgSim::Math::getSubVector(restState.getPositions(), m_nodeIds[1], 3);
	Vector3d c = SurgSim::Math::getSubVector(restState.getPositions(), m_nodeIds[2], 3);
	Vector3d d = SurgSim::Math::getSubVector(restState.getPositions(), m_nodeIds[3], 3);
	double tmp, tmpa, tmpb, tmpc, tmpd;

	m_restVolume = getVolume(restState);

	m_ai[0] =  det( b , c , d ); // 6 V(0,b,c,d)
	m_ai[1] = -det( a , c , d ); // 6 V(0,c,d,a)
	m_ai[2] =  det( a , b , d ); // 6 V(0,d,a,b)
	m_ai[3] = -det( a , b , c ); // 6 V(0,a,b,c)

	// We should have m_restVolume = m_ai[0] + m_ai[1] + m_ai[2] + m_ai[3]

	// Save the 'x' component of the 4 vertices
	tmpa = a[0]; tmpb = b[0]; tmpc = c[0]; tmpd = d[0];
	// And replace them by '1'
	a[0] = 1.; b[0] = 1.; c[0] = 1.; d[0] = 1.;

	// HERE WE HAVE for example with BCD:
	// | 1 yb zb |
	// | 1 yc zc |
	// | 1 yd zd |
	m_bi[0] = -( det( b , c , d ) );
	m_bi[1] =  ( det( a , c , d ) );
	m_bi[2] = -( det( a , b , d ) );
	m_bi[3] =  ( det( a , b , c ) );

	// Save the 'y' component of the 4 vertices and replace it with the 'x' one which was stored in tmpx values
	tmp = a[1]; a[1] = tmpa; tmpa = tmp;
	tmp = b[1]; b[1] = tmpb; tmpb = tmp;
	tmp = c[1]; c[1] = tmpc; tmpc = tmp;
	tmp = d[1]; d[1] = tmpd; tmpd = tmp;
	// HERE WE HAVE for example with BCD:
	// | 1 xb zb |
	// | 1 xc zc |
	// | 1 xd zd |
	m_ci[0] =  ( det( b , c , d ) );
	m_ci[1] = -( det( a , c , d ) );
	m_ci[2] =  ( det( a , b , d ) );
	m_ci[3] = -( det( a , b , c ) );

	// Replace the 'z' component of the 4 vertices with the 'y' component stores in the tmpx values
	// No need to save the 'z' component as it won't be used any further !
	// HERE WE HAVE for example with BCD:
	// | 1 xb yb |
	// | 1 xc yc |
	// | 1 xd yd |
	a[2] = tmpa;
	b[2] = tmpb;
	c[2] = tmpc;
	d[2] = tmpd;
	m_di[0] = -( det( b , c , d ) );
	m_di[1] =  ( det( a , c , d ) );
	m_di[2] = -( det( a , b , d ) );
	m_di[3] =  ( det( a , b , c ) );
}

} // namespace Physics

} // namespace SurgSim
