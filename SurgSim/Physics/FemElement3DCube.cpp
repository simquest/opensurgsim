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

#include "SurgSim/Framework/Log.h"

#include "SurgSim/Math/GaussLegendreQuadrature.h"

#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/FemElement3DCube.h"

using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubVector;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

FemElement3DCube::FemElement3DCube(std::array<unsigned int, 8> nodeIds,
												 const DeformableRepresentationState& restState)
{
	using SurgSim::Framework::Logger;

	// Set the number of dof per node (3 in this case)
	setNumDofPerNode(3);

	// Set the shape functions coefficients
	// Ni(epsilon, eta, mu) = (1 + epsilon * sign(epsilon_i))(1 + eta * sign(eta_i))(1 + mu * sign(mu_i))/8
	std::array<double, 8> tmpEpsilon = {{-1.0, +1.0, +1.0, -1.0, -1.0, +1.0, +1.0, -1.0}};
	std::array<double, 8> tmpEta     = {{-1.0, -1.0, +1.0, +1.0, -1.0, -1.0, +1.0, +1.0}};
	std::array<double, 8> tmpMu      = {{-1.0, -1.0, -1.0, -1.0, +1.0, +1.0, +1.0, +1.0}};
	m_shapeFunctionsEpsilonSign = tmpEpsilon;
	m_shapeFunctionsEtaSign     = tmpEta;
	m_shapeFunctionsMuSign      = tmpMu;

	// Store the 8 nodeIds in order
	for (auto nodeId = nodeIds.cbegin(); nodeId != nodeIds.cend(); nodeId++)
	{
		SURGSIM_ASSERT(*nodeId >= 0 && *nodeId < restState.getNumNodes()) <<
			"Invalid nodeId " << *nodeId << " expected in range [0.."<< restState.getNumNodes()-1<<"]";
		m_nodeIds.push_back(*nodeId);
	}

	// Store the rest state for this cube in m_x0
	getSubVector(restState.getPositions(), m_nodeIds, 3, &m_x0);

	// Compute the cube rest volume
	m_restVolume = getVolume(restState);
}

void FemElement3DCube::initialize(const DeformableRepresentationState& state)
{
	// Test the validity of the physical parameters
	FemElement::initialize(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state, &m_mass);
	computeStiffness(state, &m_strain, &m_stress, &m_stiffness);
}

void FemElement3DCube::addForce(const DeformableRepresentationState& state,
									   const Eigen::Matrix<double, 24, 24>& k, SurgSim::Math::Vector* F, double scale)
{
	Eigen::Matrix<double, 24, 1> x, f;

	// K.U = Fext
	// K.(x - x0) = Fext
	// 0 = Fext + Fint     with Fint = -K.(x - x0)
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);
	f = (- scale) * k * (x - m_x0);
	addSubVector(f, m_nodeIds, 3, F);
}

void FemElement3DCube::addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
									   double scale)
{
	addForce(state, m_stiffness, F, scale);
}

void FemElement3DCube::computeMass(const DeformableRepresentationState& state,
										  Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* M)
{
	using SurgSim::Math::gaussQuadrature2Points;

	// M = rho \int_{Volume} Ni^T.Ni dV

	// Zerout the mass matrix
	M->setZero();

	// Build up the mass matrix using a 2-points Gauss-Legendre quadrature
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				addMassMatrixAtPoint(state,
					gaussQuadrature2Points[i].first,
					gaussQuadrature2Points[j].first,
					gaussQuadrature2Points[k].first,
					gaussQuadrature2Points[i].second,
					gaussQuadrature2Points[j].second,
					gaussQuadrature2Points[k].second,
					M);
			}
		}
	}
}

void FemElement3DCube::addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M,
									  double scale)
{
	addSubMatrix(m_mass * scale, m_nodeIds, 3, M);
}

void FemElement3DCube::addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D,
										 double scale)
{
}

void FemElement3DCube::computeStiffness(const DeformableRepresentationState& state,
											   Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* strain,
											   Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* stress,
											   Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* stiffness)
{
	using SurgSim::Math::gaussQuadrature2Points;

	// Zerout the stiffness matrix
	stiffness->setZero();

	// Build up the stiffness matrix using a 2-points Gauss-Legendre quadrature
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				addStrainStressStiffnessAtPoint(state,
					gaussQuadrature2Points[i].first,
					gaussQuadrature2Points[j].first,
					gaussQuadrature2Points[k].first,
					gaussQuadrature2Points[i].second,
					gaussQuadrature2Points[j].second,
					gaussQuadrature2Points[k].second,
					strain, stress, stiffness);
			}
		}
	}
}

void FemElement3DCube::evaluateJ(const DeformableRepresentationState& state, double epsilon, double eta, double mu,
								 SurgSim::Math::Matrix33d *J,
								 SurgSim::Math::Matrix33d *Jinv,
								 double *detJ) const
{
	using SurgSim::Framework::Logger;

	SURGSIM_ASSERT(J) << "Trying to evalute J with a nullptr for matrix J";

	Vector3d p[8];
	for (size_t index = 0; index < 8; index++)
	{
		p[index] = state.getPosition(m_nodeIds[index]);
	}

	// Zerout the matrix J
	J->setZero();

	// Compute J = d(x,y,z)/d(epsilon,eta,mu)
	// Note that (x,y,z) = sum((xi,yi,zi).Ni(epsilon,eta,mu))
	for (size_t index = 0; index < 8; ++index)
	{
		for(size_t axis = 0; axis < 3; ++axis)
		{
			(*J)(0, axis) += p[index][axis] * dNdepsilon(index, epsilon, eta, mu);
			(*J)(1, axis) += p[index][axis] * dNdeta    (index, epsilon, eta, mu);
			(*J)(2, axis) += p[index][axis] * dNdmu     (index, epsilon, eta, mu);
		}
	}

	if (Jinv != nullptr && detJ != nullptr)
	{
		bool invertible;
		J->computeInverseAndDetWithCheck(*Jinv, *detJ, invertible);

		SURGSIM_ASSERT(invertible) <<
			"Found a non invertible matrix J\n"<<*J<<"\ndet(J)="<<*detJ<<
			") while computing FemElement3DCube stiffness matrix\n";
		SURGSIM_LOG_IF(*detJ <= 1e-8 && *detJ >= -1e-8, Logger::getLogger("Physics"), WARNING) <<
			"Found an invalid matrix J\n"<<*J<<"\ninvertible, but det(J)="<<*detJ<<
			") while computing FemElement3DCube stiffness matrix\n";
	}
}

void FemElement3DCube::evaluateStrainDisplacement(double epsilon, double eta, double mu,
												  const SurgSim::Math::Matrix33d& Jinv,
												  Eigen::Matrix<double, 6, 24, Eigen::DontAlign> *B) const
{
	SURGSIM_ASSERT(B) << "Trying to evalute the strain-displacmenet with a nullptr";

	// Zerout the strain-displacement
	B->setZero();

	// Set non-zero entries of the strain-displacement
	for (size_t index = 0; index < 8; ++index)
	{
		// Compute dNi/d(x,y,z) = dNi/d(epsilon,eta,mu) d(epsilon,eta,mu)/d(x,y,z)
		//                      = J^{-1}.dNi/d(epsilon,eta,mu)
		Vector3d dNidEpsilonEtaMu(
			dNdepsilon(index, epsilon, eta, mu),
			dNdeta(index, epsilon, eta, mu),
			dNdmu(index, epsilon, eta, mu)
		);
		Vector3d dNidxyz = Jinv * dNidEpsilonEtaMu;

		// B = (dNi/dx      0      0)
		//     (     0 dNi/dy      0)
		//     (     0      0 dNi/dz)
		//     (dNi/dy dNi/dx      0)
		//     (     0 dNi/dz dNi/dy)
		//     (dNi/dz      0 dNi/dx)
		// c.f. http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch11.d/AFEM.Ch11.pdf
		(*B)(0, getNumDofPerNode() * index    ) = dNidxyz[0];
		(*B)(1, getNumDofPerNode() * index + 1) = dNidxyz[1];
		(*B)(2, getNumDofPerNode() * index + 2) = dNidxyz[2];
		(*B)(3, getNumDofPerNode() * index    ) = dNidxyz[1];
		(*B)(3, getNumDofPerNode() * index + 1) = dNidxyz[0];
		(*B)(4, getNumDofPerNode() * index + 1) = dNidxyz[2];
		(*B)(4, getNumDofPerNode() * index + 2) = dNidxyz[1];
		(*B)(5, getNumDofPerNode() * index    ) = dNidxyz[2];
		(*B)(5, getNumDofPerNode() * index + 2) = dNidxyz[0];
	}
}

void FemElement3DCube::addStrainStressStiffnessAtPoint(const DeformableRepresentationState& state,
	double epsilon, double eta, double mu,
	double weightEpsilon, double weightEta, double weightMu,
	Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* strain,
	Eigen::Matrix<double, 6, 24, Eigen::DontAlign>* stress,
	Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* k)
{
	using SurgSim::Framework::Logger;

	SurgSim::Math::Matrix33d J, Jinv;
	double detJ;
	Eigen::Matrix<double, 6, 24, Eigen::DontAlign> B;

	evaluateJ(state, epsilon, eta, mu, &J, &Jinv, &detJ);

	evaluateStrainDisplacement(epsilon, eta, mu, Jinv, &B);

	{
		// Compute the elasticity material matrix
		// which is commonly based on the Lame coefficients (1st = lambda, 2nd = mu = shear modulus):
		double lambda = m_E * m_nu / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));
		double mu = m_E / (2.0 * (1 + m_nu));
		m_constitutiveMaterial.setZero();
		m_constitutiveMaterial(0, 0) = m_constitutiveMaterial(1, 1) = m_constitutiveMaterial(2, 2) = 2.0 * mu + lambda;
		m_constitutiveMaterial(0, 1) = m_constitutiveMaterial(0, 2) = m_constitutiveMaterial(1, 0) = lambda;
		m_constitutiveMaterial(1, 2) = m_constitutiveMaterial(2, 0) = m_constitutiveMaterial(2, 1) = lambda;
		m_constitutiveMaterial(3, 3) = m_constitutiveMaterial(4, 4) = m_constitutiveMaterial(5, 5) = mu;
	}

	*strain += (weightEpsilon * weightEta * weightMu * detJ) * B;
	*stress += (weightEpsilon * weightEta * weightMu * detJ) * m_constitutiveMaterial * B;
	*k += (weightEpsilon * weightEta * weightMu * detJ) * B.transpose() * m_constitutiveMaterial * B;
}

void FemElement3DCube::addMassMatrixAtPoint(const DeformableRepresentationState& state,
	double epsilon, double eta, double mu,
	double weightEpsilon, double weightEta, double weightMu,
	Eigen::Matrix<double, 24, 24, Eigen::DontAlign>* m)
{
	using SurgSim::Framework::Logger;

	SurgSim::Math::Matrix33d J, Jinv;
	double detJ;
	SurgSim::Math::Matrix Ni(3, 24);

	evaluateJ(state, epsilon, eta, mu, &J, &Jinv, &detJ);

	Ni.setZero();
	for (size_t index = 0; index < 8; ++index)
	{
		double coef = N(index, epsilon, eta, mu);
		Ni(0, getNumDofPerNode() * index    ) += coef;
		Ni(1, getNumDofPerNode() * index + 1) += coef;
		Ni(2, getNumDofPerNode() * index + 2) += coef;
	}

	*m += (weightEpsilon * weightEta * weightMu * detJ * m_rho) * Ni.transpose() * Ni;
}

void FemElement3DCube::addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K,
										   double scale)
{
	addSubMatrix(m_stiffness * scale, getNodeIds(), 3, K);
}

void FemElement3DCube::addFMDK(const DeformableRepresentationState& state,
	SurgSim::Math::Vector* F,
	SurgSim::Math::Matrix* M,
	SurgSim::Math::Matrix* D,
	SurgSim::Math::Matrix* K)
{
	// Assemble the mass matrix
	addMass(state, M);

	// No damping matrix as we are using linear elasticity (not visco-elasticity)

	// Assemble the stiffness matrix
	addStiffness(state, K);

	// Assemble the force vector
	addForce(state, F);
}

void FemElement3DCube::addMatVec(const DeformableRepresentationState& state,
										double alphaM, double alphaD, double alphaK,
										const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubVector;

	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 24, 1, Eigen::DontAlign> xLoc, resLoc;
	getSubVector(x, m_nodeIds, 3, &xLoc);

	// Adds the mass contribution
	if (alphaM)
	{
		resLoc = alphaM * (m_mass * xLoc);
		addSubVector(resLoc, m_nodeIds, 3, F);
	}

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK)
	{
		resLoc = alphaK * (m_stiffness * xLoc);
		addSubVector(resLoc, m_nodeIds, 3, F);
	}
}

double FemElement3DCube::getVolume(const DeformableRepresentationState& state) const
{
	using SurgSim::Math::gaussQuadrature2Points;

	double v = 0.0;

	// Compute the volume using a 2-points Gauss-Legendre quadrature
	// V = \int_V dV
	//   = sum_{i=0}^2 sum_{j=0}^2 sum_{k=0}^2 weightEpsilon_i weightEta_j weightMu_k det(J(epsilon_i, eta_j, mu_k))
	for (int i = 0; i < 2; ++i)
	{
		double &epsilon = gaussQuadrature2Points[i].first;
		double &weightEpsilon = gaussQuadrature2Points[i].second;

		for (int j = 0; j < 2; ++j)
		{
			double &eta= gaussQuadrature2Points[j].first;
			double &weightEta = gaussQuadrature2Points[j].second;

			for (int k = 0; k < 2; ++k)
			{
				double &mu= gaussQuadrature2Points[k].first;
				double &weightMu = gaussQuadrature2Points[k].second;

				SurgSim::Math::Matrix33d J, Jinv;
				double detJ;

				evaluateJ(state, epsilon, eta, mu, &J, &Jinv, &detJ);

				v += weightEpsilon * weightEta * weightMu * detJ;
			}
		}
	}

	SURGSIM_ASSERT(v >= 0) << "FemElement3DCube ill-defined, its volume is " << v << std::endl <<
		"Please check the node ordering of your element formed by node ids " <<
		m_nodeIds[0]<<" "<<m_nodeIds[1]<<" "<<m_nodeIds[2]<<" "<<m_nodeIds[3]<<" "<<
		m_nodeIds[4]<<" "<<m_nodeIds[5]<<" "<<m_nodeIds[6]<<" "<<m_nodeIds[7]<<std::endl;

	SURGSIM_ASSERT(v > 1e-12) << "FemElement3DCube ill-defined, its volume is " << v << std::endl <<
		"Please check the node ordering of your element formed by node ids " <<
		m_nodeIds[0]<<" "<<m_nodeIds[1]<<" "<<m_nodeIds[2]<<" "<<m_nodeIds[3]<<" "<<
		m_nodeIds[4]<<" "<<m_nodeIds[5]<<" "<<m_nodeIds[6]<<" "<<m_nodeIds[7]<<std::endl;

	return v;
}

double FemElement3DCube::N(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		(1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		(1 + eta * m_shapeFunctionsEtaSign[i]) *
		(1 + mu * m_shapeFunctionsMuSign[i]);
}

double FemElement3DCube::dNdepsilon(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		m_shapeFunctionsEpsilonSign[i] *
		(1 + eta * m_shapeFunctionsEtaSign[i]) *
		(1 + mu * m_shapeFunctionsMuSign[i]);
}

double FemElement3DCube::dNdeta(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		(1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		m_shapeFunctionsEtaSign[i] *
		(1 + mu * m_shapeFunctionsMuSign[i]);
}

double FemElement3DCube::dNdmu(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		(1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		(1 + eta * m_shapeFunctionsEtaSign[i]) *
		m_shapeFunctionsMuSign[i];
}

} // namespace Physics

} // namespace SurgSim
