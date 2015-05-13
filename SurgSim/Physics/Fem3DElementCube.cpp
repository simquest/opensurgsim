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
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem3DElementCube.h"

using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubVector;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

Fem3DElementCube::Fem3DElementCube(std::array<size_t, 8> nodeIds)
{
	m_nodeIds.assign( nodeIds.cbegin(), nodeIds.cend());
}

Fem3DElementCube::Fem3DElementCube(std::vector<size_t> nodeIds)
{
	SURGSIM_ASSERT(nodeIds.size() == 8) << "Incorrect number of nodes for Fem3D cube";
	m_nodeIds.assign(nodeIds.begin(), nodeIds.end());
}

void Fem3DElementCube::initialize(const SurgSim::Math::OdeState& state)
{
	// Test the validity of the physical parameters
	FemElement::initialize(state);

	// Set the number of dof per node (3 in this case)
	setNumDofPerNode(3);

	// Set the shape functions coefficients
	// Ni(epsilon, eta, mu) = (1 + epsilon * sign(epsilon_i))(1 + eta * sign(eta_i))(1 + mu * sign(mu_i))/8
	std::array<double, 8> tmpEpsilon = {{ -1.0, +1.0, +1.0, -1.0, -1.0, +1.0, +1.0, -1.0}};
	std::array<double, 8> tmpEta     = {{ -1.0, -1.0, +1.0, +1.0, -1.0, -1.0, +1.0, +1.0}};
	std::array<double, 8> tmpMu      = {{ -1.0, -1.0, -1.0, -1.0, +1.0, +1.0, +1.0, +1.0}};
	m_shapeFunctionsEpsilonSign = tmpEpsilon;
	m_shapeFunctionsEtaSign     = tmpEta;
	m_shapeFunctionsMuSign      = tmpMu;

	// Store the 8 nodeIds in order
	for (auto nodeId = m_nodeIds.cbegin(); nodeId != m_nodeIds.cend(); ++nodeId)
	{
		SURGSIM_ASSERT(*nodeId >= 0 && *nodeId < state.getNumNodes()) <<
				"Invalid nodeId " << *nodeId << " expected in range [0.." << state.getNumNodes() - 1 << "]";
	}

	// Store the rest state for this cube in m_elementRestPosition
	getSubVector(state.getPositions(), m_nodeIds, 3, &m_elementRestPosition);

	// Compute the cube rest volume
	m_restVolume = getVolume(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state, &m_mass);
	computeStiffness(state, &m_strain, &m_stress, &m_stiffness);
}

void Fem3DElementCube::addForce(const SurgSim::Math::OdeState& state,
								const Eigen::Matrix<double, 24, 24>& k, SurgSim::Math::Vector* F, double scale)
{
	Eigen::Matrix<double, 24, 1> x, f;

	// K.U = Fext
	// K.(x - x0) = Fext
	// 0 = Fext + Fint     with Fint = -K.(x - x0)
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);
	f = (- scale) * k * (x - m_elementRestPosition);
	addSubVector(f, m_nodeIds, 3, F);
}

void Fem3DElementCube::addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale)
{
	addForce(state, m_stiffness, F, scale);
}

void Fem3DElementCube::computeMass(const SurgSim::Math::OdeState& state,
								   Eigen::Matrix<double, 24, 24>* M)
{
	using SurgSim::Math::gaussQuadrature2Points;

	// From the internal documentation on cube mass matrix calculation, we have:
	// M = rho * integration{over volume} {phi^T.phi} dV
	// with phi = (N0  0  0 N1  0  0...)
	//            ( 0 N0  0  0 N1  0...)
	//            ( 0  0 N0  0  0 N1...)
	// a 3x24 matrix filled with shape functions evaluation at a given point.
	// Using the Gauss-Legendre quadrature to evaluate this integral, we have:
	// M = sum{i=1..2} sum{j=1..2} sum{k=1..2}(w_i * w_j * _w_k * det(J) * rho * phi^T.phi)

	// Zero out the mass matrix
	M->setZero();

	// Build up the mass matrix using a 2-points Gauss-Legendre quadrature
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				addMassMatrixAtPoint(state,
									 gaussQuadrature2Points[i], gaussQuadrature2Points[j],
									 gaussQuadrature2Points[k], M);
			}
		}
	}
}

void Fem3DElementCube::addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* M,
							   double scale)
{
	assembleMatrixBlocks(m_mass * scale, m_nodeIds, 3, M, false);
}

void Fem3DElementCube::addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* D,
								  double scale)
{
}

void Fem3DElementCube::computeStiffness(const SurgSim::Math::OdeState& state,
										Eigen::Matrix<double, 6, 24>* strain,
										Eigen::Matrix<double, 6, 24>* stress,
										Eigen::Matrix<double, 24, 24>* stiffness)
{
	using SurgSim::Math::gaussQuadrature2Points;

	// Zero out the stiffness matrix
	stiffness->setZero();

	// Build up the stiffness matrix using a 2-points Gauss-Legendre quadrature
	for (int i = 0; i < 2; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 2; ++k)
			{
				addStrainStressStiffnessAtPoint(state,
												gaussQuadrature2Points[i], gaussQuadrature2Points[j],
												gaussQuadrature2Points[k],
												strain, stress, stiffness);
			}
		}
	}
}

void Fem3DElementCube::evaluateJ(const SurgSim::Math::OdeState& state, double epsilon, double eta, double mu,
								 SurgSim::Math::Matrix33d* J,
								 SurgSim::Math::Matrix33d* Jinv,
								 double* detJ) const
{
	using SurgSim::Framework::Logger;

	SURGSIM_ASSERT(J != nullptr) << "Trying to evaluate J with a nullptr for matrix J";

	Vector3d p[8];
	for (size_t index = 0; index < 8; index++)
	{
		p[index] = state.getPosition(m_nodeIds[index]);
	}

	// Zero out the matrix J
	J->setZero();

	// Compute J = d(x,y,z)/d(epsilon,eta,mu)
	// Note that (x,y,z) = for(i in {0..7}){ (x,y,z) += (xi,yi,zi).Ni(epsilon,eta,mu)}
	for (size_t index = 0; index < 8; ++index)
	{
		for (size_t axis = 0; axis < 3; ++axis)
		{
			(*J)(0, axis) += p[index][axis] * dShapeFunctiondepsilon(index, epsilon, eta, mu);
			(*J)(1, axis) += p[index][axis] * dShapeFunctiondeta(index, epsilon, eta, mu);
			(*J)(2, axis) += p[index][axis] * dShapeFunctiondmu(index, epsilon, eta, mu);
		}
	}

	if (Jinv != nullptr && detJ != nullptr)
	{
		bool invertible;
		J->computeInverseAndDetWithCheck(*Jinv, *detJ, invertible);

		SURGSIM_ASSERT(invertible) <<
								   "Found a non invertible matrix J\n" << *J << "\ndet(J)=" << *detJ <<
								   ") while computing Fem3DElementCube stiffness matrix\n";
		SURGSIM_LOG_IF(*detJ <= 1e-8 && *detJ >= -1e-8, Logger::getLogger("Physics"), WARNING) <<
				"Found an invalid matrix J\n" << *J << "\ninvertible, but det(J)=" << *detJ <<
				") while computing Fem3DElementCube stiffness matrix\n";
	}
}

void Fem3DElementCube::evaluateStrainDisplacement(double epsilon, double eta, double mu,
		const SurgSim::Math::Matrix33d& Jinv,
		Eigen::Matrix<double, 6, 24>* B) const
{
	SURGSIM_ASSERT(B != nullptr) << "Trying to evaluate the strain-displacmenet with a nullptr";

	// Zero out the strain-displacement
	B->setZero();

	// Set non-zero entries of the strain-displacement
	for (size_t index = 0; index < 8; ++index)
	{
		// Compute dNi/d(x,y,z) = dNi/d(epsilon,eta,mu) d(epsilon,eta,mu)/d(x,y,z)
		//                      = J^{-1}.dNi/d(epsilon,eta,mu)
		Vector3d dNidEpsilonEtaMu(
			dShapeFunctiondepsilon(index, epsilon, eta, mu),
			dShapeFunctiondeta(index, epsilon, eta, mu),
			dShapeFunctiondmu(index, epsilon, eta, mu)
		);
		Vector3d dNidxyz = Jinv * dNidEpsilonEtaMu;

		// B = (dNi/dx      0      0)
		//     (     0 dNi/dy      0)
		//     (     0      0 dNi/dz)
		//     (dNi/dy dNi/dx      0)
		//     (     0 dNi/dz dNi/dy)
		//     (dNi/dz      0 dNi/dx)
		// c.f. http://www.colorado.edu/engineering/CAS/courses.d/AFEM.d/AFEM.Ch11.d/AFEM.Ch11.pdf
		(*B)(0, getNumDofPerNode() * index) = dNidxyz[0];
		(*B)(1, getNumDofPerNode() * index + 1) = dNidxyz[1];
		(*B)(2, getNumDofPerNode() * index + 2) = dNidxyz[2];
		(*B)(3, getNumDofPerNode() * index) = dNidxyz[1];
		(*B)(3, getNumDofPerNode() * index + 1) = dNidxyz[0];
		(*B)(4, getNumDofPerNode() * index + 1) = dNidxyz[2];
		(*B)(4, getNumDofPerNode() * index + 2) = dNidxyz[1];
		(*B)(5, getNumDofPerNode() * index) = dNidxyz[2];
		(*B)(5, getNumDofPerNode() * index + 2) = dNidxyz[0];
	}
}

void Fem3DElementCube::buildConstitutiveMaterialMatrix(
	Eigen::Matrix<double, 6, 6>* constitutiveMatrix)
{
	// Compute the elasticity material matrix
	// which is commonly based on the Lame coefficients (1st = lambda, 2nd = mu = shear modulus):
	double lambda = m_E * m_nu / ((1.0 + m_nu) * (1.0 - 2.0 * m_nu));
	double mu = m_E / (2.0 * (1 + m_nu));
	constitutiveMatrix->setZero();
	(*constitutiveMatrix)(0, 0) = (*constitutiveMatrix)(1, 1) = (*constitutiveMatrix)(2, 2) = 2.0 * mu + lambda;
	(*constitutiveMatrix)(0, 1) = (*constitutiveMatrix)(0, 2) = (*constitutiveMatrix)(1, 0) = lambda;
	(*constitutiveMatrix)(1, 2) = (*constitutiveMatrix)(2, 0) = (*constitutiveMatrix)(2, 1) = lambda;
	(*constitutiveMatrix)(3, 3) = (*constitutiveMatrix)(4, 4) = (*constitutiveMatrix)(5, 5) = mu;
}

void Fem3DElementCube::addStrainStressStiffnessAtPoint(const SurgSim::Math::OdeState& state,
		const SurgSim::Math::gaussQuadraturePoint& epsilon,
		const SurgSim::Math::gaussQuadraturePoint& eta,
		const SurgSim::Math::gaussQuadraturePoint& mu,
		Eigen::Matrix<double, 6, 24>* strain,
		Eigen::Matrix<double, 6, 24>* stress,
		Eigen::Matrix<double, 24, 24>* k)
{
	SurgSim::Math::Matrix33d J, Jinv;
	double detJ;
	Eigen::Matrix<double, 6, 24> B;

	evaluateJ(state, epsilon.point, eta.point, mu.point, &J, &Jinv, &detJ);

	evaluateStrainDisplacement(epsilon.point, eta.point, mu.point, Jinv, &B);

	buildConstitutiveMaterialMatrix(&m_constitutiveMaterial);

	*strain += (epsilon.weight * eta.weight * mu.weight * detJ) * B;
	*stress += (epsilon.weight * eta.weight * mu.weight * detJ) * m_constitutiveMaterial * B;
	*k += (epsilon.weight * eta.weight * mu.weight * detJ) * B.transpose() * m_constitutiveMaterial * B;
}

void Fem3DElementCube::addMassMatrixAtPoint(const SurgSim::Math::OdeState& state,
		const SurgSim::Math::gaussQuadraturePoint& epsilon,
		const SurgSim::Math::gaussQuadraturePoint& eta,
		const SurgSim::Math::gaussQuadraturePoint& mu,
		Eigen::Matrix<double, 24, 24>* m)
{
	// This helper method hels to compute:
	// M = rho * integration{over volume} {phi^T.phi} dV
	//   = sum{i=1..2} sum{j=1..2} sum{k=1..2} (w_i * w_j * w_k * det(J) * rho * phi^T.phi)
	// with phi = (N0  0  0 N1  0  0...)   a 3x24 matrix filled with shape functions
	//            ( 0 N0  0  0 N1  0...)   evaluation at a given point.
	//            ( 0  0 N0  0  0 N1...)
	// by computing the term inside the sums: (w_i * w_j * w_k * det(J) * rho * phi^T.phi)
	SurgSim::Math::Matrix33d J, Jinv;
	SurgSim::Math::Matrix phi(3, 24);
	double detJ;

	evaluateJ(state, epsilon.point, eta.point, mu.point, &J, &Jinv, &detJ);

	phi.setZero();
	for (size_t index = 0; index < 8; ++index)
	{
		double weightPerIndex = shapeFunction(index, epsilon.point, eta.point, mu.point);
		phi(0, getNumDofPerNode() * index) += weightPerIndex;
		phi(1, getNumDofPerNode() * index + 1) += weightPerIndex;
		phi(2, getNumDofPerNode() * index + 2) += weightPerIndex;
	}

	*m += (epsilon.weight * eta.weight * mu.weight * detJ * m_rho) * phi.transpose() * phi;
}

void Fem3DElementCube::addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* K,
									double scale)
{
	assembleMatrixBlocks(m_stiffness * scale, getNodeIds(), 3, K, false);
}

void Fem3DElementCube::addFMDK(const SurgSim::Math::OdeState& state,
							   SurgSim::Math::Vector* F,
							   SurgSim::Math::SparseMatrix* M,
							   SurgSim::Math::SparseMatrix* D,
							   SurgSim::Math::SparseMatrix* K)
{
	// Assemble the mass matrix
	addMass(state, M);

	// No damping matrix as we are using linear elasticity (not visco-elasticity)

	// Assemble the stiffness matrix
	addStiffness(state, K);

	// Assemble the force vector
	addForce(state, F);
}

void Fem3DElementCube::addMatVec(const SurgSim::Math::OdeState& state,
								 double alphaM, double alphaD, double alphaK,
								 const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubVector;

	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 24, 1> xElement, fElement;
	getSubVector(x, m_nodeIds, 3, &xElement);

	// Adds the mass contribution
	if (alphaM)
	{
		fElement = alphaM * (m_mass * xElement);
		addSubVector(fElement, m_nodeIds, 3, F);
	}

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK)
	{
		fElement = alphaK * (m_stiffness * xElement);
		addSubVector(fElement, m_nodeIds, 3, F);
	}
}

double Fem3DElementCube::getVolume(const SurgSim::Math::OdeState& state) const
{
	using SurgSim::Math::gaussQuadrature2Points;

	double v = 0.0;

	// Compute the volume:
	// V = integration{over volume} dV
	// Using a 2-points Gauss-Legendre quadrature:
	// V = for{i in {0..1}} for{j in {0..1}} for{k in {0..1}}
	//        V += weightEpsilon[i] * weightEta[j] * weightMu[k] * det(J(epsilon[i], eta[j], mu[k]))
	for (int i = 0; i < 2; ++i)
	{
		const double& epsilon = gaussQuadrature2Points[i].point;
		const double& weightEpsilon = gaussQuadrature2Points[i].weight;

		for (int j = 0; j < 2; ++j)
		{
			const double& eta = gaussQuadrature2Points[j].point;
			const double& weightEta = gaussQuadrature2Points[j].weight;

			for (int k = 0; k < 2; ++k)
			{
				const double& mu = gaussQuadrature2Points[k].point;
				const double& weightMu = gaussQuadrature2Points[k].weight;

				SurgSim::Math::Matrix33d J, Jinv;
				double detJ;

				evaluateJ(state, epsilon, eta, mu, &J, &Jinv, &detJ);

				v += weightEpsilon * weightEta * weightMu * detJ;
			}
		}
	}

	SURGSIM_ASSERT(v > 1e-12) << "Fem3DElementCube ill-defined, its volume is " << v << std::endl <<
							  "Please make sure the element is not degenerate and " <<
							  "check the node ordering of your element formed by node ids " <<
							  m_nodeIds[0] << " " << m_nodeIds[1] << " " << m_nodeIds[2] << " " <<
							  m_nodeIds[3] << " " <<
							  m_nodeIds[4] << " " << m_nodeIds[5] << " " << m_nodeIds[6] << " " <<
							  m_nodeIds[7] << std::endl;

	return v;
}

double Fem3DElementCube::shapeFunction(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		   (1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		   (1 + eta * m_shapeFunctionsEtaSign[i]) *
		   (1 + mu * m_shapeFunctionsMuSign[i]);
}

double Fem3DElementCube::dShapeFunctiondepsilon(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		   m_shapeFunctionsEpsilonSign[i] *
		   (1 + eta * m_shapeFunctionsEtaSign[i]) *
		   (1 + mu * m_shapeFunctionsMuSign[i]);
}

double Fem3DElementCube::dShapeFunctiondeta(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		   (1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		   m_shapeFunctionsEtaSign[i] *
		   (1 + mu * m_shapeFunctionsMuSign[i]);
}

double Fem3DElementCube::dShapeFunctiondmu(size_t i, double epsilon, double eta, double mu) const
{
	return 1.0 / 8.0 *
		   (1 + epsilon * m_shapeFunctionsEpsilonSign[i]) *
		   (1 + eta * m_shapeFunctionsEtaSign[i]) *
		   m_shapeFunctionsMuSign[i];
}

SurgSim::Math::Vector Fem3DElementCube::computeCartesianCoordinate(
	const SurgSim::Math::OdeState& state,
	const SurgSim::Math::Vector& naturalCoordinate) const
{
	SURGSIM_ASSERT(isValidCoordinate(naturalCoordinate))
			<< "naturalCoordinate must be normalized and length 8 within [0 1].";

	Vector3d cartesianCoordinate(0.0, 0.0, 0.0);

	const Vector& positions = state.getPositions();

	for (int i = 0; i < 8; i++)
	{
		cartesianCoordinate += naturalCoordinate(i) * getSubVector(positions, m_nodeIds[i], 3);
	}

	return cartesianCoordinate;
}

SurgSim::Math::Vector Fem3DElementCube::computeNaturalCoordinate(
	const SurgSim::Math::OdeState& state,
	const SurgSim::Math::Vector& cartesianCoordinate) const
{
	SURGSIM_FAILURE() << "Function " << __FUNCTION__ << " not yet implemented.";
	return SurgSim::Math::Vector3d::Zero();
}

} // namespace Physics

} // namespace SurgSim
