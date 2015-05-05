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
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"

using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
using SurgSim::Math::gaussQuadrature2DTriangle3Points;
using SurgSim::Math::getSubMatrix;
using SurgSim::Math::getSubVector;
using SurgSim::Math::setSubMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace
{
const double epsilon = 1e-8;
const double epsilonArea = 1e-10;
};

namespace SurgSim
{

namespace Physics
{
SURGSIM_REGISTER(SurgSim::Physics::FemElement, SurgSim::Physics::Fem2DElementTriangle, Fem2DElementTriangle);

Fem2DElementTriangle::Fem2DElementTriangle(std::array<size_t, 3> nodeIds)
	: m_restArea(0.0),
	  m_thickness(0.0)
{
	// 6 dof per node (x, y, z, thetaX, thetaY, thetaZ)
	setNumDofPerNode(6);

	m_nodeIds.assign(nodeIds.cbegin(), nodeIds.cend());
}

Fem2DElementTriangle::Fem2DElementTriangle(std::vector<size_t> nodeIds)
	: m_restArea(0.0),
	  m_thickness(0.0)
{
	setNumDofPerNode(6);

	SURGSIM_ASSERT(nodeIds.size() == 3) << "Incorrect nodes for Fem2D Triangle";
	m_nodeIds.assign(nodeIds.begin(), nodeIds.end());
}

void Fem2DElementTriangle::setThickness(double thickness)
{
	SURGSIM_ASSERT(thickness != 0.0) << "The thickness cannot be set to 0";
	SURGSIM_ASSERT(thickness > 0.0) << "The thickness cannot be negative (trying to set it to " << thickness << ")";

	m_thickness = thickness;
}

double Fem2DElementTriangle::getThickness() const
{
	return m_thickness;
}

double Fem2DElementTriangle::getVolume(const SurgSim::Math::OdeState& state) const
{
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);
	const Vector3d C = state.getPosition(m_nodeIds[2]);

	return m_thickness * (B - A).cross(C - A).norm() / 2.0;
}

void Fem2DElementTriangle::initialize(const SurgSim::Math::OdeState& state)
{
	// Test the validity of the physical parameters
	FemElement::initialize(state);

	SURGSIM_ASSERT(m_thickness > 0.0) << "Fem2DElementTriangle thickness should be positive and non-zero. " <<
		"Did you call setThickness(thickness) ?";

	// Store the rest state for this beam in m_x0
	getSubVector(state.getPositions(), m_nodeIds, 6, &m_x0);

	// Store the rest rotation in m_initialRotation
	m_initialRotation = computeRotation(state);

	// computeShapeFunctionsParameters needs the initial rotation and
	// is required to compute the stiffness and mass matrices
	computeShapeFunctionsParameters(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state, &m_M);
	computeStiffness(state, &m_K);
}

void Fem2DElementTriangle::addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F, double scale)
{
	Eigen::Matrix<double, 18, 1> x, f;

	// K.U = F_ext
	// K.(x - x0) = F_ext
	// 0 = F_ext + F_int, with F_int = -K.(x - x0)
	getSubVector(state.getPositions(), m_nodeIds, 6, &x);
	f = -scale * (m_K * (x - m_x0));
	addSubVector(f, m_nodeIds, 6, F);
}

void Fem2DElementTriangle::addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* M, double scale)
{
	addSubMatrix(m_M * scale, m_nodeIds, 6, M);
}

void Fem2DElementTriangle::addDamping(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* D,
									  double scale)
{
}

void Fem2DElementTriangle::addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
										double scale)
{
	addSubMatrix(m_K * scale, getNodeIds(), 6, K);
}

void Fem2DElementTriangle::addFMDK(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
							   SurgSim::Math::Matrix* M, SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K)
{
	// Assemble the mass matrix
	addMass(state, M);

	// No damping matrix as we are using linear elasticity (not visco-elasticity)

	// Assemble the stiffness matrix
	addStiffness(state, K);

	// Assemble the force vector
	addForce(state, F);
}

void Fem2DElementTriangle::addMatVec(const SurgSim::Math::OdeState& state, double alphaM, double alphaD,
								 double alphaK, const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubVector;

	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 18, 1> extractedX, extractedResult;
	getSubVector(x, m_nodeIds, 6, &extractedX);

	// Adds the mass contribution
	if (alphaM != 0.0)
	{
		extractedResult = alphaM * (m_M * extractedX);
		addSubVector(extractedResult, m_nodeIds, 6, F);
	}

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK != 0.0)
	{
		extractedResult = alphaK * (m_K * extractedX);
		addSubVector(extractedResult, m_nodeIds, 6, F);
	}
}

void Fem2DElementTriangle::computeLocalMembraneMass(const SurgSim::Math::OdeState& state,
													Eigen::Matrix<double, 18, 18>* localMassMatrix)
{
	double mass = m_rho * m_restArea * m_thickness;

	for(size_t i = 0; i < 3; ++i)
	{
		size_t j = (i + 1) % 3;
		size_t k = (j + 1) % 3;

		localMassMatrix->block<2, 2>(i * 6, i * 6).diagonal().setConstant(mass / 6.0);
		localMassMatrix->block<2, 2>(i * 6, j * 6).diagonal().setConstant(mass / 12.0);
		localMassMatrix->block<2, 2>(i * 6, k * 6).diagonal().setConstant(mass / 12.0);
	}
}

void Fem2DElementTriangle::computeLocalPlateMass(const SurgSim::Math::OdeState& state,
													Eigen::Matrix<double, 18, 18>* localMassMatrix)
{
	double coefZ = 2.0 * m_restArea * m_rho * m_thickness;
	double coefTheta = m_restArea * m_rho * (m_thickness * m_thickness * m_thickness) / 6.0;

	for(size_t nodeId1 = 0; nodeId1 < 3; ++nodeId1)
	{
		for(size_t nodeId2 = 0; nodeId2 < 3; ++nodeId2)
		{
			localMassMatrix->block<3, 3>(6 * nodeId1 + 2, 6 * nodeId2 + 2) =
				coefZ * m_integral_dT_d.block<3, 3>(3 * nodeId1, 3 * nodeId2) +
				coefTheta * m_integralHxiHxj.block<3, 3>(3 * nodeId1, 3 * nodeId2) +
				coefTheta * m_integralHyiHyj.block<3, 3>(3 * nodeId1, 3 * nodeId2);
		}
	}
}

void Fem2DElementTriangle::computeLocalMass(const SurgSim::Math::OdeState& state,
											Eigen::Matrix<double, 18, 18>* localMassMatrix)
{
	localMassMatrix->setIdentity();
	computeLocalMembraneMass(state, localMassMatrix);
	computeLocalPlateMass(state, localMassMatrix);
}

void Fem2DElementTriangle::computeMass(const SurgSim::Math::OdeState& state,
									   Eigen::Matrix<double, 18, 18>* massMatrix)
{
	computeLocalMass(state, &m_MLocal);

	// Transformation Local -> Global
	const SurgSim::Math::Matrix33d& rotation = m_initialRotation;
	const SurgSim::Math::Matrix33d rotationTranspose = m_initialRotation.transpose();
	for (size_t rowBlockId = 0; rowBlockId < 6; ++rowBlockId)
	{
		for (size_t colBlockId = 0; colBlockId < 6; ++colBlockId)
		{
			auto MLocal3x3block = getSubMatrix(m_MLocal, rowBlockId, colBlockId, 3, 3);
			setSubMatrix(rotation * MLocal3x3block * rotationTranspose, rowBlockId, colBlockId, 3, 3, massMatrix);
		}
	}
}

void Fem2DElementTriangle::computeLocalStiffness(const SurgSim::Math::OdeState& state,
												 Eigen::Matrix<double, 18, 18>* localStiffnessMatrix)
{
	// Membrane part from "Theory of Matrix Structural Analysis" from J.S. Przemieniecki
	// Compute the membrane local strain-displacement matrix
	Matrix36Type membraneStrainDisplacement = Matrix36Type::Zero();
	for(size_t i = 0; i < 3; ++i)
	{
		// Noting f(x,y) the membrane shape function, the displacement is:
		// u(x,y) = f0(x,y).u0 + f1(x,y).u1 + f2(x,y).u2
		// The strain is E=(Exx , Eyy , Exy)
		// Exx = dux/dx = df0/dx.u0x + df1/dx.u1x + df2/dx.u2x
		//                dfi/dx = bi
		membraneStrainDisplacement(0, 2 * i) = m_membraneShapeFunctionsParameters(i, 1);
		// Eyy = duy/dy = df0/dy.u0y + df1/dy.u1y + df2/dy.u2y
		//                dfi/dy = ci
		membraneStrainDisplacement(1, 2 * i + 1) = m_membraneShapeFunctionsParameters(i, 2);
		// Exy = dux/dy + duy/dx =
		// (df0/dy.u0x + df0/dx.u0y) + (df1/dy.u1x + df1/dx.u1y) + (df2/dy.u2x + df2/dx.u2y)
		membraneStrainDisplacement(2, 2 * i) = m_membraneShapeFunctionsParameters(i, 2);
		membraneStrainDisplacement(2, 2 * i + 1) = m_membraneShapeFunctionsParameters(i, 1);
	}
	// Membrane material stiffness coming from Hooke Law (isotropic material)
	Matrix33Type membraneElasticMaterial;
	membraneElasticMaterial.setIdentity();
	membraneElasticMaterial(2, 2) = 0.5 * (1.0 - m_nu);
	membraneElasticMaterial(0, 1) = m_nu;
	membraneElasticMaterial(1, 0) = m_nu;
	membraneElasticMaterial*= m_E / (1.0 - m_nu * m_nu);
	// Membrane local stiffness matrix = integral(strain:stress)
	Matrix66Type membraneKLocal =
		membraneStrainDisplacement.transpose() * membraneElasticMaterial * membraneStrainDisplacement;
	membraneKLocal *= m_thickness * m_restArea;

	// Thin-plate part from "A Study Of Three-Node Triangular Plate Bending Elements", Jean-Louis Batoz
	// Thin-plate material stiffness coming from Hooke Law (isotropic material)
	Matrix33Type plateElasticMaterial = membraneElasticMaterial;
	plateElasticMaterial *= m_thickness * m_thickness * m_thickness / 12.0;
	// Thin-Plate stiffness matrix evaluation using a 3 point Gauss quadrature for exact integration (quadratic terms)
	Matrix99Type plateKLocal = Matrix99Type::Zero();
	for (size_t pointId = 0; pointId < 3; ++pointId)
	{
		const double xi = gaussQuadrature2DTriangle3Points[pointId].coordinateXi;
		const double eta = gaussQuadrature2DTriangle3Points[pointId].coordinateEta;
		const double weight = gaussQuadrature2DTriangle3Points[pointId].weight;

		Matrix39Type strainDisplacementAtGaussPoint = batozStrainDisplacement(xi , eta);
		plateKLocal += ((2.0 * m_restArea) * 0.5 * weight) *
			strainDisplacementAtGaussPoint.transpose() * plateElasticMaterial * strainDisplacementAtGaussPoint;
	}

	// Assemble shell stiffness as combination of membrane (Ux Uy) and plate stiffnesses (Uz ThetaX ThetaY)
	// In the Kirchhof theory of Thin-Plate, the drilling dof (ThetaZ) is not considered.
	// DOF are stored as follow (Ux Uy Uz ThetaX ThetaY ThetaZ)
	localStiffnessMatrix->setIdentity();
	for(size_t row = 0; row < 3; ++row)
	{
		for(size_t column = 0; column < 3; ++column)
		{
			// Membrane part
			localStiffnessMatrix->block<2, 2>(6 * row, 6 * column) = membraneKLocal.block<2 ,2>(2 * row , 2 * column);

			// Thin-plate part
			localStiffnessMatrix->block<3, 3>(6 * row + 2, 6 * column + 2) =
				plateKLocal.block<3, 3>(3 * row, 3 * column);
		}
	}
}

void Fem2DElementTriangle::computeStiffness(const SurgSim::Math::OdeState& state,
											Eigen::Matrix<double, 18, 18>* stiffnessMatrix)
{
	computeLocalStiffness(state, &m_KLocal);

	// Transformation Local -> Global
	const SurgSim::Math::Matrix33d& rotation = m_initialRotation;
	const SurgSim::Math::Matrix33d rotationTranspose = m_initialRotation.transpose();
	for (size_t rowBlockId = 0; rowBlockId < 6; ++rowBlockId)
	{
		for (size_t colBlockId = 0; colBlockId < 6; ++colBlockId)
		{
			auto KLocal3x3block = getSubMatrix(m_KLocal, rowBlockId, colBlockId, 3, 3);
			setSubMatrix(rotation * KLocal3x3block * rotationTranspose, rowBlockId, colBlockId, 3, 3, stiffnessMatrix);
		}
	}
}

SurgSim::Math::Matrix33d Fem2DElementTriangle::computeRotation(const SurgSim::Math::OdeState& state)
{
	SurgSim::Math::Matrix33d rotation;

	// Build (A; i, j, k) an orthonormal frame
	// such that in the local frame, we have:
	// A(0, 0)
	// B(xb > 0, 0)
	// C(xc, yc > 0)
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);
	const Vector3d C = state.getPosition(m_nodeIds[2]);
	Vector3d i = B - A;
	SURGSIM_ASSERT(!i.isZero())
		<< "Degenerate triangle A=B, A=(" << A.transpose() << ") B=(" << B.transpose() << ")";
	i.normalize();
	Vector3d j = C - A;
	SURGSIM_ASSERT(!j.isZero())
		<< "Degenerate triangle A=C, A=(" << A.transpose() << ") C=(" << C.transpose() << ")";
	Vector3d k = i.cross(j);
	SURGSIM_ASSERT(!k.isZero()) << "Degenerate triangle ABC aligned or B=C, "<<
		"A=(" << A.transpose() << ") " <<
		"B=(" << B.transpose() << ") " <<
		"C=(" << C.transpose() << ")";
	k.normalize();
	j = k.cross(i);
	j.normalize();

	// Initialize the initial rotation matrix (transform vectors from local to global coordinates)
	rotation.col(0) = i;
	rotation.col(1) = j;
	rotation.col(2) = k;

	return rotation;
}

SurgSim::Math::Vector Fem2DElementTriangle::computeCartesianCoordinate(
	const SurgSim::Math::OdeState& state,
	const SurgSim::Math::Vector& naturalCoordinate) const
{
	SURGSIM_ASSERT(isValidCoordinate(naturalCoordinate)) << "naturalCoordinate must be normalized and length 3.";

	Vector3d cartesianCoordinate(0.0, 0.0, 0.0);

	const Vector& positions = state.getPositions();

	for (int i = 0; i < 3; i++)
	{
		cartesianCoordinate += naturalCoordinate(i) * getSubVector(positions, m_nodeIds[i], 6).segment<3>(0);
	}

	return cartesianCoordinate;
}

SurgSim::Math::Vector Fem2DElementTriangle::computeNaturalCoordinate(
	const SurgSim::Math::OdeState& state,
	const SurgSim::Math::Vector& cartesianCoordinate) const
{
	SURGSIM_FAILURE() << "Function " << __FUNCTION__ << " not yet implemented.";
	return SurgSim::Math::Vector3d::Zero();
}

void Fem2DElementTriangle::computeIntegral_dTd()
{
	// Component of the plate mass matrix coming from the displacement w(x,y) = d.U = [d1 d2 d3 d4 d5 d6 d7 d8 d9].U
	// With d = [N1, N2.y10 + N3.y20, -N2.x10 - N3.x20,
	//           N4, N5.y10 + N6.y20, -N5.x10 - N6.x20,
	//           N7, N8.y10 + N9.y20, -N8.x10 - N9.x20]
	// Let's define lambda = 1 - xi - eta
	// N1(xi, eta) = 3.lambda^2 - 2.lambda^3 + 2.xi.eta.lambda
	// N2(xi, eta) = lambda^2.xi + xi.eta.lambda/2
	// N3(xi, eta) = lambda^2.eta + xi.eta.lambda/2
	// N4(xi, eta) = 3.xi^2 - 2.xi^3 + 2.xi.eta.lambda
	// N5(xi, eta) = xi^2.(xi - 1) - xi.eta.lambda
	// N6(xi, eta) = xi^2.eta + xi.eta.lambda/2
	// N7(xi, eta) = 3.eta^2 - 2.eta^3 + 2.xi.eta.lambda
	// N8(xi, eta) = eta^2.xi + xi.eta.lambda/2
	// N9(xi, eta) = eta^2.(eta - 1) - xi.eta.lambda
	// For more details, c.f. "Shell elements: modelizations DKT, DST, DKTG and Q4g", Code_Aster, 2013, Thomas De Soza.
	// w = N1.w1 + N2.dw1/dxi + N3.dw1/deta
	//   + N4.w2 + N5.dw2/dxi + N6.dw2/deta
	//   + N7.w3 + N8.dw3/dxi + N9.dw3/deta
	//
	// Note that J = (dx/dxi=x10   dy/dxi=y10 )
	//               (dx/deta=x20  dy/deta=y20)
	// And dw/dx = -thetay
	// And dw/dy = thetax

	// with dwi/dxi = dwi/dx.dx/dxi + dwi/dy.dy/dxi = -thetayi.x10 + thetaxi.y10
	// with dwi/deta = dwi/dx.dx/deta + dwi/dy.dy/deta = -thetayi.x20 + thetaxi.y20

	// m_integral_dT_d(i, j) = int_0^1 int_0^{1-eta} di.dj dxi deta

	m_integral_dT_d.resize(9, 9);

	// In the local space, the triangle point coordinates are A(x0=0, y0=0) B(x1, y1=0) C(x2, y2)
	const double x1 = -m_xij[2]; // -(x0 - x1) = x1 (as x0 = 0)
	const double x2 = m_xij[1];  // x2 - x0    = x2 (as x0 = 0)
	const double y2 = m_yij[1];  // y2 - y0)   = y2 (as y1 = 0)

	const double x1x1 = x1 * x1;
	const double x2x2 = x2 * x2;
	const double y2y2 = y2 * y2;
	const double x1x2 = x1 * x2;

	m_integral_dT_d(0, 0) = 121.0 / 1260.0;
	m_integral_dT_d(0, 1) = 13.0 / 1260.0 * y2;
	m_integral_dT_d(0, 2) = 13.0 / 1260.0 * (-x1 - x2);
	m_integral_dT_d(0, 3) = 89.0 / 2520.0;
	m_integral_dT_d(0, 4) = 19.0 / 5040.0 * y2;
	m_integral_dT_d(0, 5) = (53.0 * x1 - 19.0 * x2) / 5040.0;
	m_integral_dT_d(0, 6) = 89.0 / 2520.0;
	m_integral_dT_d(0, 7) = 53.0 / 5040.0 * (-y2);
	m_integral_dT_d(0, 8) = (53.0 * x2 - 19.0 * x1) / 5040.0;

	m_integral_dT_d(1, 0) = m_integral_dT_d(0, 1); // symmetric part
	m_integral_dT_d(1, 1) = 31.0 / 20160.0 * y2y2;
	m_integral_dT_d(1, 2) = (-y2) / 20160.0 * (19.0 * x1 + 31.0 * x2);
	m_integral_dT_d(1, 3) = 19.0 / 5040.0 * y2;
	m_integral_dT_d(1, 4) = 11.0 / 20160.0 * y2y2;
	m_integral_dT_d(1, 5) = y2 / 20160.0 * (24.0 * x1 - 11.0 * x2);
	m_integral_dT_d(1, 6) = 17.0 / 2520.0 * y2;
	m_integral_dT_d(1, 7) = 19.0 / 10080.0 * (-y2y2);
	m_integral_dT_d(1, 8) = (-y2) / 20160.0 * (13.0 * x1 - 38.0 * x2);

	m_integral_dT_d(2, 0) = m_integral_dT_d(0, 2); // symmetric part
	m_integral_dT_d(2, 1) = m_integral_dT_d(1, 2); // symmetric part
	m_integral_dT_d(2, 2) = (31.0 * (x1x1 + x2x2) + 2.0 * 19.0 * x1x2) / 20160.0;
	m_integral_dT_d(2, 3) = (-19.0 * x2 - 2.0 * 17.0 * x1) / 5040.0;
	m_integral_dT_d(2, 4) = (-y2) / 20160.0 * (13.0 * x1 + 11.0 * x2);
	m_integral_dT_d(2, 5) = (11.0 * x2 * (x2 - x1) - 2.0 * 19.0 * x1x1) / 20160.0;
	m_integral_dT_d(2, 6) = (-19.0 * x1 - 2.0 * 17.0 * x2) / 5040.0;
	m_integral_dT_d(2, 7) = y2 / 10080.0 * (12.0 * x1 + 19.0 * x2);
	m_integral_dT_d(2, 8) = (11.0 * x1 * (x1 - x2) - 2.0 * 19.0 * x2x2) / 20160.0;

	m_integral_dT_d(3, 0) = m_integral_dT_d(0, 3); // symmetric part
	m_integral_dT_d(3, 1) = m_integral_dT_d(1, 3); // symmetric part
	m_integral_dT_d(3, 2) = m_integral_dT_d(2, 3); // symmetric part
	m_integral_dT_d(3, 3) = 121.0 / 1260.0;
	m_integral_dT_d(3, 4) = 13.0 / 1260.0 * y2;
	m_integral_dT_d(3, 5) = (-13.0 * x2 + 2.0 * 13.0 * x1) / 1260.0;
	m_integral_dT_d(3, 6) = 89.0 / 2520.0;
	m_integral_dT_d(3, 7) = 53.0 / 5040.0 * (-y2);
	m_integral_dT_d(3, 8) = (53.0 * x2 - 2.0 * 17.0 * x1)/ 5040.0;

	m_integral_dT_d(4, 0) = m_integral_dT_d(0, 4); // symmetric part
	m_integral_dT_d(4, 1) = m_integral_dT_d(1, 4); // symmetric part
	m_integral_dT_d(4, 2) = m_integral_dT_d(2, 4); // symmetric part
	m_integral_dT_d(4, 3) = m_integral_dT_d(3, 4); // symmetric part
	m_integral_dT_d(4, 4) = 31.0 / 20160.0 * y2y2;
	m_integral_dT_d(4, 5) = y2 / 20160.0 * (50.0 * x1 - 31.0 * x2);
	m_integral_dT_d(4, 6) = 17.0 / 2520.0 * y2;
	m_integral_dT_d(4, 7) = 19.0 / 10080.0 * (-y2y2);
	m_integral_dT_d(4, 8) = (-y2) / 20160.0 * (25.0 * x1 - 38.0 * x2);

	m_integral_dT_d(5, 0) = m_integral_dT_d(0, 5); // symmetric part
	m_integral_dT_d(5, 1) = m_integral_dT_d(1, 5); // symmetric part
	m_integral_dT_d(5, 2) = m_integral_dT_d(2, 5); // symmetric part
	m_integral_dT_d(5, 3) = m_integral_dT_d(3, 5); // symmetric part
	m_integral_dT_d(5, 4) = m_integral_dT_d(4, 5); // symmetric part
	m_integral_dT_d(5, 5) = (20.0 * 5.0 * x1 * (x1 - x2) + 31.0 * x2x2) / 20160.0;
	m_integral_dT_d(5, 6) = (53.0 * x1 - 2.0 * 17.0 * x2) / 5040.0;
	m_integral_dT_d(5, 7) = (-y2) / 10080.0 * (31.0 * x1 - 19.0 * x2);
	m_integral_dT_d(5, 8) = 19.0 * (-x1x1 - x2x2) / 10080.0 + 29.0 / 6720.0 * x1x2;

	m_integral_dT_d(6, 0) = m_integral_dT_d(0, 6); // symmetric part
	m_integral_dT_d(6, 1) = m_integral_dT_d(1, 6); // symmetric part
	m_integral_dT_d(6, 2) = m_integral_dT_d(2, 6); // symmetric part
	m_integral_dT_d(6, 3) = m_integral_dT_d(3, 6); // symmetric part
	m_integral_dT_d(6, 4) = m_integral_dT_d(4, 6); // symmetric part
	m_integral_dT_d(6, 5) = m_integral_dT_d(5, 6); // symmetric part
	m_integral_dT_d(6, 6) = 121.0 / 1260.0;
	m_integral_dT_d(6, 7) = (-y2) * 13.0 / 630.0;
	m_integral_dT_d(6, 8) = (-13.0 * x1 + 2.0 * 13.0 * x2) / 1260.0;

	m_integral_dT_d(7, 0) = m_integral_dT_d(0, 7); // symmetric part
	m_integral_dT_d(7, 1) = m_integral_dT_d(1, 7); // symmetric part
	m_integral_dT_d(7, 2) = m_integral_dT_d(2, 7); // symmetric part
	m_integral_dT_d(7, 3) = m_integral_dT_d(3, 7); // symmetric part
	m_integral_dT_d(7, 4) = m_integral_dT_d(4, 7); // symmetric part
	m_integral_dT_d(7, 5) = m_integral_dT_d(5, 7); // symmetric part
	m_integral_dT_d(7, 6) = m_integral_dT_d(6, 7); // symmetric part
	m_integral_dT_d(7, 7) = 5.0 / 1008.0 * y2y2;
	m_integral_dT_d(7, 8) = 5.0 / 2016.0 * y2 * (x1 - 2.0 * x2);

	m_integral_dT_d(8, 0) = m_integral_dT_d(0, 8); // symmetric part
	m_integral_dT_d(8, 1) = m_integral_dT_d(1, 8); // symmetric part
	m_integral_dT_d(8, 2) = m_integral_dT_d(2, 8); // symmetric part
	m_integral_dT_d(8, 3) = m_integral_dT_d(3, 8); // symmetric part
	m_integral_dT_d(8, 4) = m_integral_dT_d(4, 8); // symmetric part
	m_integral_dT_d(8, 5) = m_integral_dT_d(5, 8); // symmetric part
	m_integral_dT_d(8, 6) = m_integral_dT_d(6, 8); // symmetric part
	m_integral_dT_d(8, 7) = m_integral_dT_d(7, 8); // symmetric part
	m_integral_dT_d(8, 8) = 5.0 / 1008.0 * x2 * (x2 - x1) + 31.0 / 20160.0 * x1x1;
}

void Fem2DElementTriangle::computeIntegral_HxHxT()
{
	// Compute the integral terms of Batoz Hx product functions
	// m_integralHxiHxj(i, j) = int_0^1 int_0^{1-eta} Hxi.Hxj dxi deta

	const double &a4 = m_ak[0];
	const double &a5 = m_ak[1];
	const double &a6 = m_ak[2];

	const double &b4 = m_bk[0];
	const double &b5 = m_bk[1];
	const double &b6 = m_bk[2];

	const double &c4 = m_ck[0];
	const double &c5 = m_ck[1];
	const double &c6 = m_ck[2];

	m_integralHxiHxj.resize(9, 9);

	m_integralHxiHxj(0, 0) = 1./5. * (a6 * a6 - a5 * a6 + a5 * a5);
	m_integralHxiHxj(0, 1) = 1./15. * (b6 * (2.0 * a6 - a5) + b5 * (a6 - 2.0 * a5));
	m_integralHxiHxj(0, 2) = 1./15. * (c6 * (-2.0 * a6 + a5) + c5 *(-a6 + 2.0 * a5));
	m_integralHxiHxj(0, 3) = 1./10. * (a6 * (-2.0 * a6 + a5) + a4 * (a6 - a5));
	m_integralHxiHxj(0, 4) = 1./15. * (b6 * (2.0 * a6 - a5) + b4 * (a6 - a5));
	m_integralHxiHxj(0, 5) = 1./60. * a5 + 1./15. * (c6 * (-2.0 * a6 + a5) + c4 * (-a6 + a5));
	m_integralHxiHxj(0, 6) = 1./10. * (a5 * (a6 - 2.0 * a5) + a4 * (-a6 + a5));
	m_integralHxiHxj(0, 7) = 1./15. * (b5 * (a6 - 2.0 * a5) + b4 * (a6 - a5));
	m_integralHxiHxj(0, 8) = -1./60. * a6 + 1./15. * (c5 * (-a6 + 2.0 * a5) + c4 * (-a6 + a5));

	m_integralHxiHxj(1, 0) = m_integralHxiHxj(0, 1); // symmetric part
	m_integralHxiHxj(1, 1) = 4./45. * (b5 * b5 + b5 * b6 + b6 * b6);
	m_integralHxiHxj(1, 2) = 2./45. * (c5 * (-2.0 * b5 - b6) + c6 * (-b5 - 2.0 * b6));
	m_integralHxiHxj(1, 3) = 1./15. * (a6 * (-2.0 * b6 - b5) + a4 * (b6 + b5));
	m_integralHxiHxj(1, 4) = 2./45. * (b6 * (b5 + 2.0 * b6) + b4 * (b6 + b5));
	m_integralHxiHxj(1, 5) = -1./90. * b5 + 2./45. * (c6 * (-2.0 * b6 - b5) - c4 * (b6 + b5));
	m_integralHxiHxj(1, 6) = 1./15. * (a5 * (b6 + 2.0 * b5) - a4 * (b6 + b5));
	m_integralHxiHxj(1, 7) = 2./45. * (b5 * (b6 + 2.0 * b5) + b4 * (b6 + b5));
	m_integralHxiHxj(1, 8) = -1./90. * b6 - 2./45. * (c5 * (2.0 * b5 + b6) + c4 * (b6 + b5));

	m_integralHxiHxj(2, 0) = m_integralHxiHxj(0, 2); // symmetric part
	m_integralHxiHxj(2, 1) = m_integralHxiHxj(1, 2); // symmetric part
	m_integralHxiHxj(2, 2) = 1./60. + 4./45. * (c5 * c5 + c6 * c6 + c5 * c6);
	m_integralHxiHxj(2, 3) = -1./60. * a4 + 1./15. * (-a4 * (c6 + c5) + a6 * (2.0 * c6 + c5));
	m_integralHxiHxj(2, 4) = -1./90. * b4 - 2./45. * (b6 * (c5 + 2.0 * c6) + b4 * (c6 + c5));
	m_integralHxiHxj(2, 5) = -1./360. + 1./90. * (c5 + c4) + 2./45. * (c6 * (2.0 * c6 + c5) + c4 * (c6 + c5));
	m_integralHxiHxj(2, 6) = 1./60. * a4 + 1./15. * (a4 * (c6 + c5) - a5 * (c6 + 2.0 * c5));
	m_integralHxiHxj(2, 7) = -1./90. * b4 - 2./45. * (b5 * (2.0 * c5 + c6) + b4 * (c6 + c5));
	m_integralHxiHxj(2, 8) = -1./360. + 1./90. * (c6 + c4) + 2./45. * (c5 * (2.0 * c5 + c6) + c4 * (c6 + c5));

	m_integralHxiHxj(3, 0) = m_integralHxiHxj(0, 3); // symmetric part
	m_integralHxiHxj(3, 1) = m_integralHxiHxj(1, 3); // symmetric part
	m_integralHxiHxj(3, 2) = m_integralHxiHxj(2, 3); // symmetric part
	m_integralHxiHxj(3, 3) = 1./5. * (a6 * a6 - a6 * a4 + a4 * a4);
	m_integralHxiHxj(3, 4) = 1./15. * (b6 * (-2.0 * a6 + a4) + b4 * (-a6 + 2.0 * a4));
	m_integralHxiHxj(3, 5) = 1./15. * (c6 * (-a4 + 2.0 * a6) + c4 * (-2.0 * a4 + a6));
	m_integralHxiHxj(3, 6) = 1./10. * (a6 * (-a5 + a4) + a4 * (a5 - 2.0 * a4));
	m_integralHxiHxj(3, 7) = 1./15. * (-a6 * (b4 + b5) + a4 * (2.0 * b4 + b5));
	m_integralHxiHxj(3, 8) = 1./60. * a6 + 1./15. * (a6 * (c4 + c5) - a4 * (2.0 * c4 + c5));

	m_integralHxiHxj(4, 0) = m_integralHxiHxj(0, 4); // symmetric part
	m_integralHxiHxj(4, 1) = m_integralHxiHxj(1, 4); // symmetric part
	m_integralHxiHxj(4, 2) = m_integralHxiHxj(2, 4); // symmetric part
	m_integralHxiHxj(4, 3) = m_integralHxiHxj(3, 4); // symmetric part
	m_integralHxiHxj(4, 4) = 4./45. * (b6 * b6 + b6 * b4 + b4 * b4);
	m_integralHxiHxj(4, 5) = -2./45. * (b6 * (2.0 * b6 + c4) + b4 * (c6 + 2.0 * c4));
	m_integralHxiHxj(4, 6) = 1./15. * (b6 * (a5 - a4) + b4 * (a5 - 2.0 * a4));
	m_integralHxiHxj(4, 7) = 2./45. * (b6 * (b4 + b5) + b4 * (2.0 * b4 + b5));
	m_integralHxiHxj(4, 8) = -1./90. * b6 - 2./45. * (b6 * (c4 + c5) + b4 * (2.0 * c4 + c5));

	m_integralHxiHxj(5, 0) = m_integralHxiHxj(0, 5); // symmetric part
	m_integralHxiHxj(5, 1) = m_integralHxiHxj(1, 5); // symmetric part
	m_integralHxiHxj(5, 2) = m_integralHxiHxj(2, 5); // symmetric part
	m_integralHxiHxj(5, 3) = m_integralHxiHxj(3, 5); // symmetric part
	m_integralHxiHxj(5, 4) = m_integralHxiHxj(4, 5); // symmetric part
	m_integralHxiHxj(5, 5) = 1./60. + 4./45. * (c6 * c6 + c6 * c4 + c4 * c4);
	m_integralHxiHxj(5, 6) = -1./60. * a5 + 1./15. * (-a5 * (c6 + c4) + a4 * (c6 + 2.0 * c4));
	m_integralHxiHxj(5, 7) = -1./90. * b5 - 2./45. * (c6 * (b4 + b5) + c4 * (b5 + 2.0 * b4));
	m_integralHxiHxj(5, 8) = -1./360. + 1./90. * (c5 + c6) + 2./45. * (c4 * (2.0 * c4 + c5) + c6 * (c5 + c4));

	m_integralHxiHxj(6, 0) = m_integralHxiHxj(0, 6); // symmetric part
	m_integralHxiHxj(6, 1) = m_integralHxiHxj(1, 6); // symmetric part
	m_integralHxiHxj(6, 2) = m_integralHxiHxj(2, 6); // symmetric part
	m_integralHxiHxj(6, 3) = m_integralHxiHxj(3, 6); // symmetric part
	m_integralHxiHxj(6, 4) = m_integralHxiHxj(4, 6); // symmetric part
	m_integralHxiHxj(6, 5) = m_integralHxiHxj(5, 6); // symmetric part
	m_integralHxiHxj(6, 6) = 1./5. * (a4 * a4 - a4 * a5 + a5 * a5);
	m_integralHxiHxj(6, 7) = 1./15. * (a5 * (b4 + 2.0 * b5) - a4 * (2.0 * b4 + b5));
	m_integralHxiHxj(6, 8) = 1./15. * (-a5 * (c4 + 2.0 * c5) + a4 * (2.0 * c4 + c5));

	m_integralHxiHxj(7, 0) = m_integralHxiHxj(0, 7); // symmetric part
	m_integralHxiHxj(7, 1) = m_integralHxiHxj(1, 7); // symmetric part
	m_integralHxiHxj(7, 2) = m_integralHxiHxj(2, 7); // symmetric part
	m_integralHxiHxj(7, 3) = m_integralHxiHxj(3, 7); // symmetric part
	m_integralHxiHxj(7, 4) = m_integralHxiHxj(4, 7); // symmetric part
	m_integralHxiHxj(7, 5) = m_integralHxiHxj(5, 7); // symmetric part
	m_integralHxiHxj(7, 6) = m_integralHxiHxj(6, 7); // symmetric part
	m_integralHxiHxj(7, 7) = 4./45. * (b4 * b4 + b4 * b5 + b5 * b5);
	m_integralHxiHxj(7, 8) = -2./45. * (b4 * (2.0 * c4 + c5) + b5 * (c4 + 2.0 * c5));

	m_integralHxiHxj(8, 0) = m_integralHxiHxj(0, 8); // symmetric part
	m_integralHxiHxj(8, 1) = m_integralHxiHxj(1, 8); // symmetric part
	m_integralHxiHxj(8, 2) = m_integralHxiHxj(2, 8); // symmetric part
	m_integralHxiHxj(8, 3) = m_integralHxiHxj(3, 8); // symmetric part
	m_integralHxiHxj(8, 4) = m_integralHxiHxj(4, 8); // symmetric part
	m_integralHxiHxj(8, 5) = m_integralHxiHxj(5, 8); // symmetric part
	m_integralHxiHxj(8, 6) = m_integralHxiHxj(6, 8); // symmetric part
	m_integralHxiHxj(8, 7) = m_integralHxiHxj(7, 8); // symmetric part
	m_integralHxiHxj(8, 8) = 1./60. + 4./45. * (c4 * c4 + c5 * c4 + c5 * c5);
}

void Fem2DElementTriangle::computeIntegral_HyHyT()
{
	// Compute the integral terms of Batoz Hy product functions
	// m_integralHyiHyj(i, j) = int_0^1 int_0^{1-eta} Hyi.Hyj dxi deta

	const double &b4 = m_bk[0];
	const double &b5 = m_bk[1];
	const double &b6 = m_bk[2];

	const double &d4 = m_dk[0];
	const double &d5 = m_dk[1];
	const double &d6 = m_dk[2];

	const double &e4 = m_ek[0];
	const double &e5 = m_ek[1];
	const double &e6 = m_ek[2];

	m_integralHyiHyj.resize(9, 9);

	m_integralHyiHyj(0, 0) = 1./5. * (d6 * d6 - d5 * d6 + d5 * d5);
	m_integralHyiHyj(0, 1) = 1./15. * (d6 * (2.0 * e6 + e5) - d5 * (e6 + 2.0 * e5));
	m_integralHyiHyj(0, 2) = 1./15. * (b6 * (-2.0 * d6 + d5) + b5 * (-d6 + 2.0 * d5));
	m_integralHyiHyj(0, 3) = 1./10. * (d6 * (d4 - 2.0 * d6) + d5 * (-d4 + d6));
	m_integralHyiHyj(0, 4) = -1./60. * d5 + 1./15. * (d6 * (2.0 * e6 + e4) - d5 * (e6 + e4));
	m_integralHyiHyj(0, 5) = 1./15. * (b6 * (-2.0 * d6 + d5) + b4 * (-d6 + d5));
	m_integralHyiHyj(0, 6) = 1./10. * (d6 * (d5 - d4) + d5 * (-2.0 * d5 + d4));
	m_integralHyiHyj(0, 7) = 1./60. * d6 + 1./15. * (d6 * (e4 + e5) - d5 * (e4 + 2.0 * e5));
	m_integralHyiHyj(0, 8) = 1./15. * (b5 * (-d6 + 2.0 * d5) + b4 * (-d6 + d5));

	m_integralHyiHyj(1, 0) = m_integralHyiHyj(0, 1); // symmetric part
	m_integralHyiHyj(1, 1) = 1./60. + 4./45. * (e6 * e6 + e5 * e6 + e5 * e5);
	m_integralHyiHyj(1, 2) = 2./45. * (-e5 * (b6 + 2.0 * b5) - e6 * (b5 + 2.0 * b6));
	m_integralHyiHyj(1, 3) = 1./60. * d4 + 1./15. * (e6 * (d4 - 2.0 * d6) + e5 * (-d6 + d4));
	m_integralHyiHyj(1, 4) = -1./360. + 1./90. * (e5 + e4) + 2./45. * (e6 * (2.0 * e6 + e5) + e4 * (e5 + e6));
	m_integralHyiHyj(1, 5) = -1./90. * b4 - 2./45. * (e5 * (b6 + b4) + e6 * (2.0 * b6 + b4));
	m_integralHyiHyj(1, 6) = -1./60. * d4 + 1./15. * (e6 * (-d4 + d5) + e5 * (-d4 + 2.0 * d5));
	m_integralHyiHyj(1, 7) = -1./360. + 1./90. * (e4 + e6) + 2./45. * (e6 * (e5 + e4) + e5 * (e4 + 2.0 * e5));
	m_integralHyiHyj(1, 8) = -1./90. * b4 - 2./45. * (e5 * (b4 + 2.0 * b5) + e6 * (b5 + b4));

	m_integralHyiHyj(2, 0) = m_integralHyiHyj(0, 2); // symmetric part
	m_integralHyiHyj(2, 1) = m_integralHyiHyj(1, 2); // symmetric part
	m_integralHyiHyj(2, 2) = 4./45. * (b5 * b5 + b5 * b6 + b6 * b6);
	m_integralHyiHyj(2, 3) = 1./15. * (b6 * (2.0 * d6 - d4) + b5 * (d6 - d4));
	m_integralHyiHyj(2, 4) = -1./90. * b5 - 2./45. * (b6 * (2.0 * e6 + e4) + b5 * (e6 + e4));
	m_integralHyiHyj(2, 5) = 2./45. * (b6 * (2.0 * b6 + b4) + b5 *(b6 + b4));
	m_integralHyiHyj(2, 6) = 1./15. * (b6 * (-d5 + d4) + b5 * (-2.0 * d5 + d4));
	m_integralHyiHyj(2, 7) = -1./90. * b6 - 2./45. * (b6 * (e5 + e4) + b5 * (2.0 * e5 + e4));
	m_integralHyiHyj(2, 8) = 2./45. * (b5 * (b4 + 2.0 * b5) + b6 *(b4 + b5));

	m_integralHyiHyj(3, 0) = m_integralHyiHyj(0, 3); // symmetric part
	m_integralHyiHyj(3, 1) = m_integralHyiHyj(1, 3); // symmetric part
	m_integralHyiHyj(3, 2) = m_integralHyiHyj(2, 3); // symmetric part
	m_integralHyiHyj(3, 3) = 1./5. * (d6 * d6 - d6 * d4 + d4 * d4);
	m_integralHyiHyj(3, 4) = 1./15. * (d4 * (e6 + 2.0 * e4) - d6 * (2.0 * e6 + e4));
	m_integralHyiHyj(3, 5) = 1./15. * (d6 * (2.0 * b6 + b4) - d4 * (b6 + 2.0 * b4));
	m_integralHyiHyj(3, 6) = 1./10. * (d6 * (-d5 + d4) + d4 * (d5 - 2.0 * d4));
	m_integralHyiHyj(3, 7) = -1./60. * d6 + 1./15. * (-d6 * (e4 + e5) + d4 * (2.0 * e4 + e5));
	m_integralHyiHyj(3, 8) = 1./15. * (d6 * (b4 + b5) - d4 * (2.0 * b4 + b5));

	m_integralHyiHyj(4, 0) = m_integralHyiHyj(0, 4); // symmetric part
	m_integralHyiHyj(4, 1) = m_integralHyiHyj(1, 4); // symmetric part
	m_integralHyiHyj(4, 2) = m_integralHyiHyj(2, 4); // symmetric part
	m_integralHyiHyj(4, 3) = m_integralHyiHyj(3, 4); // symmetric part
	m_integralHyiHyj(4, 4) = 1./60. + 4./45. * (e6 * e6 + e6 * e4 + e4 * e4);
	m_integralHyiHyj(4, 5) = -2./45. * (e6 * (2.0 * b6 + b4) + e4 * (b6 + 2.0 * b4));
	m_integralHyiHyj(4, 6) = 1./60. * d5 + 1./15. * (e6 * (d5 - d4) + e4 * (d5 - 2.0 * d4));
	m_integralHyiHyj(4, 7) = -1./360. + 1./90. * (e6 + e5) + 2./45. * (e6 * (e5 + e4) + e4 * (e5 + 2.0 * e4));
	m_integralHyiHyj(4, 8) = -1./90. * b5 - 2./45. * (e6 * (b5 + b4) + e4 * (b5 + 2.0 * b4));

	m_integralHyiHyj(5, 0) = m_integralHyiHyj(0, 5); // symmetric part
	m_integralHyiHyj(5, 1) = m_integralHyiHyj(1, 5); // symmetric part
	m_integralHyiHyj(5, 2) = m_integralHyiHyj(2, 5); // symmetric part
	m_integralHyiHyj(5, 3) = m_integralHyiHyj(3, 5); // symmetric part
	m_integralHyiHyj(5, 4) = m_integralHyiHyj(4, 5); // symmetric part
	m_integralHyiHyj(5, 5) = 4./45. * (b6 * b6 + b6 * b4 + b4 * b4);
	m_integralHyiHyj(5, 6) = 1./15. * (b6 * (-d5 + d4) + b4 * (-d5 + 2.0 * d4));
	m_integralHyiHyj(5, 7) = -1./90. * b6 - 2./45. * (b6 * (e4 + e5) + b4 * (2.0 * e4 + e5));
	m_integralHyiHyj(5, 8) = 2./45. * (b6 * (b4 + b5) + b4 * (2.0 * b4 + b5));

	m_integralHyiHyj(6, 0) = m_integralHyiHyj(0, 6); // symmetric part
	m_integralHyiHyj(6, 1) = m_integralHyiHyj(1, 6); // symmetric part
	m_integralHyiHyj(6, 2) = m_integralHyiHyj(2, 6); // symmetric part
	m_integralHyiHyj(6, 3) = m_integralHyiHyj(3, 6); // symmetric part
	m_integralHyiHyj(6, 4) = m_integralHyiHyj(4, 6); // symmetric part
	m_integralHyiHyj(6, 5) = m_integralHyiHyj(5, 6); // symmetric part
	m_integralHyiHyj(6, 6) = 1./5. * (d4 * d4 - d5 * d4 + d5 * d5);
	m_integralHyiHyj(6, 7) = 1./15. * (d5 * (e4 + 2.0 * e5) - d4 * (2.0 * e4 + e5));
	m_integralHyiHyj(6, 8) = 1./15. * (-d5 * (b4 + 2.0 * b5) + d4 * (2.0 * b4 + b5));

	m_integralHyiHyj(7, 0) = m_integralHyiHyj(0, 7); // symmetric part
	m_integralHyiHyj(7, 1) = m_integralHyiHyj(1, 7); // symmetric part
	m_integralHyiHyj(7, 2) = m_integralHyiHyj(2, 7); // symmetric part
	m_integralHyiHyj(7, 3) = m_integralHyiHyj(3, 7); // symmetric part
	m_integralHyiHyj(7, 4) = m_integralHyiHyj(4, 7); // symmetric part
	m_integralHyiHyj(7, 5) = m_integralHyiHyj(5, 7); // symmetric part
	m_integralHyiHyj(7, 6) = m_integralHyiHyj(6, 7); // symmetric part
	m_integralHyiHyj(7, 7) = 1./60. + 4./45. * (e4 * e4 + e4 * e5 + e5 * e5);
	m_integralHyiHyj(7, 8) = -2./45. * (e4 * (2.0 * b4 + b5) + e5 * (b4 + 2.0 * b5));

	m_integralHyiHyj(8, 0) = m_integralHyiHyj(0, 8); // symmetric part
	m_integralHyiHyj(8, 1) = m_integralHyiHyj(1, 8); // symmetric part
	m_integralHyiHyj(8, 2) = m_integralHyiHyj(2, 8); // symmetric part
	m_integralHyiHyj(8, 3) = m_integralHyiHyj(3, 8); // symmetric part
	m_integralHyiHyj(8, 4) = m_integralHyiHyj(4, 8); // symmetric part
	m_integralHyiHyj(8, 5) = m_integralHyiHyj(5, 8); // symmetric part
	m_integralHyiHyj(8, 6) = m_integralHyiHyj(6, 8); // symmetric part
	m_integralHyiHyj(8, 7) = m_integralHyiHyj(7, 8); // symmetric part
	m_integralHyiHyj(8, 8) = 4./45. * (b4 * b4 + b4 * b5 + b5 * b5);
}

void Fem2DElementTriangle::computeShapeFunctionsParameters(const SurgSim::Math::OdeState& restState)
{
	SURGSIM_ASSERT(m_nodeIds[0] < restState.getNumNodes()) << "Invalid nodeId[0] = " << m_nodeIds[0] <<
		", the number of nodes is " << restState.getNumNodes();
	SURGSIM_ASSERT(m_nodeIds[1] < restState.getNumNodes()) << "Invalid nodeId[1] = " << m_nodeIds[1] <<
		", the number of nodes is " << restState.getNumNodes();
	SURGSIM_ASSERT(m_nodeIds[2] < restState.getNumNodes()) << "Invalid nodeId[2] = " << m_nodeIds[2] <<
		", the number of nodes is " << restState.getNumNodes();

	const SurgSim::Math::Vector3d a = restState.getPosition(m_nodeIds[0]);
	const SurgSim::Math::Vector3d b = restState.getPosition(m_nodeIds[1]);
	const SurgSim::Math::Vector3d c = restState.getPosition(m_nodeIds[2]);

	// Transforms the 3 triangle points in 2D triangle cartesian coordinates
	SurgSim::Math::RigidTransform3d inverseTransform;
	inverseTransform = SurgSim::Math::makeRigidTransform(m_initialRotation, a).inverse();
	SurgSim::Math::Vector2d a2D = (inverseTransform * a).segment(0, 2);
	SurgSim::Math::Vector2d b2D = (inverseTransform * b).segment(0, 2);
	SurgSim::Math::Vector2d c2D = (inverseTransform * c).segment(0, 2);

	// To avoid confusion, we base all our notation on a 0-based indexing
	// Note that Batoz paper has a 0-based indexing as well, but Przemieniecki has a 1-base indexing
	double x0 = a2D[0];
	double y0 = a2D[1];
	double x1 = b2D[0];
	double y1 = b2D[1];
	double x2 = c2D[0];
	double y2 = c2D[1];

	// Note that by construction, we should have x0=y0=0 and y1=0
	SURGSIM_ASSERT(std::abs(x0) < epsilon && std::abs(y0) < epsilon && std::abs(y1) < epsilon) <<
		"Membrane local transform problem. We should have x0=y0=y1=0, but we have x0=" << x0 <<
		" y0=" << y0 << " y1=" << y1;
	x0=y0=y1=0.0; // Force it to exactly 0 for numerical purpose

	// Also note that x1>=0 and y2>=0 by construction
	SURGSIM_ASSERT(x1 >= 0 && y2 >= 0) <<
		"Membrane local transform problem. We should have x1>=0 and y2>=0, but we have x1=" << x1 <<
		" y2=" << y2;

	// Note: 2Area(ABC) = 2A = (x0y1 + x1y2 + x2y0 - x0y2 - x1y0 - x2y1) =
	//                   (x1-x2)(y1-y0) - (x1-x0)(y1-y2) = x12y10 - x10y12
	// In the local frame, we have x0=y0=y1=0
	// 2A = x1y2  (with x1>=0 and y2>=0)
	m_restArea = x1 * y2 / 2.0;
	SURGSIM_ASSERT(m_restArea != 0.0) << "Triangle with null area, A=(" << a.transpose() <<
		"), B=(" << b.transpose() << "), C=(" << c.transpose() << ")";
	SURGSIM_ASSERT(m_restArea > 0.0) << "Triangle with negtive area, Area = " << m_restArea <<
		", A=(" << a.transpose() << "), B=(" << b.transpose() << "), C=(" << c.transpose() << ")";

	// Membrane shape functions
	// Notation: yij = yi - yj (reminder Przemieniecki use  1-based indexing, while we use 0-based)
	// Notation: xij = xi - xj (reminder Przemieniecki use  1-based indexing, while we use 0-based)
	//
	// Shape functions fi(x, y) = ai + bi.x + ci.y
	// Let's note A the area of the triangle
	//
	// Identifying coefficient for f1:
	// (a0)     1  (  x1y2-x2y1 -(x0y2-x2y0)   x0y1-x1y0 )(1)   1/2A (x1y2-x2y1)        ( x1y2-x2y1)
	// (b0) = ---- (  y1-y2     -(y0-y2)       y0-y1     )(0) = 1/2A y12         = 1/2A (-y21      )
	// (c0)    2A  (-(x1-x2)      x0-x2      -(x0-x1)    )(0)   1/2A x21                ( x21      )
	// Similarly for f2:
	// (a1)     1  (  x1y2-x2y1 -(x0y2-x2y0)   x0y1-x1y0 )(0)   1/2A (x2y0-x0y2)        (-x0y2+x2y0)
	// (b1) = ---- (  y1-y2     -(y0-y2)       y0-y1     )(1) = 1/2A y20         = 1/2A ( y20      )
	// (c1)    2A  (-(x1-x2)      x0-x2      -(x0-x1)    )(0)   1/2A x02                (-x20      )
	// Similarly for f3:
	// (a2)     1  (  x1y2-x2y1 -(x0y2-x2y0)   x0y1-x1y0 )(0)    1/2A (x0y1-x1y0)        ( x0y1-x1y0)
	// (b2) = ---- (  y1-y2     -(y0-y2)       y0-y1     )(0) =  1/2A y01         = 1/2A (-y10      )
	// (c2)    2A  (-(x1-x2)      x0-x2      -(x0-x1)    )(1)    1/2A x10                ( x10      )
	//
	// The above equation has been simplified in the following assignment using x0=y0=y1=0
	// and 2A = x1y2 (cf. above for more details)

	// f0(x, y) = a0 + b0.x + c0.y, store a0 b0 c0
	m_membraneShapeFunctionsParameters(0, 0) = 1.0;                  // = (x1 * y2 - x2 * y1) / 2A
	m_membraneShapeFunctionsParameters(0, 1) = -1.0 / x1;            // = (y1 - y2) / 2A
	m_membraneShapeFunctionsParameters(0, 2) = (x2 / x1 - 1.0) / y2; // = (x2 - x1) / 2A
	// f1(x, y) = a1 + b1.x + c1.y, store a1 b1 c1
	m_membraneShapeFunctionsParameters(1, 0) = 0.0;                  // = (x2 * y0 - x0 * y2) / 2A
	m_membraneShapeFunctionsParameters(1, 1) = 1.0 / x1;             // = (y2 - y0) / 2A
	m_membraneShapeFunctionsParameters(1, 2) = -x2 / (x1 * y2);      // = (x0 - x2) / 2A
	// f2(x, y) = a2 + b2.x + c2.y, store a2 b2 c2
	m_membraneShapeFunctionsParameters(2, 0) = 0.0;                  // = (x0 * y1 - x1 * y0) / 2A
	m_membraneShapeFunctionsParameters(2, 1) = 0.0;                  // = (y0 - y1) / 2A
	m_membraneShapeFunctionsParameters(2, 2) = 1.0 / y2;             // = (x1 - x0) / 2A

	// Thin-Plate Batoz specific data
	m_xij[0] = x1 - x2; // xij[0] = x1 - x2
	m_xij[1] = x2;      // xij[1] = x2 - x0    but x0=0
	m_xij[2] = -x1;     // xij[2] = x0 - x1    but x0=0
	m_yij[0] = -y2;     // yij[0] = y1 - y2    but y1=0
	m_yij[1] = y2;      // yij[1] = y2 - y0    but y0=0
	m_yij[2] = 0.0;     // yij[2] = y0 - y1    but y0=y1=0
	for(size_t k = 0; k < 3; ++k)
	{
		m_lij_sqr[k] = m_xij[k] * m_xij[k] + m_yij[k] * m_yij[k];  // lij_sqr = xij^2 + yij^2
		m_ak[k]  = -m_xij[k] / m_lij_sqr[k];                       // ak      = -xij/li^2
		m_bk[k]  = 0.75 * m_xij[k] * m_yij[k] / m_lij_sqr[k];      // bk      = 3/4 xij yij/lij^2
		m_ck[k]  = (0.25 * m_xij[k] * m_xij[k] - 0.5 * m_yij[k] * m_yij[k]) / m_lij_sqr[k];
																	// ck      = (1/4 xij^2 - 1/2 yij^2)/lij^2
		m_dk[k]  = -m_yij[k] / m_lij_sqr[k];                       // dk      = -yij/lij^2
		m_ek[k]  = (0.25 * m_yij[k] * m_yij[k] - 0.5 * m_xij[k] * m_xij[k]) / m_lij_sqr[k];
																	// ek      = (1/4 yij^2 - 1/2 xij^2)/lij^2
		//// ... and some more for the derivatives...
		m_Pk[k]  = 6.0 * m_ak[k];                                  // Pk      = -6xij/lij^2    = 6ak
		m_qk[k]  = 4.0 * m_bk[k];                                  // qk      = 3xijyij/lij^2  = 4bk
		m_tk[k]  = 6.0 * m_dk[k];                                  // tk      = -6yij/lij^2    = 6dk
		m_rk[k]  = 3.0 * m_yij[k] * m_yij[k] / m_lij_sqr[k];       // rk      = 3yij^2/lij^2
	}

	// Pre-compute the 3 integral terms useful for the plate mass matrix
	computeIntegral_dTd();    // associated to the displacement along z (noted w)
	computeIntegral_HyHyT();  // associated to the displacement along thetax
	computeIntegral_HxHxT();  // associated to the displacement along thetay
}

std::array<double, 9> Fem2DElementTriangle::batozDhxDxi(double xi, double eta) const
{
	std::array<double, 9> res;

	double a = 1.0 - 2.0 * xi;

	res[0] = m_Pk[2] * a + (m_Pk[1] - m_Pk[2]) * eta;                   // P6(1-2xi) + (P5-P6)eta
	res[1] = m_qk[2] * a - (m_qk[1] + m_qk[2]) * eta;                   // q6(1-2xi) - (q5-q6)eta
	res[2] = -4.0 + 6.0 * (xi + eta) + m_rk[2] * a -                    // -4 + 6(xi+eta) + r6(1-2xi) - eta(r5+r6)
		eta * (m_rk[1] + m_rk[2]);

	res[3] = -m_Pk[2] * a + eta * (m_Pk[0] + m_Pk[2]);                  // -P6(1-2xi) + eta(P4+P6)
	res[4] = m_qk[2] * a - eta * (m_qk[2] - m_qk[0]);                   // q6(1-2xi) - eta(q6-q4)
	res[5] = -2.0 + 6.0 * xi + m_rk[2] * a + eta * (m_rk[0] - m_rk[2]); // -2 + 6xi + r6(1-2xi) + eta(r4-r6)

	res[6] = -eta * (m_Pk[1] + m_Pk[0]);                                // -eta(P5+P4)
	res[7] = eta * (m_qk[0] - m_qk[1]);                                 // eta(a4-a5)
	res[8] = -eta * (m_rk[1] - m_rk[0]);                                // -eta(r5-r4)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhxDeta(double xi, double eta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * eta;

	res[0] = -m_Pk[1] * a - xi * (m_Pk[2] - m_Pk[1]);                    // -P5(1-2eta) - xi(P6-P5)
	res[1] =  m_qk[1] * a - xi * (m_qk[1] + m_qk[2]);                    //  q5(1-2eta) - xi(q5+q6)
	res[2] = -4.0 + 6.0 * (xi + eta) + m_rk[1] * a -                     // -4 + 6(xi+eta) + r5(1-2eta) - xi(r5+r6)
		xi * (m_rk[1] + m_rk[2]);

	res[3] = xi * (m_Pk[0] + m_Pk[2]);                                   //  xi(P4+P6)
	res[4] = xi * (m_qk[0] - m_qk[2]);                                   //  xi(q4-q6)
	res[5] = -xi * (m_rk[2] - m_rk[0]);                                  // -xi(r6-r4)

	res[6] = m_Pk[1] * a - xi * (m_Pk[0] + m_Pk[1]);                     //  P5(1-2eta) - xi(P4+P5)
	res[7] = m_qk[1] * a + xi * (m_qk[0] - m_qk[1]);                     //  q5(1-2eta) + xi(q4-q5)
	res[8] = -2.0 + 6.0 * eta + m_rk[1] * a + xi * (m_rk[0] - m_rk[1]);  // -2 + 6eta + r5(1-2eta) + xi(r4-r5)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhyDxi(double xi, double eta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * xi;

	res[0] = m_tk[2] * a + eta * (m_tk[1] - m_tk[2]);        // t6(1-2xi) + eta(t5-t6)
	res[1] = 1.0 + m_rk[2] * a - eta * (m_rk[1] + m_rk[2]);  // 1+r6(1-2xi) - eta(r5+r6)
	res[2] = -m_qk[2] * a + eta * (m_qk[1] + m_qk[2]);       // -q6(1-2xi) + eta(q5+q6)

	res[3] = -m_tk[2] * a + eta * (m_tk[0] + m_tk[2]);       // -t6(1-2xi) + eta(t4+t6)
	res[4] = -1.0 + m_rk[2] * a + eta * (m_rk[0] - m_rk[2]); // -1 + r6(1-2xi) + eta(r4-r6)
	res[5] = -m_qk[2] * a - eta * (m_qk[0] - m_qk[2]);       // -q6(1-2xi) - eta(q4-q6)

	res[6] = -eta * (m_tk[0] + m_tk[1]);                     // -eta(t4+t5)
	res[7] = eta * (m_rk[0] - m_rk[1]);                      // eta(r4-r5)
	res[8] = -eta * (m_qk[0] - m_qk[1]);                     // -eta(q4-q5)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhyDeta(double xi, double eta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * eta;

	res[0] = -m_tk[1] * a - xi * (m_tk[2] - m_tk[1]);       // -t5(1-2eta) - xi(t6-t5)
	res[1] = 1.0 + m_rk[1] * a - xi * (m_rk[1] + m_rk[2]);  // 1+r5(1-2eta) - xi(r5+r6)
	res[2] = -m_qk[1] * a + xi * (m_qk[1] + m_qk[2]);       // -q5(1-2eta) + xi(q5+q6)

	res[3] = xi * (m_tk[0] + m_tk[2]);                      // xi(t4+t6)
	res[4] = xi * (m_rk[0] - m_rk[2]);                      // xi(r4-r6)
	res[5] = -xi * (m_qk[0] - m_qk[2]);                     // -xi(q4-q6)

	res[6] = m_tk[1] * a - xi * (m_tk[0] + m_tk[1]);        // t5(1-2eta) - xi(t4+t5)
	res[7] = -1.0 + m_rk[1] * a + xi * (m_rk[0] - m_rk[1]); // -1 + r5(1-2eta) + xi (r4-r5)
	res[8] = -m_qk[1] * a - xi * (m_qk[0] - m_qk[1]);       // -q5(1-2eta) - xi(q4-q5)

	return res;
}

Fem2DElementTriangle::Matrix39Type Fem2DElementTriangle::batozStrainDisplacement(double xi, double eta)const
{
	Matrix39Type res;
	std::array<double, 9> dHx_dxi, dHx_deta, dHy_dxi, dHy_deta;
	double coefficient = 1.0 / (2.0 * m_restArea);

	dHx_dxi   = batozDhxDxi(xi, eta);
	dHx_deta = batozDhxDeta(xi, eta);
	dHy_dxi   = batozDhyDxi(xi, eta);
	dHy_deta = batozDhyDeta(xi, eta);

	for(size_t i = 0; i < 9; ++i)
	{
		//  4 -> mid-edge 12
		//  5 -> mid-edge 20
		//  6 -> mid-edge 01
		res(0, i) = coefficient * ( m_yij[1] * dHx_dxi[i] + m_yij[2] * dHx_deta[i]);
		res(1, i) = coefficient * (-m_xij[1] * dHy_dxi[i] - m_xij[2] * dHy_deta[i]);
		res(2, i) = coefficient *
			(-m_xij[1] * dHx_dxi[i] - m_xij[2] * dHx_deta[i] + m_yij[1] * dHy_dxi[i] + m_yij[2] * dHy_deta[i]);
	}

	return res;
}

} // namespace Physics

} // namespace SurgSim
