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
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"

using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
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

Fem2DElementTriangle::Fem2DElementTriangle(std::array<size_t, 3> nodeIds)
	: m_restArea(0.0),
	  m_thickness(0.0)
{
	// 6 dof per node (x, y, z, thetaX, thetaY, thetaZ)
	setNumDofPerNode(6);

	m_nodeIds.assign(nodeIds.cbegin(), nodeIds.cend());
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

void Fem2DElementTriangle::computeLocalMass(const SurgSim::Math::OdeState& state,
											Eigen::Matrix<double, 18, 18>* localMassMatrix)
{
	double mass = m_rho * m_restArea * m_thickness;

	localMassMatrix->setIdentity();

	for(size_t i = 0; i < 3; ++i)
	{
		size_t j = (i + 1) % 3;
		size_t k = (j + 1) % 3;

		// Membrane inertia matrix
		// Przemieniecki book "Theory of Matrix Structural Analysis"
		// Chapter 11.6, equation 11.42 for a in-plane triangle deformation
		// m = rho.A(123).t/12.0.[2 1 1] for the axis X and Y
		//                       [1 2 1]
		//                       [1 1 2]
		localMassMatrix->block<2, 2>(i * 6, i * 6).diagonal().setConstant(mass / 6.0);
		localMassMatrix->block<2, 2>(i * 6, j * 6).diagonal().setConstant(mass / 12.0);
		localMassMatrix->block<2, 2>(i * 6, k * 6).diagonal().setConstant(mass / 12.0);

		// Plate inertia matrix developed from Batoz paper
		// Interpolation of the rotational displacement over the triangle w.r.t. DOF:
		// Uthetax(xi,neta) = z.Hx^T. U
		// Uthetay(xi,neta) = z.Hy^T. U
		//
		// Which means that the shape functions for the rotational DOF are:
		// a=(zHx^T zH^yT)
		//
		// Mass = \int_V rho.a^T.a dV
		//      = rho . \int_z \int_A a^T.a dA dz
		//      = rho . \int_{-h/2}^{h/2} \int_A z^2 ( Hx^t.Hx Hx^T.Hy ) dA dz
		//                                           ( Hy^t.Hx Hy^T.Hy )
		//
		//      = rho . [z^3/3]_{-h/2}^{h/2} \int_0^{1} \int_0^{1-neta} 2A (Hx^T.Hx  Hx^T.Hy) dxi dneta
		//                                                                 (Hy^T.Hx  Hy^T.Hy)
		//
		//      = rho . (h^3/12) . 2A . \int_0^{1} \int_0^{1-neta} (Hx^T.Hx  Hx^T.Hy) dxi dneta
		//                                                         (Hy^T.Hx  Hy^T.Hy)
		//
		// int_0^1 \int_0^(1-neta) Hx^T.Hx dxi dneta =
		//        1/5.( 2a4^2 + 2a5^2 + 2a6^2 - a4a5 - a4a6 - a5a6 ) +
		//        1/20 +
		//        4/45.( 2b4^2 + 2b5^2 + 2b6^2 + b4b5 + b4b6 + b5b6 + 2c4^2 + 2c5^2 + 2c6^2 + c4c5 + c4c6 + c5c6)
		//
		// int_0^1 \int_0^(1-neta) Hy^T.Hy dxi dneta =
		//        1/5.( 2d4^2 + 2d5^2 + 2d6^2 - d4d5 - d4d6 - d5d6 ) +
		//        1/20 +
		//        4/45.( 2b4^2 + 2b5^2 + 2b6^2 + b4b5 + b4b6 + b5b6 + 2e4^2 + 2e5^2 + 2e6^2 + e4e5 + e4e6 + e5e6)
		//
		// int_0^1 \int_0^(1-neta) Hy^T.Hx dxi dneta =
		// int_0^1 \int_0^(1-neta) Hx^T.Hy dxi dneta =
		//        1/5 .( 2a4d4 + 2a5d5 + 2a6d6) - 1/10.( a4(d5+d6) + a5(d4+d6) + a6(d4+d5) )
		//        4/45.( 2b4(e4+c4) + 2b5(e5+c5) + 2b6(e6+c6) )
		//        2/45.( b4(e5+c5) + b4(e6+c6) + b5(e4+c4) + b5(e6+c6) + b6(e4+c4) + b6(e5+c5) )
		//
		double xx = 1.0 / 20.0;
		xx += 1.0 / 5.0 * ( 2.0 * m_ak.squaredNorm() - m_ak[0] * m_ak[1] - m_ak[0] * m_ak[2] - m_ak[1] * m_ak[2] );
		xx += 4.0 / 45.0 * ( 2.0 * m_bk.squaredNorm() + m_bk[0] * m_bk[1] + m_bk[0] * m_bk[2] + m_bk[1] * m_bk[2] );
		xx += 4.0 / 45.0 * ( 2.0 * m_ck.squaredNorm() + m_ck[0] * m_ck[1] + m_ck[0] * m_ck[2] + m_ck[1] * m_ck[2] );
		double yy = 1.0 / 20.0;
		yy += 1.0 / 5.0 * ( 2.0 * m_dk.squaredNorm() - m_dk[0] * m_dk[1] - m_dk[0] * m_dk[2] - m_dk[1] * m_dk[2] );
		yy += 4.0 / 45.0 * ( 2.0 * m_bk.squaredNorm() + m_bk[0] * m_bk[1] + m_bk[0] * m_bk[2] + m_bk[1] * m_bk[2] );
		yy += 4.0 / 45.0 * ( 2.0 * m_ek.squaredNorm() + m_ek[0] * m_ek[1] + m_ek[0] * m_ek[2] + m_ek[1] * m_ek[2] );
		double xy = 0.0;
		xy += 1.0 / 5.0 * ( 2.0 * m_ak.dot(m_dk) );
		xy -= 1.0 / 10.0 * ( m_ak.dot(Vector3d(m_dk[1] + m_dk[2], m_dk[0] + m_dk[2], m_dk[0] + m_dk[1])) );
		xy += 4.0 / 45.0 * ( 2.0 * m_bk.dot(m_ek + m_ck) );
		xy += 2.0 / 45.0 * ( m_bk[0] * (m_ek[1] + m_ck[1] + m_ek[2] + m_ck[2]) );
		xy += 2.0 / 45.0 * ( m_bk[1] * (m_ek[0] + m_ck[0] + m_ek[2] + m_ck[2]) );
		xy += 2.0 / 45.0 * ( m_bk[2] * (m_ek[0] + m_ck[0] + m_ek[1] + m_ck[1]) );

		double coef2 = m_rho * (m_restArea * 2.0) * (m_thickness * m_thickness * m_thickness / 12.0);
		(*localMassMatrix)(6 * i + 3, 6 * i + 3) = coef2 * xx;
		(*localMassMatrix)(6 * i + 3, 6 * i + 4) = coef2 * xy;
		(*localMassMatrix)(6 * i + 4, 6 * i + 3) = coef2 * xy;
		(*localMassMatrix)(6 * i + 4, 6 * i + 4) = coef2 * yy;
	}
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
	Matrix36Type membraneStrainDisplacement;
	membraneStrainDisplacement.setZero();
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
	// Thin-Plate strain-displacement matrices using a 3 point Gauss quadrature located at the middle of the edges
	Matrix39Type plateStrainDisplacementGaussPoint0 = batozStrainDisplacement(0.0 , 0.5);
	Matrix39Type plateStrainDisplacementGaussPoint1 = batozStrainDisplacement(0.5 , 0.0);
	Matrix39Type plateStrainDisplacementGaussPoint2 = batozStrainDisplacement(0.5 , 0.5);
	// Thin-plate material stiffness coming from Hooke Law (isotropic material)
	Matrix33Type plateElasticMaterial = membraneElasticMaterial;
	plateElasticMaterial *= m_thickness * m_thickness * m_thickness / 12.0;
	// Thin-plate local stiffness matrix = integral(strain:stress) using a 3 points integration
	// rule over the parametrized triangle (exact integration because only quadratic terms)
	// integral(over parametric triangle) Bt.Em.B = sum(gauss point (xi,neta)) wi Bt(xi,neta).Em.B(xi,neta)
	// http://www.ams.org/journals/mcom/1959-13-068/S0025-5718-1959-0107976-5/S0025-5718-1959-0107976-5.pdf
	// = Area(parametric triangle)/3.0 * Bt(0.5, 0.0).Em.B(0.5, 0.0)
	// + Area(parametric triangle)/3.0 * Bt(0.0, 0.5).Em.B(0.0, 0.5)
	// + Area(parametric triangle)/3.0 * Bt(0.5, 0.5).Em.B(0.5, 0.5)
	double areaParametricTriangle = 1.0 / 2.0;
	Matrix99Type plateKLocal = (areaParametricTriangle / 3.0) *
		plateStrainDisplacementGaussPoint0.transpose() * plateElasticMaterial * plateStrainDisplacementGaussPoint0;
	plateKLocal += (areaParametricTriangle / 3.0) *
		plateStrainDisplacementGaussPoint1.transpose() * plateElasticMaterial * plateStrainDisplacementGaussPoint1;
	plateKLocal += (areaParametricTriangle / 3.0) *
		plateStrainDisplacementGaussPoint2.transpose() * plateElasticMaterial * plateStrainDisplacementGaussPoint2;
	plateKLocal *= 2.0 * m_restArea; // Factor due to switch from cartesian coordinates to parametric coordinates

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
}

std::array<double, 9> Fem2DElementTriangle::batozDhxDxi(double xi, double neta) const
{
	std::array<double, 9> res;

	double a = 1.0 - 2.0 * xi;

	res[0] = m_Pk[2] * a + (m_Pk[1] - m_Pk[2]) * neta;                   // P6(1-2xi) + (P5-P6)neta
	res[1] = m_qk[2] * a - (m_qk[1] + m_qk[2]) * neta;                   // q6(1-2xi) - (q5-q6)neta
	res[2] = -4.0 + 6.0 * (xi + neta) + m_rk[2] * a - neta * (m_rk[1] + m_rk[2]); // -4 + 6(xi+neta) + r6(1-2xi) -
																				  // neta(r5+r6)
	res[3] = -m_Pk[2] * a + neta * (m_Pk[0] + m_Pk[2]);                  // -P6(1-2xi) + neta(P4+P6)
	res[4] = m_qk[2] * a - neta * (m_qk[2] - m_qk[0]);                   // q6(1-2xi) - neta(q6-q4)
	res[5] = -2.0 + 6.0 * xi + m_rk[2] * a + neta * (m_rk[0] - m_rk[2]); // -2 + 6xi + r6(1-2xi) + neta(r4-r6)

	res[6] = -neta * (m_Pk[1] + m_Pk[0]);                                // -neta(P5+P4)
	res[7] = neta * (m_qk[0] - m_qk[1]);                                 // neta(a4-a5)
	res[8] = -neta * (m_rk[1] - m_rk[0]);                                // -neta(r5-r4)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhxDneta(double xi, double neta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * neta;

	res[0] = -m_Pk[1] * a - xi * (m_Pk[2] - m_Pk[1]);                    // -P5(1-2neta) - xi(P6-P5)
	res[1] =  m_qk[1] * a - xi * (m_qk[1] + m_qk[2]);                    //  q5(1-2neta) - xi(q5+q6)
	res[2] = -4.0 + 6.0 * (xi + neta) + m_rk[1] * a - xi * (m_rk[1] + m_rk[2]); // -4 + 6(xi+neta) + r5(1-2neta) -
																				// xi(r5+r6)
	res[3] = xi * (m_Pk[0] + m_Pk[2]);                                   //  xi(P4+P6)
	res[4] = xi * (m_qk[0] - m_qk[2]);                                   //  xi(q4-q6)
	res[5] = -xi * (m_rk[2] - m_rk[0]);                                  // -xi(r6-r4)

	res[6] = m_Pk[1] * a - xi * (m_Pk[0] + m_Pk[1]);                     //  P5(1-2neta) - xi(P4+P5)
	res[7] = m_qk[1] * a + xi * (m_qk[0] - m_qk[1]);                     //  q5(1-2neta) + xi(q4-q5)
	res[8] = -2.0 + 6.0 * neta + m_rk[1] * a + xi * (m_rk[0] - m_rk[1]); // -2 + 6neta + r5(1-2neta) + xi(r4-r5)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhyDxi(double xi, double neta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * xi;

	res[0] = m_tk[2] * a + neta * (m_tk[1] - m_tk[2]);        // t6(1-2xi) + neta(t5-t6)
	res[1] = 1.0 + m_rk[2] * a - neta * (m_rk[1] + m_rk[2]);  // 1+r6(1-2xi) - neta(r5+r6)
	res[2] = -m_qk[2] * a + neta * (m_qk[1] + m_qk[2]);       // -q6(1-2xi) + neta(q5+q6)

	res[3] = -m_tk[2] * a + neta * (m_tk[0] + m_tk[2]);       // -t6(1-2xi) + neta(t4+t6)
	res[4] = -1.0 + m_rk[2] * a + neta * (m_rk[0] - m_rk[2]); // -1 + r6(1-2xi) + neta(r4-r6)
	res[5] = -m_qk[2] * a - neta * (m_qk[0] - m_qk[2]);       // -q6(1-2xi) - neta(q4-q6)

	res[6] = -neta * (m_tk[0] + m_tk[1]);                     // -neta(t4+t5)
	res[7] = neta * (m_rk[0] - m_rk[1]);                      // neta(r4-r5)
	res[8] = -neta * (m_qk[0] - m_qk[1]);                     // -neta(q4-q5)

	return res;
}

std::array<double, 9> Fem2DElementTriangle::batozDhyDneta(double xi, double neta) const
{
	std::array<double, 9> res;
	double a = 1.0 - 2.0 * neta;

	res[0] = -m_tk[1] * a - xi * (m_tk[2] - m_tk[1]);       // -t5(1-2neta) - xi(t6-t5)
	res[1] = 1.0 + m_rk[1] * a - xi * (m_rk[1] + m_rk[2]);  // 1+r5(1-2neta) - xi(r5+r6)
	res[2] = -m_qk[1] * a + xi * (m_qk[1] + m_qk[2]);       // -q5(1-2neta) + xi(q5+q6)

	res[3] = xi * (m_tk[0] + m_tk[2]);                      // xi(t4+t6)
	res[4] = xi * (m_rk[0] - m_rk[2]);                      // xi(r4-r6)
	res[5] = -xi * (m_qk[0] - m_qk[2]);                     // -xi(q4-q6)

	res[6] = m_tk[1] * a - xi * (m_tk[0] + m_tk[1]);        // t5(1-2neta) - xi(t4+t5)
	res[7] = -1.0 + m_rk[1] * a + xi * (m_rk[0] - m_rk[1]); // -1 + r5(1-2neta) + xi (r4-r5)
	res[8] = -m_qk[1] * a - xi * (m_qk[0] - m_qk[1]);       // -q5(1-2neta) - xi(q4-q5)

	return res;
}

Fem2DElementTriangle::Matrix39Type Fem2DElementTriangle::batozStrainDisplacement(double xi, double neta)const
{
	Matrix39Type res;
	std::array<double, 9> dHx_dxi, dHx_dneta, dHy_dxi, dHy_dneta;
	double coefficient = 1.0 / (2.0 * m_restArea);

	dHx_dxi   = batozDhxDxi(xi, neta);
	dHx_dneta = batozDhxDneta(xi, neta);
	dHy_dxi   = batozDhyDxi(xi, neta);
	dHy_dneta = batozDhyDneta(xi, neta);

	for(size_t i = 0; i < 9; ++i)
	{
		//  4 -> mid-edge 12
		//  5 -> mid-edge 20
		//  6 -> mid-edge 01
		res(0, i) = coefficient * ( m_yij[1] * dHx_dxi[i] + m_yij[2] * dHx_dneta[i]);
		res(1, i) = coefficient * (-m_xij[1] * dHy_dxi[i] - m_xij[2] * dHy_dneta[i]);
		res(2, i) = coefficient *
			(-m_xij[1] * dHx_dxi[i] - m_xij[2] * dHx_dneta[i] + m_yij[1] * dHy_dxi[i] + m_yij[2] * dHy_dneta[i]);
	}

	return res;
}

} // namespace Physics

} // namespace SurgSim
