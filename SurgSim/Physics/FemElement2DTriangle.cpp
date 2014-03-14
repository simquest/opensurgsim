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
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/FemElement2DTriangle.h"

using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubMatrix;
using SurgSim::Math::getSubVector;
using SurgSim::Math::setSubMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

FemElement2DTriangle::FemElement2DTriangle(std::array<unsigned int, 3> nodeIds)
	: m_G(0.0),
	  m_thickness(0.0),
	  m_restArea(0.0)
{
	// 6 dof per node (x, y, z, thetaX, thetaY, thetaZ)
	setNumDofPerNode(6);

	m_nodeIds.assign(nodeIds.cbegin(), nodeIds.cend());
}

void FemElement2DTriangle::setThickness(double thickness)
{
	SURGSIM_ASSERT(thickness != 0.0) << "The triangle thickness cannot be set to 0";
	SURGSIM_ASSERT(thickness > 0.0) << "The triangle thickness cannot be negative (trying to set it to " << thickness << ")";

	m_thickness = thickness;
}

double FemElement2DTriangle::getThickness() const
{
	return m_thickness;
}

double FemElement2DTriangle::getVolume(const DeformableRepresentationState& state) const
{
	const Vector3d A = state.getPosition(m_nodeIds[0]);
	const Vector3d B = state.getPosition(m_nodeIds[1]);
	const Vector3d C = state.getPosition(m_nodeIds[2]);

	return m_thickness * (B - A).cross(C - A).norm() / 2.0;
}

void FemElement2DTriangle::initialize(const DeformableRepresentationState& state)
{
	// Test the validity of the physical parameters
	FemElement::initialize(state);

	SURGSIM_ASSERT(m_thickness > 0) << "FemElement2DTriangle thickness should be positive and non-zero.  Did you call "
									"setThickness(thickness) ?";

	// Store the rest state for this beam in m_x0
	getSubVector(state.getPositions(), m_nodeIds, 6, &m_x0);

	m_restArea = (m_x0.segment<3>(6) - m_x0.segment<3>(0)).cross((m_x0.segment<3>(12) - m_x0.segment<3>(0))).norm() / 2.0;
	SURGSIM_ASSERT(m_restArea > 0) << "FemElement2DTriangle rest area is zero (degenerate triangle)";

	computeInitialRotation(state);

	// Pre-compute the mass and stiffness matrix
	computeMass(state, &m_M);
	computeStiffness(state, &m_K);
}

void FemElement2DTriangle::addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F, double scale)
{
	Eigen::Matrix<double, 18, 1> x, f;

	// K.U = F_ext
	// K.(x - x0) = F_ext
	// 0 = F_ext + F_int, with F_int = -K.(x - x0)
	getSubVector(state.getPositions(), m_nodeIds, 6, &x);
	f = (-scale) * m_K * (x - m_x0);
	addSubVector(f, m_nodeIds, 6, F);
}

void FemElement2DTriangle::addMass(const DeformableRepresentationState& state, SurgSim::Math::Matrix* M, double scale)
{
	addSubMatrix(m_M * scale, m_nodeIds, 6, M);
}

void FemElement2DTriangle::addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D, double scale)
{
}

void FemElement2DTriangle::addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K, double scale)
{
	addSubMatrix(m_K * scale, getNodeIds(), 6, K);
}

void FemElement2DTriangle::addFMDK(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
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

void FemElement2DTriangle::addMatVec(const DeformableRepresentationState& state, double alphaM, double alphaD,
								 double alphaK, const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	using SurgSim::Math::addSubVector;
	using SurgSim::Math::getSubVector;

	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 18, 1, Eigen::DontAlign> extractedX, extractedResult;
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

void FemElement2DTriangle::computeMass(const DeformableRepresentationState& state,
								   Eigen::Matrix<double, 18, 18, Eigen::DontAlign>* M)
{
	double m = m_restArea * m_thickness * m_rho;

	// XXX TO DO
	m_MLocal.setIdentity();
	m_MLocal *= m;

	// Transformation Local -> Global
	m_M = m_R0.transpose() * m_MLocal * m_R0;
}

void FemElement2DTriangle::computeStiffness(const DeformableRepresentationState& state,
										Eigen::Matrix<double, 18, 18, Eigen::DontAlign>* k)
{
	// General expresson for shear modulus in terms of Young's modulus and Poisson's ratio
	m_G = m_E / (2.0 * (1.0 + m_nu));

	m_KLocal.setZero();
	// XXX TO DO LOOK UP REF From pg XXX of "Theory of Matrix Structural Analysis" from J.S. Przemieniecki:
	// XXX TO DO

	// Transformation Local -> Global
	m_K = m_R0.transpose() * m_KLocal * m_R0;
}

void FemElement2DTriangle::computeInitialRotation(const DeformableRepresentationState& state)
{
	// Build (i, j, k) an orthonormal frame
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

	// Set up a temporary 3x3 initial rotation matrix
	SurgSim::Math::Matrix33d rotation3x3;
	rotation3x3.col(0) = i;
	rotation3x3.col(1) = j;
	rotation3x3.col(2) = k;

	// Set up the 18x18 initial rotation matrix
	m_R0.setZero();
	setSubMatrix(rotation3x3, 0, 0, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 1, 1, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 2, 2, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 3, 3, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 4, 4, 3, 3, &m_R0);
	setSubMatrix(rotation3x3, 5, 5, 3, 3, &m_R0);
}

bool FemElement2DTriangle::isValidCoordinate(const SurgSim::Math::Vector& naturalCoordinate) const
{
	return (std::abs(naturalCoordinate.sum() - 1.0) < SurgSim::Math::Geometry::ScalarEpsilon)
		   && (naturalCoordinate.size() == 3)
		   && (0.0 <= naturalCoordinate.minCoeff() && naturalCoordinate.maxCoeff() <= 1.0);
}

SurgSim::Math::Vector FemElement2DTriangle::computeCartesianCoordinate(const DeformableRepresentationState& state,
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

} // namespace Physics

} // namespace SurgSim
