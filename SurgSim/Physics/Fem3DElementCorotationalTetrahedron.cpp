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
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"

using SurgSim::Math::getSubVector;
using SurgSim::Math::addSubVector;
using SurgSim::Math::addSubMatrix;

namespace SurgSim
{

namespace Physics
{

Fem3DElementCorotationalTetrahedron::Fem3DElementCorotationalTetrahedron(std::array<unsigned int, 4> nodeIds)
	: Fem3DElementTetrahedron(nodeIds)
{
}

void Fem3DElementCorotationalTetrahedron::initialize(const SurgSim::Math::OdeState& state)
{
	// Initialize the linear tetrahedrom element (this computes the linear stiffness matrix)
	Fem3DElementTetrahedron::initialize(state);

	// The co-rotational frame is set to identity at first
	m_rotation.setIdentity();

	// The initial co-rotated stiffness matrix is equal to the local stiffness matrix
	m_corotationalStiffnessMatrix = m_K;

	// Pre-compute the matrix V^-1 with V the matrix of the undeformed tetrahedron points in homogenous coordinates
	SurgSim::Math::Matrix44d m_V = SurgSim::Math::Matrix44d::Ones();
	m_V.col(0).segment<3>(0) = state.getPosition(m_nodeIds[0]);
	m_V.col(1).segment<3>(0) = state.getPosition(m_nodeIds[1]);
	m_V.col(2).segment<3>(0) = state.getPosition(m_nodeIds[2]);
	m_V.col(3).segment<3>(0) = state.getPosition(m_nodeIds[3]);
	double determinant;
	bool invertible;
	m_V.computeInverseAndDetWithCheck(m_Vinverse, determinant, invertible);
	SURGSIM_ASSERT(invertible) << "Trying to initialize an invalid co-rotational tetrahedron." <<
		" Matrix V not invertible." << std::endl <<
		"More likely the tetrahderon is degenerated:" << std::endl <<
		"  A = (" << state.getPosition(m_nodeIds[0]).transpose() << ")" << std::endl <<
		"  B = (" << state.getPosition(m_nodeIds[1]).transpose() << ")" << std::endl <<
		"  C = (" << state.getPosition(m_nodeIds[2]).transpose() << ")" << std::endl <<
		"  D = (" << state.getPosition(m_nodeIds[3]).transpose() << ")" << std::endl;
}

void Fem3DElementCorotationalTetrahedron::addForce(const SurgSim::Math::OdeState& state, SurgSim::Math::Vector* F,
												   double scale)
{
	Eigen::Matrix<double, 12, 1> x, R_x0, f;

	// R.K.(R^-1.x - x0) = Fext
	// 0 = Fext + Fint     with Fint = -R.K.R^-1.(x - R.x0)
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		getSubVector(R_x0, nodeId, 3) = m_rotation * getSubVector(m_x0, nodeId, 3);
	}
	f = (- scale) * m_corotationalStiffnessMatrix * (x - R_x0);
	addSubVector(f, m_nodeIds, 3, F);
}

void Fem3DElementCorotationalTetrahedron::addStiffness(const SurgSim::Math::OdeState& state, SurgSim::Math::Matrix* K,
													   double scale)
{
	addSubMatrix(m_corotationalStiffnessMatrix * scale, getNodeIds(), 3, K);
}

void Fem3DElementCorotationalTetrahedron::addMatVec(const SurgSim::Math::OdeState& state,
										double alphaM, double alphaD, double alphaK,
										const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 12, 1, Eigen::DontAlign> xLoc, resLoc;
	getSubVector(x, m_nodeIds, 3, &xLoc);

	// Adds the mass contribution
	if (alphaM)
	{
		resLoc = alphaM * (m_M * xLoc);
		addSubVector(resLoc, m_nodeIds, 3, F);
	}

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK)
	{
		resLoc = alphaK * (m_corotationalStiffnessMatrix * xLoc);
		addSubVector(resLoc, m_nodeIds, 3, F);
	}
}

bool Fem3DElementCorotationalTetrahedron::update(const SurgSim::Math::OdeState& state)
{
	// The update does two things:
	// 1) Recompute the element's rotation
	// 2) Update the element's stiffness matrix based on the new rotation

	// Matrix P is the matrix of the deformed tetrahedron points in homogenous coordinates.
	SurgSim::Math::Matrix44d P;
	P.col(0).segment<3>(0) = state.getPosition(m_nodeIds[0]);
	P.col(1).segment<3>(0) = state.getPosition(m_nodeIds[1]);
	P.col(2).segment<3>(0) = state.getPosition(m_nodeIds[2]);
	P.col(3).segment<3>(0) = state.getPosition(m_nodeIds[3]);
	P.row(3).setOnes();

	// Matrix V is the matrix of the undeformed tetrahedron points in homogenous coordinates.
	// F = P.V^1 is an affine transformation (deformation gradient) measuring the transformation
	// between the undeformed and deformed configurations.
	Eigen::Transform<double, 3, Eigen::Affine> F(P * m_Vinverse);
	// F is of the form (B t) where t contains the translation part and B the rotation and stretching.
	//                  (0 1)
	// c.f. "Interactive Virtual Materials", Muller, Gross. Graphics Interface 2004

	// http://en.wikipedia.org/wiki/Polar_decomposition
	// The polar decomposition of F (=RS) gives R an orthonormal matrix and S a symmetric matrix.
	// The polar decomposition of F extracts a rotation R and a stretching (or scaling) S.
	// The polar decomposition always exists, moreover is unique if F is invertible.
	// The polar decomposition can be deduced from the SVD decomposition F = U.D.V^t => R = U.V^t and S = V.D.V^t
	SurgSim::Math::Matrix33d scaling;
	F.computeRotationScaling(&m_rotation, &scaling);

	SURGSIM_ASSERT(F.linear().isApprox(m_rotation * scaling)) <<
		"Deformation gradient polar decomposition failed F != Rotation.Scaling";

	if (std::abs(m_rotation.determinant() - 1.0) > 1e-8)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Rotation has an invalida determinant of " << m_rotation.determinant();
		return false;
	}

	// Build a 12x12 rotation matrix, useful of the stiffness matrix computation
	Eigen::Matrix<double, 12, 12> R12x12 = Eigen::Matrix<double, 12, 12>::Zero();
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		R12x12.block<3, 3>(3 * nodeId, 3 * nodeId) = m_rotation;
	}

	// Compute the rotated stiffness matrix K = R.Ke.R^t
	// c.f. "Interactive Virtual Materials", Muller, Gross. Graphics Interface 2004
	m_corotationalStiffnessMatrix = R12x12 * m_K * R12x12.transpose();

	return true;
}

} // namespace Physics

} // namespace SurgSim
