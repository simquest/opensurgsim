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

using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubMatrix;
using SurgSim::Math::getSubVector;

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
	// Initialize the linear tetrahedron element (this computes the linear stiffness matrix)
	Fem3DElementTetrahedron::initialize(state);

	// The co-rotational frame is set to identity at first
	m_rotation.setIdentity();

	// The initial co-rotated stiffness matrix is equal to the local stiffness matrix
	m_corotationalStiffnessMatrix = m_K;

	// Pre-compute the matrix V^-1 with V the matrix of the undeformed tetrahedron points in homogeneous coordinates
	SurgSim::Math::Matrix44d m_V;
	m_V.col(0).segment<3>(0) = state.getPosition(m_nodeIds[0]);
	m_V.col(1).segment<3>(0) = state.getPosition(m_nodeIds[1]);
	m_V.col(2).segment<3>(0) = state.getPosition(m_nodeIds[2]);
	m_V.col(3).segment<3>(0) = state.getPosition(m_nodeIds[3]);
	m_V.row(3).setOnes();
	double determinant;
	bool invertible;
	m_V.computeInverseAndDetWithCheck(m_Vinverse, determinant, invertible);
	SURGSIM_ASSERT(invertible) << "Trying to initialize an invalid co-rotational tetrahedron." <<
		" Matrix V not invertible." << std::endl <<
		"More likely the tetrahedron is degenerated:" << std::endl <<
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
													const SurgSim::Math::Vector& vector, SurgSim::Math::Vector* result)
{
	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 12, 1> vectorLocal, resultLocal;
	getSubVector(vector, m_nodeIds, 3, &vectorLocal);

	// Adds the mass contribution
	if (alphaM != 0.0)
	{
		resultLocal = alphaM * (m_M * vectorLocal);
		addSubVector(resultLocal, m_nodeIds, 3, result);
	}

	// Adds the stiffness contribution
	if (alphaK != 0.0)
	{
		resultLocal = alphaK * (m_corotationalStiffnessMatrix * vectorLocal);
		addSubVector(resultLocal, m_nodeIds, 3, result);
	}
}

bool Fem3DElementCorotationalTetrahedron::update(const SurgSim::Math::OdeState& state)
{
	using SurgSim::Math::makeSkewSymmetricMatrix;
	using SurgSim::Math::skew;
	using SurgSim::Math::Vector3d;

	// The update does two things:
	// 1) Recompute the element's rotation
	// 2) Update the element's stiffness matrix based on the new rotation

	// 1) Recompute the element's rotation
	Eigen::Matrix<double, 12, 1> x;
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);

	// Matrix P is the matrix of the deformed tetrahedron points in homogenous coordinates.
	SurgSim::Math::Matrix44d P;
	P.col(0).segment<3>(0) = getSubVector(x, 0, 3);
	P.col(1).segment<3>(0) = getSubVector(x, 1, 3);
	P.col(2).segment<3>(0) = getSubVector(x, 2, 3);
	P.col(3).segment<3>(0) = getSubVector(x, 3, 3);
	P.row(3).setOnes();

	// Matrix V is the matrix of the undeformed tetrahedron points in homogenous coordinates.
	// F = P.V^1 is an affine transformation (deformation gradient) measuring the transformation
	// between the undeformed and deformed configurations.
	Eigen::Transform<double, 3, Eigen::Affine> F(P * m_Vinverse);
	// F is of the form (B t) where t contains the translation part and B the rotation and stretching.
	//                  (0 1)
	// c.f. "Interactive Virtual Materials", Muller, Gross. Graphics Interface 2004

	// http://en.wikipedia.org/wiki/Polar_decomposition
	// Compute the polar decomposition of F to extract the rotation and the scaling parts.
	SurgSim::Math::Matrix33d scaling;
	F.computeRotationScaling(&m_rotation, &scaling);

	if (std::abs(m_rotation.determinant() - 1.0) > 1e-8)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Rotation has an invalid determinant of " << m_rotation.determinant();
		return false;
	}

	// Build a 12x12 rotation matrix, duplicating the 3x3 rotation on the diagonal blocks
	Eigen::Matrix<double, 12, 12> R12x12 = Eigen::Matrix<double, 12, 12>::Zero();
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		getSubMatrix(R12x12, nodeId, nodeId, 3, 3) = m_rotation;
	}

	// 2) Update the element's stiffness matrix based on the new rotation
	// Compute the exact stiffness matrix K = R.Ke.R^t + [dR/dxl.Ke.(R^t.x-x0]_l + [R.Ke.(dR/dxl)^T.x]_l
	// c.f. "Exact Corotational Linear FEM Stiffness Matrix", Jernej Barbic. Technical Report USC 2012.
	// with
	//   xl the l^th component of the dof vector x
	//   [v]_l the l^th column of a 12x12 matrix defined by the column vector v

	// Here is the rotated element stiffness matrix part
	Eigen::Matrix<double, 12, 12> RK = R12x12 * m_K;
	m_corotationalStiffnessMatrix = RK * R12x12.transpose();

	// Now we compute some useful matrices for the next step
	double determinant;
	bool invertible;
	SurgSim::Math::Matrix33d G, Ginv;
	G = (scaling.trace() * SurgSim::Math::Matrix33d::Identity() - scaling) * m_rotation.transpose();
	G.computeInverseAndDetWithCheck(Ginv, determinant, invertible);
	if (!invertible)
	{
		SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), WARNING) <<
			"Corotational element has a singular G matrix";
		return false;
	}

	// dR/dx = dR/dF . dF/dx with dF/dx constant over time.
	// Here, we compute the various dR/d(F[i][j]).
	std::array<Vector3d, 3> e = {{Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()}};
	std::array<std::array<SurgSim::Math::Matrix33d, 3>, 3> dRdF;
	for (size_t i = 0; i < 3; ++i)
	{
		for (size_t j = 0; j < 3; ++j)
		{
			// Compute wij by solving G.wij = 2.skew(R^t.ei.ej^t)
			Vector3d wij = Ginv * 2.0 * skew((m_rotation.transpose() * (e[i] * e[j].transpose())).eval());

			// Compute dR/dFij = [wij].R
			dRdF[i][j] = makeSkewSymmetricMatrix(wij) * m_rotation;
		}
	}

	// dR/dx = dR/dF . dF/dx
	// Let define the following notation to follow the construction in the paper:
	// A3x3 being a 3x3 matrix, aLaBarbic(A3x3) = (A00 A01 A02 A10 A11 A12 A20 A21 A22)
	// dR/dF becomes a single 9x9 matrix where the 3x3 matrix dR/dFij is stored aLaBarbic as the (3*i+j)th row
	// dF/dx becomes a single 9x12 matrix of the form (n1 0 0 n2 0 0 n3 0 0 n4 0 0)
	//                                                (0 n1 0 0 n2 0 0 n3 0 0 n4 0)
	//                                                (0 0 n1 0 0 n2 0 0 n3 0 0 n4)
	// ni being the first 3 entries of the i^th row of V^1 (m_Vinverse)
	//
	// dR/dx = (aLaBarbic(dRdF00)).(n1x 0 0 n2x 0 0 n3x 0 0 n4x 0 0)
	//         (aLaBarbic(dRdF01)) (n1y 0 0 n2y 0 0 n3y 0 0 n4y 0 0)
	//         (       ...       ) (              ...              )
	//         (aLaBarbic(dRdF22)) (0 0 n1z 0 0 n2z 0 0 n3z 0 0 n4z)
	// dR/dxl is a 3x3 matrix stored as the l^th column of the above resulting matrix
	std::array<SurgSim::Math::Matrix33d, 12> dRdX;
	for(int nodeId = 0; nodeId < 4; ++nodeId)
	{
		Vector3d ni(m_Vinverse.row(nodeId).segment<3>(0));

		for(int axis = 0; axis < 3; ++axis)
		{
			size_t dofId = 3 * nodeId + axis;

			// Let's define the 3x3 matrix dR/d(x[dofId])
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					dRdX[dofId](i, j) = dRdF[i][j].row(axis).dot(ni);
				}
			}
		}
	}

	// Now that we have dR/dx, we can add the 2 extra terms to the stiffness matrix:
	// K += [dR/dxl.Ke.(R^t.x-x0]_l + [R.Ke.(dR/dxl)^T.x]_l
	// with
	//   xl the l^th component of the dof vector x
	//   [v]_l the l^th column of a 12x12 matrix defined by the column vector v
	Eigen::Matrix<double, 12, 1> KTimesRx_x0 = m_K * (R12x12.transpose() * x - m_x0);
	Eigen::Matrix<double, 12, 12> dRdxl12x12 = Eigen::Matrix<double, 12, 12>::Zero();
	for (size_t dofId = 0; dofId < 12; ++dofId)
	{
		for (size_t nodeId = 0; nodeId < 4; ++nodeId)
		{
			getSubMatrix(dRdxl12x12, nodeId, nodeId, 3, 3) = dRdX[dofId];
		}

		m_corotationalStiffnessMatrix.col(dofId) += dRdxl12x12 * KTimesRx_x0 + (RK * dRdxl12x12.transpose()) * x;
	}

	return true;
}

}; // namespace Physics

}; // namespace SurgSim
