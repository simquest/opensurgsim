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

Fem3DElementCorotationalTetrahedron::Fem3DElementCorotationalTetrahedron(std::array<size_t, 4> nodeIds)
	: Fem3DElementTetrahedron(nodeIds)
{
}

void Fem3DElementCorotationalTetrahedron::initialize(const SurgSim::Math::OdeState& state)
{
	// Initialize the linear tetrahedron element (this computes the linear stiffness matrix)
	Fem3DElementTetrahedron::initialize(state);

	// Pre-compute the matrix V^-1 with V the matrix of the undeformed tetrahedron points in homogeneous coordinates
	SurgSim::Math::Matrix44d V;
	V.col(0).segment<3>(0) = state.getPosition(m_nodeIds[0]);
	V.col(1).segment<3>(0) = state.getPosition(m_nodeIds[1]);
	V.col(2).segment<3>(0) = state.getPosition(m_nodeIds[2]);
	V.col(3).segment<3>(0) = state.getPosition(m_nodeIds[3]);
	V.row(3).setOnes();
	double determinant;
	bool invertible;
	V.computeInverseAndDetWithCheck(m_Vinverse, determinant, invertible);
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
	Eigen::Matrix<double, 12, 12> RKRt;
	Eigen::Matrix<double, 12, 1> x, R_x0, f;
	SurgSim::Math::Matrix33d R;

	computeRotationMassAndStiffness(state, &R, nullptr, &RKRt);

	// R.K.(R^-1.x - x0) = Fext
	// 0 = Fext + Fint     with Fint = -R.K.R^-1.(x - R.x0)
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		getSubVector(R_x0, nodeId, 3) = R * getSubVector(m_x0, nodeId, 3);
	}
	f = -scale * (RKRt * (x - R_x0));
	addSubVector(f, m_nodeIds, 3, F);
}

void Fem3DElementCorotationalTetrahedron::addStiffness(const SurgSim::Math::OdeState& state,
		SurgSim::Math::SparseMatrix* K,
		double scale /*= 1.0*/)
{
	Eigen::Matrix<double, 12, 12> RKRt;

	computeRotationMassAndStiffness(state, nullptr, nullptr, &RKRt);

	Math::Matrix scaledDense(12, 12);
	scaledDense = RKRt * scale;
	addSubMatrix(scaledDense, getNodeIds(), 3, K);
}

void Fem3DElementCorotationalTetrahedron::addMass(const SurgSim::Math::OdeState& state, SurgSim::Math::SparseMatrix* M,
		double scale /*= 1.0*/)
{
	Eigen::Matrix<double, 12, 12> RMRt;

	computeRotationMassAndStiffness(state, nullptr, &RMRt, nullptr);

	Math::Matrix scaledDense(12, 12);
	scaledDense = RMRt * scale;
	addSubMatrix(scaledDense, getNodeIds(), 3, M);
}

void Fem3DElementCorotationalTetrahedron::addFMDK(const SurgSim::Math::OdeState& state,
		SurgSim::Math::Vector* F,
		SurgSim::Math::SparseMatrix* M,
		SurgSim::Math::SparseMatrix* D,
		SurgSim::Math::SparseMatrix* K)
{
	Eigen::Matrix<double, 12, 12> RMRt, RKRt;
	Eigen::Matrix<double, 12, 1> x, R_x0, f;
	SurgSim::Math::Matrix33d R;

	computeRotationMassAndStiffness(state, &R, &RMRt, &RKRt);

	// Assemble the force Fint = -R.K.R^-1.(x - R.x0)
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);
	for (size_t nodeId = 0; nodeId < 4; ++nodeId)
	{
		getSubVector(R_x0, nodeId, 3) = R * getSubVector(m_x0, nodeId, 3);
	}
	f = -RKRt * (x - R_x0);
	addSubVector(f, m_nodeIds, 3, F);

	// Assemble the stiffness matrix
	addSubMatrix(RKRt, getNodeIds(), 3, K);

	// Assemble the mass matrix
	addSubMatrix(RMRt, getNodeIds(), 3, M);
}

void Fem3DElementCorotationalTetrahedron::addMatVec(const SurgSim::Math::OdeState& state,
		double alphaM, double alphaD, double alphaK,
		const SurgSim::Math::Vector& vector, SurgSim::Math::Vector* result)
{
	if (alphaM == 0.0 && alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 12, 12> RMRt, RKRt;
	computeRotationMassAndStiffness(state, nullptr, (alphaM != 0 ? &RMRt : nullptr), (alphaK != 0 ? &RKRt : nullptr));

	Eigen::Matrix<double, 12, 1> vectorLocal, resultLocal = Eigen::Matrix<double, 12, 1>::Zero();
	getSubVector(vector, m_nodeIds, 3, &vectorLocal);

	// Adds the mass contribution
	if (alphaM != 0.0)
	{
		resultLocal += alphaM * (RMRt * vectorLocal);
	}

	// Adds the stiffness contribution
	if (alphaK != 0.0)
	{
		resultLocal += alphaK * (RKRt * vectorLocal);
	}

	// Assembly in the global vector
	addSubVector(resultLocal, m_nodeIds, 3, result);
}

void Fem3DElementCorotationalTetrahedron::computeRotationMassAndStiffness(const SurgSim::Math::OdeState& state,
		SurgSim::Math::Matrix33d* rotation,
		Eigen::Matrix<double, 12, 12>* Me,
		Eigen::Matrix<double, 12, 12>* Ke) const
{
	using SurgSim::Math::makeSkewSymmetricMatrix;
	using SurgSim::Math::skew;
	using SurgSim::Math::Vector3d;

	SurgSim::Math::Matrix33d R;

	// This method does two things:
	// 1) Recompute the element's rotation R
	// 2) Update the element's stiffness matrix based on the new rotation

	// 1) Recompute the element's rotation R
	Eigen::Matrix<double, 12, 1> x;
	getSubVector(state.getPositions(), m_nodeIds, 3, &x);

	// Matrix P is the matrix of the deformed tetrahedron points in homogenous coordinates.
	SurgSim::Math::Matrix44d P;
	P.col(0).segment<3>(0) = getSubVector(x, 0, 3);
	P.col(1).segment<3>(0) = getSubVector(x, 1, 3);
	P.col(2).segment<3>(0) = getSubVector(x, 2, 3);
	P.col(3).segment<3>(0) = getSubVector(x, 3, 3);
	P.row(3).setOnes();

	// Matrix V is the matrix of the undeformed tetrahedron points in homogeneous coordinates.
	// F = P.V^1 is an affine transformation (deformation gradient) measuring the transformation
	// between the undeformed and deformed configurations.
	Eigen::Transform<double, 3, Eigen::Affine> F(P * m_Vinverse);
	// F is of the form (B t) where t contains the translation part and B the rotation and stretching.
	//                  (0 1)
	// c.f. "Interactive Virtual Materials", Muller, Gross. Graphics Interface 2004

	// http://en.wikipedia.org/wiki/Polar_decomposition
	// Compute the polar decomposition of F to extract the rotation and the scaling parts.
	SurgSim::Math::Matrix33d scaling;
	F.computeRotationScaling(&R, &scaling);

	if (rotation != nullptr)
	{
		*rotation = std::move(R);
		R = *rotation;
	}

	if (std::abs(R.determinant() - 1.0) > 1e-8)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"Rotation has an invalid determinant of " << R.determinant();
		return;
	}

	Eigen::Matrix<double, 12, 12> R12x12;
	if (Me != nullptr || Ke != nullptr)
	{
		// Build a 12x12 rotation matrix, duplicating the 3x3 rotation on the diagonal blocks
		// This is useful only in the mass and stiffness matrices calculation.
		R12x12.setZero();
		for (size_t nodeId = 0; nodeId < 4; ++nodeId)
		{
			getSubMatrix(R12x12, nodeId, nodeId, 3, 3) = R;
		}
	}

	// 2) Update the element's mass matrix based on the new rotation
	// M = R.Me.R^t
	if (Me != nullptr)
	{
		Eigen::Matrix<double, 12, 12> RM = R12x12 * m_M;
		*Me = RM * R12x12.transpose();
	}

	// 3) Update the element's stiffness matrix based on the new rotation
	// F = -RKe(R^t.x - x0)   with Ke the element linear stiffness matrix, R the 12x12 rotation matrix
	// K = -dF/dx
	// K = sum[dR/dxl.Ke.(R^t.x-x0)]_l + R.Ke.R^t + [R.Ke.(dR/dxl)^T.x]_l
	// c.f. "Exact Corotational Linear FEM Stiffness Matrix", Jernej Barbic. Technical Report USC 2012.
	// with
	//   xl the l^th component of the dof vector x
	//   [v(l)]_l is a 12x12 matrix whose l^th column is defined by the column vector v(l)
	// Each column of the 2 matrices [dR/dxl.Ke.(R^t.x-x0)]_l + [R.Ke.(dR/dxl)^T.x]_l
	// can be calculated independently by differentiating R with respect to x, so the entire matrices are defined
	// with a sum over the 12 degrees of freedom, to define each column one by one.
	if (Ke != nullptr)
	{
		// Here is the rotated element stiffness matrix part
		Eigen::Matrix<double, 12, 12> RK = R12x12 * m_K;
		*Ke = RK * R12x12.transpose();

		// Now we compute some useful matrices for the next step
		double determinant;
		bool invertible;
		SurgSim::Math::Matrix33d G, Ginv;
		G = (scaling.trace() * SurgSim::Math::Matrix33d::Identity() - scaling) * R.transpose();
		G.computeInverseAndDetWithCheck(Ginv, determinant, invertible);
		if (!invertible)
		{
			SURGSIM_LOG(SurgSim::Framework::Logger::getDefaultLogger(), WARNING) <<
					"Corotational element has a singular G matrix";
			return;
		}

		// dR/dx = dR/dF . dF/dx with dF/dx constant over time.
		// Here, we compute the various dR/d(F[i][j]).
		std::array<std::array<SurgSim::Math::Matrix33d, 3>, 3> dRdF;
		for (size_t i = 0; i < 3; ++i)
		{
			for (size_t j = 0; j < 3; ++j)
			{
				// Compute wij, the vector solution of G.wij = 2.skew(R^t.ei.ej^t)
				// wij is a rotation vector by nature
				Vector3d wij = Ginv * 2.0 * skew((R.transpose() *
												  (Vector3d::Unit(i) * Vector3d::Unit(j).transpose())).eval());

				// Compute dR/dFij = [wij].R
				dRdF[i][j] = makeSkewSymmetricMatrix(wij) * R;
			}
		}

		// dR/dx = dR/dF . dF/dx
		// dF/dx is block sparse and constant over time, so we develop the matrix multiplication to avoid unecessary
		// calculation. Nevertheless, to allow the user to follow this calculation, we relate it to the notation in
		// the paper:
		// A3x3 being a 3x3 matrix, asVector(A3x3) = (A00 A01 A02 A10 A11 A12 A20 A21 A22)
		// dR/dF becomes a single 9x9 matrix where the 3x3 matrix dR/dFij is stored asVector in the (3*i+j)th row
		// dF/dx becomes a single 9x12 matrix of the form (n1 0 0 n2 0 0 n3 0 0 n4 0 0)
		//                                                (0 n1 0 0 n2 0 0 n3 0 0 n4 0)
		//                                                (0 0 n1 0 0 n2 0 0 n3 0 0 n4)
		// ni being the first 3 entries of the i^th row of V^1 (m_Vinverse)
		//
		// dR/dx = (asVector(dRdF00)).(n1x 0 0 n2x 0 0 n3x 0 0 n4x 0 0)
		//         (asVector(dRdF01)) (n1y 0 0 n2y 0 0 n3y 0 0 n4y 0 0)
		//         (       ...       ) (              ...              )
		//         (asVector(dRdF22)) (0 0 n1z 0 0 n2z 0 0 n3z 0 0 n4z)
		// dR/dxl is a 3x3 matrix stored asVector in the l^th column of the above resulting matrix
		std::array<SurgSim::Math::Matrix33d, 12> dRdX;
		for (int nodeId = 0; nodeId < 4; ++nodeId)
		{
			Vector3d ni(m_Vinverse.row(nodeId).segment<3>(0));

			for (int axis = 0; axis < 3; ++axis)
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
		//   [v(l)]_l a 12x12 matrix whose l^th column is defined by the column vector v(l)
		Eigen::Matrix<double, 12, 1> KTimesRx_x0 = m_K * (R12x12.transpose() * x - m_x0);
		Eigen::Matrix<double, 12, 12> dRdxl12x12 = Eigen::Matrix<double, 12, 12>::Zero();
		for (size_t dofId = 0; dofId < 12; ++dofId)
		{
			for (size_t nodeId = 0; nodeId < 4; ++nodeId)
			{
				getSubMatrix(dRdxl12x12, nodeId, nodeId, 3, 3) = dRdX[dofId];
			}

			Ke->col(dofId) += dRdxl12x12 * KTimesRx_x0 + (RK * dRdxl12x12.transpose()) * x;
		}
	}
}

}; // namespace Physics

}; // namespace SurgSim
