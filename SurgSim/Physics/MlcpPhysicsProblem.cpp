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

#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

MlcpPhysicsProblem::~MlcpPhysicsProblem()
{
}


void MlcpPhysicsProblem::updateConstraint(
	const Eigen::SparseVector<double> &newSubH,
	const Eigen::MatrixXd &subC,
	size_t indexSubC,
	size_t indexNewSubH)
{
	using SurgSim::Math::Vector;

	// Update H, CHt, and HCHt with newSubH, denoted H'.
	//
	// Note that updates are linear for H and CHt, but not for HCHt:
	// (H+H') = H+H'
	// => H += H';
	//
	// C(H+H')t = CHt + CH't
	// => CHt += CH't;
	//
	// (H+H')C(H+H')t = HCHt + HCH't + H'C(H+H')t
	// => HCHt += H(CH't) + H'[C(H+H')t];

	Vector newCHt = subC * newSubH;
	A.col(indexNewSubH) += H.middleCols(indexSubC, subC.rows()) * newCHt;
	H.block(indexNewSubH, indexSubC, 1, subC.rows()) += newSubH.transpose();
	CHt.block(indexSubC, indexNewSubH, subC.rows(), 1) += newCHt;
	A.row(indexNewSubH) += newSubH.transpose() * CHt.middleRows(indexSubC, subC.rows());
}

void MlcpPhysicsProblem::setZero(int numDof, int numConstraintDof, int numConstraints)
{
	MlcpProblem::setZero(numDof, numConstraintDof, numConstraints);

	H.resize(numConstraintDof, numDof);
	H.setZero();
	CHt.resize(numDof, numConstraintDof);
	CHt.setZero();
}

MlcpPhysicsProblem MlcpPhysicsProblem::Zero(int numDof, int numConstraintDof, int numConstraints)
{
	MlcpPhysicsProblem result;
	result.setZero(numDof, numConstraintDof, numConstraints);

	return result;
}

}; // namespace Physics

}; // namespace SurgSim
