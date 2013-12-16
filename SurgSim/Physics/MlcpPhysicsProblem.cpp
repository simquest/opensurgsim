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
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

namespace Physics
{

void MlcpPhysicsProblem::updateConstraints(
	const Eigen::SparseVector<double> &newH,
	const Eigen::MatrixXd &subC,
	size_t indexSubC,
	size_t colNewH)
{
	using SurgSim::Math::Vector;

	Matrix& m_H = H;
	Matrix& m_CHt = CHt;
	Matrix& m_HCHt = A;

	// Update H, CHt, and HCHt with newH, denoted H'.
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

	Vector newCHt = subC * newH;
	m_HCHt.col(colNewH) += m_H.middleCols(indexSubC, subC.rows()) * newCHt;
	m_H.block(colNewH, indexSubC, 1, subC.rows()) += newH.transpose();
	m_CHt.block(indexSubC, colNewH, subC.rows(), 1) += newCHt;
	m_HCHt.row(colNewH) += newH.transpose() * m_CHt.middleRows(indexSubC, subC.rows());
}

}; // namespace Physics

}; // namespace SurgSim
