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
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Physics/LinearSpring.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::OdeState;
using SurgSim::Math::SparseMatrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Physics
{

LinearSpring::LinearSpring(size_t nodeId0, size_t nodeId1) :
	Spring(), m_restLength(-1.0), m_stiffness(-1.0), m_damping(0.0)
{
	m_nodeIds.push_back(nodeId0);
	m_nodeIds.push_back(nodeId1);
}

void LinearSpring::initialize(const OdeState& state)
{
	Spring::initialize(state);

	SURGSIM_ASSERT(m_restLength >= 0.0) << "Spring rest length was not set, please call setRestLength()";
	SURGSIM_ASSERT(m_stiffness >= 0.0) << "Spring stiffness was not set, please call setStiffness()";
}

void LinearSpring::setStiffness(double stiffness)
{
	SURGSIM_ASSERT(stiffness >= 0.0) << "Spring stiffness cannot be negative";
	m_stiffness = stiffness;
}

double LinearSpring::getStiffness() const
{
	return m_stiffness;
}

void LinearSpring::setDamping(double damping)
{
	SURGSIM_ASSERT(damping >= 0.0) << "Spring damping cannot be negative";
	m_damping = damping;
}

double LinearSpring::getDamping() const
{
	return m_damping;
}

void LinearSpring::setRestLength(double restLength)
{
	SURGSIM_ASSERT(restLength >= 0.0) << "Spring rest length cannot be negative";
	m_restLength = restLength;
}

double LinearSpring::getRestLength() const
{
	return m_restLength;
}

void LinearSpring::addForce(const OdeState& state, Vector* F, double scale)
{
	const auto& x0 = state.getPositions().segment<3>(3 * m_nodeIds[0]);
	const auto& x1 = state.getPositions().segment<3>(3 * m_nodeIds[1]);
	const auto& v0 = state.getVelocities().segment<3>(3 * m_nodeIds[0]);
	const auto& v1 = state.getVelocities().segment<3>(3 * m_nodeIds[1]);
	Vector3d u = x1 - x0;
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"Spring (initial length = " << m_restLength << ") became degenerated " <<
				"with 0 length => no force generated";
		return;
	}
	u /= length;
	double elongationPosition = length - m_restLength;
	double elongationVelocity = (v1 - v0).dot(u);
	const Vector3d f = scale * (m_stiffness * elongationPosition + m_damping * elongationVelocity) * u;

	// Assembly stage in F
	F->segment<3>(3 * m_nodeIds[0]) += f;
	F->segment<3>(3 * m_nodeIds[1]) -= f;
}

void LinearSpring::addDamping(const OdeState& state, Math::SparseMatrix* D, double scale)
{
	Matrix33d De;

	// The spring has 2 nodes with positions {x1, x2}, velocities {v1, v2} and force {F1, F2=-F1}
	// Also note from addForce that the positions and velocities play a symmetric role in the force calculation
	// i.e. dFi/dx1 = -dFi/dx2 and dFi/dv1 = -dFi/dv2
	// The damping matrix is D = -dF/dv = (-dF1/dv1 -dF1/dv2) = (-dF1/dv1  dF1/dv1)
	//                                    (-dF2/dv1 -dF2/dv2)   ( dF1/dv1 -dF1/dv1)

	// Let's compute De = -dF1/dv1
	if (!computeDampingAndStiffness(state, &De, nullptr))
	{
		return;
	}
	De *= scale;

	// Assembly stage in D
	Math::addSubMatrixNoInitialize(De, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[0]), D);
	Matrix33d negativeDe = -De;
	Math::addSubMatrixNoInitialize(negativeDe, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[1]), D);
	Math::addSubMatrixNoInitialize(negativeDe, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[0]), D);
	Math::addSubMatrixNoInitialize(De, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[1]), D);
}

void LinearSpring::addStiffness(const OdeState& state, Math::SparseMatrix* K, double scale)
{
	Matrix33d Ke;

	// The spring has 2 nodes with positions {x1, x2}, velocities {v1, v2} and force {F1, F2=-F1}
	// Also note from addForce that the positions and velocities play a symmetric role in the force calculation
	// i.e. dFi/dx1 = -dFi/dx2 and dFi/dv1 = -dFi/dv2
	// The stiffness matrix is K = -dF/dx = (-dF1/dx1 -dF1/dx2) = (-dF1/dx1  dF1/dx1)
	//                                      (-dF2/dx1 -dF2/dx2)   ( dF1/dx1 -dF1/dx1)

	// Let's compute Ke = -dF1/dx1
	if (!computeDampingAndStiffness(state, nullptr, &Ke))
	{
		return;
	}
	Ke *= scale;

	// Assembly stage in K
	Math::addSubMatrixNoInitialize(Ke, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[0]), K);
	Matrix33d negativeKe = -Ke;
	Math::addSubMatrixNoInitialize(negativeKe, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[1]), K);
	Math::addSubMatrixNoInitialize(negativeKe, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[0]), K);
	Math::addSubMatrixNoInitialize(Ke, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[1]), K);
}

void LinearSpring::addFDK(const OdeState& state, Vector* F, Math::SparseMatrix* D, Math::SparseMatrix* K)
{
	Matrix33d De, Ke;

	// Assembly stage in F. Note that the force calculation does not rely on any matrices.
	addForce(state, F);

	// The spring has 2 nodes with positions {x1, x2}, velocities {v1, v2} and force {F1, F2=-F1}
	// Also note from addForce that the positions and velocities play a symmetric role in the force calculation
	// i.e. dFi/dx1 = -dFi/dx2 and dFi/dv1 = -dFi/dv2
	// The stiffness matrix is K = -dF/dx = (-dF1/dx1 -dF1/dx2) = (-dF1/dx1  dF1/dx1)
	//                                      (-dF2/dx1 -dF2/dx2)   ( dF1/dx1 -dF1/dx1)
	// The damping matrix is D = -dF/dv = (-dF1/dv1 -dF1/dv2) = (-dF1/dv1  dF1/dv1)
	//                                    (-dF2/dv1 -dF2/dv2)   ( dF1/dv1 -dF1/dv1)

	// Let's compute De = -dF1/dv1 and Ke = -dF1/dx1
	if (!computeDampingAndStiffness(state, &De, &Ke))
	{
		return;
	}

	// Assembly stage in K
	Math::addSubMatrixNoInitialize(Ke, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[0]), K);
	Matrix33d negativeKe = -Ke;
	Math::addSubMatrixNoInitialize(negativeKe, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[1]), K);
	Math::addSubMatrixNoInitialize(negativeKe, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[0]), K);
	Math::addSubMatrixNoInitialize(Ke, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[1]), K);

	// Assembly stage in D
	Math::addSubMatrixNoInitialize(De, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[0]), D);
	Matrix33d negativeDe = -De;
	Math::addSubMatrixNoInitialize(negativeDe, static_cast<Eigen::Index>(m_nodeIds[0]),
		static_cast<Eigen::Index>(m_nodeIds[1]), D);
	Math::addSubMatrixNoInitialize(negativeDe, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[0]), D);
	Math::addSubMatrixNoInitialize(De, static_cast<Eigen::Index>(m_nodeIds[1]),
		static_cast<Eigen::Index>(m_nodeIds[1]), D);
}

void LinearSpring::addMatVec(const OdeState& state, double alphaD, double alphaK, const Vector& vector, Vector* F)
{
	// Premature return if both factors are zero
	if (alphaK == 0.0 && alphaD == 0.0)
	{
		return;
	}

	Matrix33d De, Ke;
	if (!computeDampingAndStiffness(state, (alphaD != 0 ? &De : nullptr), (alphaK != 0 ? &Ke : nullptr)))
	{
		return;
	}

	// Shared data: the 2x 3D vectors to multiply the matrices with
	const auto& vector1 = vector.segment<3>(3 * m_nodeIds[0]);
	const auto& vector2 = vector.segment<3>(3 * m_nodeIds[1]);

	if (alphaD != 0.0)
	{
		const Vector3d force = alphaD * (De * (vector1 - vector2));
		F->segment<3>(3 * m_nodeIds[0]) += force;
		F->segment<3>(3 * m_nodeIds[1]) -= force;
	}

	if (alphaK != 0.0)
	{
		const Vector3d force = alphaK * (Ke * (vector1 - vector2));
		F->segment<3>(3 * m_nodeIds[0]) += force;
		F->segment<3>(3 * m_nodeIds[1]) -= force;
	}
}

bool LinearSpring::computeDampingAndStiffness(const OdeState& state, Matrix33d* De, Matrix33d* Ke)
{
	const auto& x0 = state.getPositions().segment<3>(3 * m_nodeIds[0]);
	const auto& x1 = state.getPositions().segment<3>(3 * m_nodeIds[1]);
	const auto& v0 = state.getVelocities().segment<3>(3 * m_nodeIds[0]);
	const auto& v1 = state.getVelocities().segment<3>(3 * m_nodeIds[1]);
	Vector3d u = x1 - x0;
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"Spring (initial length = " << m_restLength <<
				") became degenerated with 0 length => force derivative degenerated";
		return false;
	}
	u /= length;
	double lRatio = (length - m_restLength) / length;
	double vRatio = (v1 - v0).dot(u) / length;
	Matrix33d uuT = u * u.transpose();

	// Update the stiffness matrix
	if (Ke != nullptr)
	{
		*Ke = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
		*Ke -= uuT * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
		*Ke += m_damping * (u * (v1 - v0).transpose()) / length;
	}

	// Update the damping matrix
	if (De != nullptr)
	{
		*De = m_damping * uuT;
	}

	return true;
}


bool LinearSpring::operator ==(const Spring& spring) const
{
	const LinearSpring* ls = dynamic_cast<const LinearSpring*>(&spring);
	if (! ls)
	{
		return false;
	}
	return m_nodeIds == ls->m_nodeIds &&
		   m_restLength == ls->m_restLength && m_stiffness == ls->m_stiffness && m_damping == ls->m_damping;
}

bool LinearSpring::operator !=(const Spring& spring) const
{
	return !((*this) == spring);
}

} // namespace Physics

} // namespace SurgSim
