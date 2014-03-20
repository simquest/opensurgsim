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

#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/LinearSpring.h"

using SurgSim::Math::Matrix33d;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;
using SurgSim::Math::addSubMatrix;
using SurgSim::Math::addSubVector;
using SurgSim::Math::getSubVector;

namespace SurgSim
{

namespace Physics
{

LinearSpring::LinearSpring(unsigned int nodeId0, unsigned int nodeId1) :
	Spring(), m_restLength(0.0), m_stiffness(0.0), m_damping(0.0)
{
	m_nodeIds.push_back(nodeId0);
	m_nodeIds.push_back(nodeId1);
}

void LinearSpring::setStiffness(double stiffness)
{
	m_stiffness = stiffness;
}

double LinearSpring::getStiffness() const
{
	return m_stiffness;
}

void LinearSpring::setDamping(double damping)
{
	m_damping = damping;
}

double LinearSpring::getDamping() const
{
	return m_damping;
}

void LinearSpring::setRestLength(double restLength)
{
	m_restLength = restLength;
}

double LinearSpring::getRestLength() const
{
	return m_restLength;
}

void LinearSpring::addForce(const DeformableRepresentationState& state, SurgSim::Math::Vector* F, double scale)
{
	const Vector& x = state.getPositions();
	const Vector& v = state.getVelocities();
	const Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const Vector& x1 = getSubVector(x, m_nodeIds[1], 3);
	const Vector& v0 = getSubVector(v, m_nodeIds[0], 3);
	const Vector& v1 = getSubVector(v, m_nodeIds[1], 3);

	Vector3d u = x1 - x0;
	double m_l = u.norm();
	u /= m_l;
	double elongationPosition = m_l - m_restLength;
	double elongationVelocity = (v1 - v0).dot(u);
	Vector3d f = scale * (m_stiffness* elongationPosition + m_damping * elongationVelocity) * u;

	// Assembly stage in F
	addSubVector( f, m_nodeIds[0], 3, F);
	addSubVector(-f, m_nodeIds[1], 3, F);
}

void LinearSpring::addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D, double scale)
{
	const Vector& x = state.getPositions();
	Vector3d u = getSubVector(x, m_nodeIds[1], 3) - getSubVector(x, m_nodeIds[0], 3);
	u.normalize();
	Matrix33d D00 = scale * m_damping * (u * u.transpose());

	// Assembly stage in D
	addSubMatrix( D00, m_nodeIds[0], m_nodeIds[0], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[0], m_nodeIds[1], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[1], m_nodeIds[0], 3, 3, D);
	addSubMatrix( D00, m_nodeIds[1], m_nodeIds[1], 3, 3, D);
}

void LinearSpring::addStiffness(const DeformableRepresentationState& state, SurgSim::Math::Matrix* K, double scale)
{
	const Vector& x = state.getPositions();
	const Vector& v = state.getVelocities();
	const Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const Vector& x1 = getSubVector(x, m_nodeIds[1], 3);
	const Vector& v0 = getSubVector(v, m_nodeIds[0], 3);
	const Vector& v1 = getSubVector(v, m_nodeIds[1], 3);
	Vector3d u = x1 - x0;
	double m_l = u.norm();
	u /= m_l;
	double lRatio = (m_l - m_restLength) / m_l;
	double vRatio = (v1 -v0).dot(u) / m_l;

	Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
	K00 += m_damping * (u * (v1 -v0).transpose()) / m_l;
	K00 *= scale;

	// Assembly stage in K
	addSubMatrix( K00, m_nodeIds[0], m_nodeIds[0], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[0], m_nodeIds[1], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[1], m_nodeIds[0], 3, 3, K);
	addSubMatrix( K00, m_nodeIds[1], m_nodeIds[1], 3, 3, K);
}

void LinearSpring::addFDK(const DeformableRepresentationState& state, SurgSim::Math::Vector* F,
							   SurgSim::Math::Matrix* D, SurgSim::Math::Matrix* K)
{
	const Vector& x = state.getPositions();
	const Vector& v = state.getVelocities();
	const Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const Vector& x1 = getSubVector(x, m_nodeIds[1], 3);
	const Vector& v0 = getSubVector(v, m_nodeIds[0], 3);
	const Vector& v1 = getSubVector(v, m_nodeIds[1], 3);
	Vector3d u = x1 - x0;
	double m_l = u.norm();
	u /= m_l;
	double elongationPosition = m_l - m_restLength;
	double elongationVelocity = (v1 - v0).dot(u);
	double lRatio = elongationPosition / m_l;
	double vRatio = (v1 -v0).dot(u) / m_l;

	Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
	K00 += m_damping * (u * (v1 -v0).transpose()) / m_l;
	addSubMatrix( K00, m_nodeIds[0], m_nodeIds[0], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[0], m_nodeIds[1], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[1], m_nodeIds[0], 3, 3, K);
	addSubMatrix( K00, m_nodeIds[1], m_nodeIds[1], 3, 3, K);

	Matrix33d D00 = m_damping * (u * u.transpose());
	addSubMatrix( D00, m_nodeIds[0], m_nodeIds[0], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[0], m_nodeIds[1], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[1], m_nodeIds[0], 3, 3, D);
	addSubMatrix( D00, m_nodeIds[1], m_nodeIds[1], 3, 3, D);

	u *= (m_stiffness* elongationPosition + m_damping * elongationVelocity);
	addSubVector( u, m_nodeIds[0], 3, F);
	addSubVector(-u, m_nodeIds[1], 3, F);
}

void LinearSpring::addMatVec(const DeformableRepresentationState& state, double alphaD, double alphaK,
							 const SurgSim::Math::Vector& x, SurgSim::Math::Vector* F)
{
	// Considering that we do not have damping yet, only the stiffness part will contribute
	if (alphaK == 0.0)
	{
		return;
	}

	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> x6D;
	getSubVector(x, m_nodeIds, 3, &x6D);

	// Adds the damping contribution (No damping)

	// Adds the stiffness contribution
	if (alphaK != 0.0)
	{
		const Vector& xState = state.getPositions();
		const Vector& x0 = getSubVector(xState, m_nodeIds[0], 3);
		const Vector& x1 = getSubVector(xState, m_nodeIds[1], 3);
		Vector3d u = x1 - x0;
		double m_l = u.norm();
		double lRatio = (m_l - m_restLength) / m_l;
		u /= m_l;

		Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio);
		K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0));

		Vector3d force = alphaK * (K00 * (x6D.segment(0, 3) - x6D.segment(3, 3)));
		addSubVector( force, m_nodeIds[0], 3, F);
		addSubVector(-force, m_nodeIds[1], 3, F);
	}
}

bool LinearSpring::operator ==(const Spring& spring) const
{
	const LinearSpring *ls = dynamic_cast<const LinearSpring*>(&spring);
	if (! ls)
	{
		return false;
	}
	return m_nodeIds == ls->m_nodeIds &&
		m_restLength == ls->m_restLength && m_stiffness == ls->m_stiffness && m_damping == ls->m_damping;
}

bool LinearSpring::operator !=(const Spring& spring) const
{
	return ! ((*this) == spring);
}

} // namespace Physics

} // namespace SurgSim
