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

#include <SurgSim/Physics/DeformableRepresentationState.h>
#include <SurgSim/Physics/LinearSpring.h>

namespace SurgSim
{

namespace Physics
{

LinearSpring::LinearSpring(unsigned int nodeId0, unsigned int nodeId1) :
	Spring(), m_l0(0.0), m_stiffness(0.0), m_damping(0.0)
{
	m_nodeIds.push_back(nodeId0);
	m_nodeIds.push_back(nodeId1);

	m_f.resize(6);
	m_f.setZero();

	m_K.resize(6, 6);
	m_K.setZero();

	m_D.resize(6, 6);
	m_D.setZero();
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

void LinearSpring::setInitialLength(double l0)
{
	m_l0 = l0;
}

double LinearSpring::getInitialLength() const
{
	return m_l0;
}

const SurgSim::Math::Vector& LinearSpring::computeForce(const DeformableRepresentationState& state)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::setSubVector;

	const SurgSim::Math::Vector& x = state.getPositions();
	const SurgSim::Math::Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const SurgSim::Math::Vector& x1 = getSubVector(x, m_nodeIds[1], 3);

	Vector3d f = x1 - x0;
	double m_l = f.norm();
	f *= (m_l - m_l0)/m_l * m_stiffness;

	SurgSim::Math::setSubVector( f, 0, 3, &m_f);
	SurgSim::Math::setSubVector(-f, 1, 3, &m_f);

	return m_f;
}

const SurgSim::Math::Matrix& LinearSpring::computeStiffness(const DeformableRepresentationState& state)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::setSubMatrix;

	const SurgSim::Math::Vector& x = state.getPositions();
	const SurgSim::Math::Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const SurgSim::Math::Vector& x1 = getSubVector(x, m_nodeIds[1], 3);
	SurgSim::Math::Vector3d u = x1 - x0;
	double m_l = u.norm();
	double lRatio = (m_l - m_l0) / m_l;
	u /= m_l;

	SurgSim::Math::Matrix33d K00 = SurgSim::Math::Matrix33d::Identity() * (m_stiffness * lRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0));
	setSubMatrix( K00, 0, 0, 3, 3, &m_K);
	setSubMatrix(-K00, 0, 1, 3, 3, &m_K);
	setSubMatrix(-K00, 1, 0, 3, 3, &m_K);
	setSubMatrix( K00, 1, 1, 3, 3, &m_K);

	return m_K;
}

const SurgSim::Math::Matrix& LinearSpring::computeDamping(const DeformableRepresentationState& state)
{
	return m_D;
}

void LinearSpring::computeFDK(const DeformableRepresentationState& state,
	SurgSim::Math::Vector** f, SurgSim::Math::Matrix** D, SurgSim::Math::Matrix** K)
{
	using SurgSim::Math::getSubVector;
	using SurgSim::Math::setSubVector;
	using SurgSim::Math::setSubMatrix;

	const SurgSim::Math::Vector& x = state.getPositions();
	const SurgSim::Math::Vector& x0 = getSubVector(x, m_nodeIds[0], 3);
	const SurgSim::Math::Vector& x1 = getSubVector(x, m_nodeIds[1], 3);
	SurgSim::Math::Vector3d u = x1 - x0;
	double m_l = u.norm();
	double lRatio = (m_l - m_l0) / m_l;
	u /= m_l;

	SurgSim::Math::Matrix33d K00 = SurgSim::Math::Matrix33d::Identity() * (m_stiffness * lRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0));
	setSubMatrix( K00, 0, 0, 3, 3, &m_K);
	setSubMatrix(-K00, 0, 1, 3, 3, &m_K);
	setSubMatrix(-K00, 1, 0, 3, 3, &m_K);
	setSubMatrix( K00, 1, 1, 3, 3, &m_K);
	*K = &m_K;

	u *= (m_l - m_l0) * m_stiffness;
	setSubVector( u, 0, 3, &m_f);
	setSubVector(-u, 1, 3, &m_f);
	*f = &m_f;

	*D = &m_D;
}

bool LinearSpring::operator ==(const Spring& spring) const
{
	const LinearSpring *ls = dynamic_cast<const LinearSpring*>(&spring);
	if (! ls)
	{
		return false;
	}
	return m_nodeIds == ls->m_nodeIds &&
		m_l0 == ls->m_l0 && m_stiffness == ls->m_stiffness && m_damping == ls->m_damping;
}

bool LinearSpring::operator !=(const Spring& spring) const
{
	return ! ((*this) == spring);
}

} // namespace Physics

} // namespace SurgSim
