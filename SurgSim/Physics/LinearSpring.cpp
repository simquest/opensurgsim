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
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Math/Geometry.h"

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
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Spring (initial length = " << m_restLength << ") became degenerated with 0 length => no force generated";
		return;
	}
	u /= length;
	double elongationPosition = length - m_restLength;
	double elongationVelocity = (v1 - v0).dot(u);
	const Vector3d f = scale * (m_stiffness* elongationPosition + m_damping * elongationVelocity) * u;

	// Assembly stage in F
	addSubVector( f, m_nodeIds[0], 3, F);
	addSubVector(-f, m_nodeIds[1], 3, F);
}

void LinearSpring::addDamping(const DeformableRepresentationState& state, SurgSim::Math::Matrix* D, double scale)
{
	const Vector& x = state.getPositions();
	Vector3d u = getSubVector(x, m_nodeIds[1], 3) - getSubVector(x, m_nodeIds[0], 3);
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Spring (initial length = " << m_restLength << ") became degenerated with 0 length => no force generated";
		return;
	}
	u /= length;
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
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Spring (initial length = " << m_restLength << ") became degenerated with 0 length => no force generated";
		return;
	}
	u /= length;
	double lRatio = (length - m_restLength) / length;
	double vRatio = (v1 -v0).dot(u) / length;

	Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
	K00 += m_damping * (u * (v1 -v0).transpose()) / length;
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
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Spring (initial length = " << m_restLength << ") became degenerated with 0 length => no force generated";
		return;
	}
	u /= length;
	double elongationPosition = length - m_restLength;
	double elongationVelocity = (v1 - v0).dot(u);
	double lRatio = elongationPosition / length;
	double vRatio = (v1 -v0).dot(u) / length;

	Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
	K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
	K00 += m_damping * (u * (v1 -v0).transpose()) / length;
	addSubMatrix( K00, m_nodeIds[0], m_nodeIds[0], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[0], m_nodeIds[1], 3, 3, K);
	addSubMatrix(-K00, m_nodeIds[1], m_nodeIds[0], 3, 3, K);
	addSubMatrix( K00, m_nodeIds[1], m_nodeIds[1], 3, 3, K);

	Matrix33d D00 = m_damping * (u * u.transpose());
	addSubMatrix( D00, m_nodeIds[0], m_nodeIds[0], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[0], m_nodeIds[1], 3, 3, D);
	addSubMatrix(-D00, m_nodeIds[1], m_nodeIds[0], 3, 3, D);
	addSubMatrix( D00, m_nodeIds[1], m_nodeIds[1], 3, 3, D);

	const Vector3d f = u * (m_stiffness* elongationPosition + m_damping * elongationVelocity);
	addSubVector( f, m_nodeIds[0], 3, F);
	addSubVector(-f, m_nodeIds[1], 3, F);
}

void LinearSpring::addMatVec(const DeformableRepresentationState& state, double alphaD, double alphaK,
							 const SurgSim::Math::Vector& vector, SurgSim::Math::Vector* F)
{
	// Premature return if both factors are zero
	if (alphaK == 0.0 && alphaD == 0.0)
	{
		return;
	}

	// Shared data: the 6D vector to multiply the 6x6 matrix with
	Eigen::Matrix<double, 6, 1, Eigen::DontAlign> vector6D;
	getSubVector(vector, m_nodeIds, 3, &vector6D);

	// Shared data: spring direction and length
	const Vector& xState = state.getPositions();
	const Vector& x0 = getSubVector(xState, m_nodeIds[0], 3);
	const Vector& x1 = getSubVector(xState, m_nodeIds[1], 3);
	Vector3d u = x1 - x0;
	double length = u.norm();
	if (length < SurgSim::Math::Geometry::DistanceEpsilon)
	{
		SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Spring (initial length = " << m_restLength << ") became degenerated with 0 length => no force generated";
		return;
	}
	u /= length;

	if (alphaD != 0.0)
	{
		Matrix33d D00 = m_damping * (u * u.transpose());

		const Vector3d force = alphaD * (D00 * (vector6D.segment(0, 3) - vector6D.segment(3, 3)));
		addSubVector( force, m_nodeIds[0], 3, F);
		addSubVector(-force, m_nodeIds[1], 3, F);
	}

	if (alphaK != 0.0)
	{
		const Vector& v = state.getVelocities();
		const Vector& v0 = getSubVector(v, m_nodeIds[0], 3);
		const Vector& v1 = getSubVector(v, m_nodeIds[1], 3);

		double elongationPosition = length - m_restLength;
		double elongationVelocity = (v1 - v0).dot(u);
		double lRatio = elongationPosition / length;
		double vRatio = elongationVelocity / length;

		Matrix33d K00 = Matrix33d::Identity() * (m_stiffness * lRatio + m_damping * vRatio);
		K00 -= (u * u.transpose()) * (m_stiffness * (lRatio - 1.0) + 2.0 * m_damping * vRatio);
		K00 += m_damping * (u * (v1 - v0).transpose()) / length;

		const Vector3d force = alphaK * (K00 * (vector6D.segment(0, 3) - vector6D.segment(3, 3)));
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
