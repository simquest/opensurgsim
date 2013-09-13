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

#ifndef SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONSTATE_H
#define SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONSTATE_H

#include <SurgSim/DataStructures/TetrahedronMesh.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>

using SurgSim::DataStructures::TetrahedronMesh;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;

namespace SurgSim
{

namespace Physics
{

class MassParameter
{
public:
	MassParameter() : m_mass(0.0), m_velocity(0.0, 0.0, 0.0){}

	void setMass(double mass){ m_mass = mass; }
	double getMass() const { return m_mass; }

	void setVelocity(const Vector3d& velocity){ m_velocity = velocity; }
	const Vector3d& getVelocity() const { return m_velocity; }

	bool operator ==(const MassParameter& m) const { return (m_mass == m.m_mass && m_velocity == m.m_velocity); }
	bool operator !=(const MassParameter& m) const { return !((*this) == m); }

protected:
	double m_mass;
	Vector3d m_velocity;
};

class LinearSpringParameter
{
public:
	LinearSpringParameter() : m_stiffness(0.0), m_damping(0.0), m_l0(0.0){}

	void setStiffness(double stiffness) { m_stiffness = stiffness; }
	void setDamping(double damping) { m_damping = damping; }
	void setInitialLength(double l0) { m_l0 = l0; }
	double getStiffness() { return m_stiffness; }
	double getDamping() { return m_damping; }
	double getInitialLength() { return m_l0; }

	const Vector3d getF(
		const Eigen::VectorBlock<Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>>& xA,
		const Eigen::VectorBlock<Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>>& xB,
		const Eigen::VectorBlock<Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>>& vA,
		const Eigen::VectorBlock<Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>>& vB) const
	{
		Vector3d u = xB - xA;
		double m_l = u.norm();
		u /= m_l;

		return u * (m_l - m_l0) * m_stiffness;
	}

	const Matrix33d getdF_dx(const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& x,
		const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& v) const
	{
		return Matrix33d::Identity(); 
	}

	const Matrix33d getdF_dv(const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& x,
		const Eigen::Matrix<double,Eigen::Dynamic,1,Eigen::DontAlign>& v) const
	{
		return Matrix33d::Identity();
	}

	bool operator ==(const LinearSpringParameter& m) const
	{
		return m_l0 == m.m_l0 && m_stiffness == m.m_stiffness && m_damping == m.m_damping;
	}

	bool operator !=(const LinearSpringParameter& m) const { return !((*this) == m); }

private:
	double m_l0;
	double m_stiffness, m_damping;
};

typedef TetrahedronMesh<MassParameter, LinearSpringParameter, void, void> MassSpringRepresentationState;

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASSSPRINGREPRESENTATIONSTATE_H