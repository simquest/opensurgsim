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

#ifndef SURGSIM_PHYSICS_VTCRIGIDPARAMETERS_H
#define SURGSIM_PHYSICS_VTCRIGIDPARAMETERS_H

namespace SurgSim
{

namespace Physics
{

/// The VtcParameters class defines all the physical parameters for a rigid vtc
class VtcParameters
{
public:
	/// Default constructor
	VtcParameters()
		: m_linearStiffness(0.0), m_linearDamping(0.0), m_angularStiffness(0.0), m_angularDamping(0.0)
	{
	}

	/// Destructor
	virtual ~VtcParameters()
	{
	}

	/// Comparison operator (equality test)
	/// \param p A VtcParameters to compare it to
	/// \return True if the 2 parameters set are equals, False otherwise
	bool operator ==(const VtcParameters &p) const
	{
		return (m_linearStiffness == p.m_linearStiffness && m_linearDamping == p.m_linearDamping &&
			m_angularStiffness == p.m_angularStiffness && m_angularDamping == p.m_angularDamping);
	}

	/// Comparison operator (difference test)
	/// \param p A VtcParameters to compare it to
	/// \return False if the 2 parameters set are equals, True otherwise
	bool operator !=(const VtcParameters &p) const
	{
		return ! ((*this) == p);
	}

	/// Set vtc linear stiffness
	/// \param linearStiffness The stiffness of the vtc in linear mode (in N·m-1)
	void setLinearStiffness(double linearStiffness)
	{
		m_linearStiffness = linearStiffness;
	}

	/// Get vtc linear stiffness
	/// \return The vtc linear stiffness (in N·m-1)
	double getLinearStiffness() const
	{
		return m_linearStiffness;
	}

	/// Set vtc linear damping
	/// \param linearDamping The damping of the vtc in linear mode (in N·s·m-1 or Kg·s-1)
	void setLinearDamping(double linearDamping)
	{
		m_linearDamping = linearDamping;
	}

	/// Get vtc linear damping
	/// \return The vtc linear damping (in N·s·m-1 or Kg·s-1)
	double getLinearDamping() const
	{
		return m_linearDamping;
	}

	/// Set vtc angular stiffness
	/// \param angularStiffness The stiffness of the vtc in angular mode (in N·m rad-1)
	void setAngularStiffness(double angularStiffness)
	{
		m_angularStiffness = angularStiffness;
	}

	/// Get vtc angular stiffness
	/// \return The vtc angular stiffness (in N·m rad-1)
	double getAngularStiffness() const
	{
		return m_angularStiffness;
	}

	/// Set vtc angular damping
	/// \param angularDamping The damping of the vtc in angular mode (in N·m·s·rad-1)
	void setAngularDamping(double angularDamping)
	{
		m_angularDamping = angularDamping;
	}

	/// Get the vtc angular damping
	/// \return the vtc angular damping (in N·m·s·rad-1)
	double getAngularDamping() const
	{
		return m_angularDamping;
	}

private:
	/// Vtc stiffness parameter in linear mode (in N·m-1)
	double m_linearStiffness;

	/// Vtc damping parameter in linear mode (in N·s·m-1 or Kg·s-1)
	double m_linearDamping;

	/// Vtc stiffness parameter in angular mode (in N·m rad-1)
	double m_angularStiffness;

	/// Vtc damping parameter in angular mode (in N·m·s·rad-1)
	double m_angularDamping;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VTCRIGIDPARAMETERS_H
