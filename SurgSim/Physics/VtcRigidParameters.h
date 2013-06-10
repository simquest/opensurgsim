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

/// The VtcRigidParameters class defines all the physical parameters for a rigid vtc
class VtcRigidParameters
{
public:
	/// Default constructor
	VtcRigidParameters()
		: m_vtcLinearStiffness(0.0), m_vtcLinearDamping(0.0), m_vtcAngularStiffness(0.0), m_vtcAngularDamping(0.0)
	{
	}

	/// Destructor
	virtual ~VtcRigidParameters()
	{
	}

	/// Comparison operator (equality test)
	/// \param p A VtcRigidParameters to compare it to
	/// \return True if the 2 parameters set are equals, False otherwise
	bool operator ==(const VtcRigidParameters &p) const
	{
		return (m_vtcLinearStiffness == p.m_vtcLinearStiffness && m_vtcLinearDamping == p.m_vtcLinearDamping &&
			m_vtcAngularStiffness == p.m_vtcAngularStiffness && m_vtcAngularDamping == p.m_vtcAngularDamping);
	}

	/// Comparison operator (difference test)
	/// \param p A VtcRigidParameters to compare it to
	/// \return False if the 2 parameters set are equals, True otherwise
	bool operator !=(const VtcRigidParameters &p) const
	{
		return ! ((*this) == p);
	}

	/// Set vtc linear stiffness
	/// \param linearStiffness The stiffness of the vtc in linear mode (in N.m-1)
	void setVtcLinearStiffness(double linearStiffness)
	{
		m_vtcLinearStiffness = linearStiffness;
	}

	/// Get vtc linear stiffness
	/// \return The vtc linear stiffness (in N.m-1)
	double getVtcLinearStiffness() const
	{
		return m_vtcLinearStiffness;
	}

	/// Set vtc linear damping
	/// \param linearDamping The damping of the vtc in linear mode (in N.s.m-1 or Kg.s-1)
	void setVtcLinearDamping(double linearDamping)
	{
		m_vtcLinearDamping = linearDamping;
	}

	/// Get vtc linear damping
	/// \return The vtc linear damping (in N.s.m-1 or Kg.s-1)
	double getVtcLinearDamping() const
	{
		return m_vtcLinearDamping;
	}

	/// Set vtc angular stiffness
	/// \param angularStiffness The stiffness of the vtc in angular mode (in N·m rad-1)
	void setVtcAngularStiffness(double angularStiffness)
	{
		m_vtcAngularStiffness = angularStiffness;
	}

	/// Get vtc angular stiffness
	/// \return The vtc angular stiffness (in N·m rad-1)
	double getVtcAngularStiffness() const
	{
		return m_vtcAngularStiffness;
	}

	/// Set vtc angular damping
	/// \param angularDamping The damping of the vtc in angular mode (in N.m.s.rad-1)
	void setVtcAngularDamping(double angularDamping)
	{
		m_vtcAngularDamping = angularDamping;
	}

	/// Get the vtc angular damping
	/// \return the vtc angular damping (in N.m.s.rad-1)
	double getVtcAngularDamping() const
	{
		return m_vtcAngularDamping;
	}

private:
	/// Vtc stiffness parameter in linear mode (in N.m-1)
	double m_vtcLinearStiffness;

	/// Vtc damping parameter in linear mode (in N.s.m-1 or Kg.s-1)
	double m_vtcLinearDamping;

	/// Vtc stiffness parameter in angular mode (in N·m rad-1)
	double m_vtcAngularStiffness;

	/// Vtc damping parameter in angular mode (in N.m.s.rad-1)
	double m_vtcAngularDamping;
};

}; // Physics

}; // SurgSim

#endif // SURGSIM_PHYSICS_VTCRIGIDPARAMETERS_H
