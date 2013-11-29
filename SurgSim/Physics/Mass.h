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

#ifndef SURGSIM_PHYSICS_MASS_H
#define SURGSIM_PHYSICS_MASS_H

namespace SurgSim
{

namespace Physics
{

// Class to handle a mass of a MassSpring node
class Mass
{
public:
	/// Constructor
	/// \param mass The mass (in Kg) default value is 0
	explicit Mass(double mass = 0.0) : m_mass(mass)
	{
	}

	/// Sets the mass
	/// \param mass The mass to be stored (in Kg)
	void setMass(double mass)
	{
		m_mass = mass;
	}

	/// Gets the mass
	/// \return The mass stored (in Kg)
	double getMass() const
	{
		return m_mass;
	}

	/// Comparison operator (equality)
	/// \param m Mass to compare it to
	/// \return True if the 2 Mass contains the same information, false otherwise
	bool operator ==(const Mass& m) const
	{
		return (m_mass == m.m_mass);
	}

	/// Comparison operator (inequality)
	/// \param m Mass to compare it to
	/// \return False if the 2 Mass contains the same information, True otherwise
	bool operator !=(const Mass& m) const
	{
		return ! ((*this) == m);
	}

protected:
	/// Mass (in kg)
	double m_mass;
};

} // namespace Physics

} // namespace SurgSim

#endif // SURGSIM_PHYSICS_MASS_H
