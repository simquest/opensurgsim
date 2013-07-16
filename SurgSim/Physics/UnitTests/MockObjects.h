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

#ifndef SURGSIM_PHYSICS_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_PHYSICS_UNITTESTS_MOCKOBJECTS_H

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Physics/Representation.h>

namespace SurgSim
{
namespace Physics
{

class MockRepresentation : public Representation
{
protected:
	int m_preUpdateCount;
	int m_updateCount;
	int m_postUpdateCount;

public:
	MockRepresentation() :
		Representation("MockRepresentation"), m_preUpdateCount(0), m_updateCount(0), m_postUpdateCount(0)
	{}

	virtual ~MockRepresentation()
	{}

	virtual RepresentationType getType() const override
	{
		return REPRESENTATION_TYPE_FIXED;
	}

	/// Set the initial pose of the representation
	/// \param pose The initial pose
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose)
	{}

	/// Get the initial pose of the representation
	/// \return The initial pose
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const
	{ static SurgSim::Math::RigidTransform3d pose; return pose; }

	/// Set the pose of the representation
	/// \param pose The pose to set the representation to
	/// \note This requests the representation to set its pose to the given pose
	/// \note In physics, the actual pose of the representation might not be exactly the requested one
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose)
	{}

	/// Get the pose of the representation
	/// \return The pose of this representation
	/// \note getPose may or may not return the pose last sets by setPose
	/// \note In physics, the simulation will drive the pose internally
	virtual const SurgSim::Math::RigidTransform3d& getPose() const
	{ static SurgSim::Math::RigidTransform3d pose; return pose; }

	/// Preprocessing done before the update call
	/// \param dt The time step (in seconds)
	virtual void beforeUpdate(double dt)
	{
		m_preUpdateCount++;
	}

	/// Update the representation state to the current time step
	/// \param dt The time step (in seconds)
	virtual void update(double dt)
	{
		m_updateCount++;
	}

	/// Postprocessing done after the update call
	/// \param dt The time step (in seconds)
	virtual void afterUpdate(double dt)
	{
		m_postUpdateCount++;
	}

	int getPreUpdateCount() const
	{ return m_preUpdateCount; }

	int getUpdateCount() const
	{ return m_updateCount; }

	int getPostUpdateCount() const
	{ return m_postUpdateCount; }
};


}; // Physics
}; // SurgSim

#endif