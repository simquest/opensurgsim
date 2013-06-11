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

#ifndef SURGSIM_BLOCKS_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_BLOCKS_UNITTESTS_MOCKOBJECTS_H

#include <SurgSim/Framework/Representation.h>

/// Concrete Representation for testing
class MockRepresentation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post	m_pose is initialized to identity
	/// \post	m_didInit is initialized to false
	/// \post	m_didWakeUp is initialized to false
	explicit MockRepresentation(const std::string& name) : SurgSim::Framework::Representation(name),
		m_initialPose(SurgSim::Math::RigidTransform3d::Identity()),
		m_currentPose(SurgSim::Math::RigidTransform3d::Identity()),
		m_didInit(false),
		m_didWakeUp(false)
	{
	}

	/// Sets the current pose of the representation
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_initialPose = transform;
		setCurrentPose(transform);
	}
	/// Returns the current pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const
	{
		return m_initialPose;
	}

	/// Sets the current pose of the representation
	virtual void setCurrentPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_currentPose = transform;
	}
	/// Returns the current pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getCurrentPose() const
	{
		return m_currentPose;
	}

	/// Gets the final pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getFinalPose() const
	{
		return getCurrentPose();
	}

	/// Returns true if the representation has been initialized, otherwise false
	bool didInit() const
	{
		return m_didInit;
	}

	/// Returns true if the representation has been woken up, otherwise false
	bool didWakeUp() const
	{
		return m_didWakeUp;
	}

private:
	/// Initial pose of the representation
	SurgSim::Math::RigidTransform3d m_initialPose;
	/// Current pose of the representation
	SurgSim::Math::RigidTransform3d m_currentPose;

	/// Whether the representation has been initialized
	bool m_didInit;
	/// Whether the representation has been woken up
	bool m_didWakeUp;

	/// Initializes the representation
	/// \return	True if succeeds, otherwise false
	virtual bool doInitialize()
	{
		m_didInit = true;
		return true;
	}

	/// Wakes up the representation
	/// \return	True if succeeds, otherwise false
	virtual bool doWakeUp()
	{
		m_didWakeUp = true;
		return true;
	}
};

#endif  // SURGSIM_BLOCKS_UNITTESTS_MOCKOBJECTS_H
