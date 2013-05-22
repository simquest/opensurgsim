// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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

#ifndef SURGSIM_GRAPHICS_UNITTESTS_MOCKOSGOBJECTS_H
#define SURGSIM_GRAPHICS_UNITTESTS_MOCKOSGOBJECTS_H

#include "SurgSim/Graphics/OsgActor.h"

#include <osg/Group>

/// Actor class for testing
class MockOsgActor : public SurgSim::Graphics::OsgActor
{
public:
	/// Constructor
	/// \param	name	Name of the actor
	/// \post m_numUpdates and m_sumDt are initialized to 0
	/// \post m_transform is set to identity
	/// \post m_isInitialized and m_isAwoken are set to false
	/// \post m_isVisible is set to true
	explicit MockOsgActor(const std::string& name) : SurgSim::Graphics::OsgActor(name, new osg::Group()),
		SurgSim::Graphics::Actor(name),
		m_numUpdates(0),
		m_sumDt(0.0),
		m_isInitialized(false),
		m_isAwoken(false),
		m_isVisible(true)
	{
		m_transform.setIdentity();
	}

	/// Sets whether the actor is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible)
	{
		m_isVisible = visible;
	}

	/// Gets whether the actor is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const
	{
		return m_isVisible;
	}

	/// Returns the number of times the actor has been updated
	int getNumUpdates() const
	{
		return m_numUpdates;
	}
	/// Returns the sum of the dt that the actor has been updated with
	double getSumDt() const
	{
		return m_sumDt;
	}

	/// Sets the pose of the actor
	/// \param	transform	Rigid transformation that describes the pose of the actor
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_transform = transform;
	}

	/// Gets the pose of the actor
	/// \return	Rigid transformation that describes the pose of the actor
	virtual const SurgSim::Math::RigidTransform3d& getPose() const
	{
		return m_transform;
	}

	/// Updates the actor.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		++m_numUpdates;
		m_sumDt += dt;
	}

	/// Gets whether the actor has been initialized
	bool isInitialized() const
	{
		return m_isInitialized;
	}
	/// Gets whether the actor has been awoken
	bool isAwoken() const
	{
		return m_isAwoken;
	}

private:
	/// Initializes the actor
	/// \post m_isInitialized is set to true
	virtual bool doInitialize()
	{
		m_isInitialized = true;
		return true;
	}
	/// Wakes up the actor
	/// \post m_isAwoken is set to true
	virtual bool doWakeUp()
	{
		m_isAwoken = true;
		return true;
	}

	/// Whether this actor is currently visible or not
	bool m_isVisible;

	/// Number of times the actor has been updated
	int m_numUpdates;
	/// Sum of the dt that the actor has been updated with
	double m_sumDt;

	/// Whether the actor has been initialized
	bool m_isInitialized;
	/// Whether the actor has been awoken
	bool m_isAwoken;

	/// Rigid transform describing pose of the actor
	SurgSim::Math::RigidTransform3d m_transform;
};

#endif  // SURGSIM_GRAPHICS_UNITTESTS_MOCKOSGOBJECTS_H
