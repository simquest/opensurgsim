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

#ifndef SURGSIM_GRAPHICS_OSGMANAGER_H
#define SURGSIM_GRAPHICS_OSGMANAGER_H

#include "SurgSim/Graphics/Manager.h"

#include <memory>

#include <osgViewer/CompositeViewer>

namespace SurgSim
{

namespace Graphics
{

class Representation;
class Group;
class View;
class OsgCamera;
class OsgGroup;
class OsgScreenSpacePass;

/// OSG-based implementation of graphics manager class.
///
/// A Graphics::OsgManager sets up an osgViewer::CompositeViewer to manage and render each Graphics::OsgView added to
/// the Manager.
class OsgManager : public Manager
{
public:
	/// Constructor
	///
	/// Sets up a default Camera which will be used if for any View that does not have a Camera assigned
	/// Sets up a default Group which is assigned to the default Camera. All added representations and groups will be
	/// added to this group.
	OsgManager();
	/// Destructor
	virtual ~OsgManager();

	friend class OsgManagerTest;

	/// Returns the OSG CompositeViewer used to manage and render the views
	osg::ref_ptr<osgViewer::CompositeViewer> getOsgCompositeViewer() const
	{
		return m_viewer;
	}

	/// OsgManager will write out the scenegraph in the working directory
	virtual void dumpDebugInfo() const override;

protected:
	/// Performs an update for a single timestep
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual bool doUpdate(double dt) override;

	/// Initializes the manager
	/// \return True if it succeeds, false if it fails
	/// \post	The default camera component is in the list of managed representations.
	/// \post	The default group component is in the list of managed groups.
	virtual bool doInitialize() override;

	/// Starts up the manager after all threads have initialized
	/// \return True if it succeeds, false if it fails
	virtual bool doStartUp() override;

	/// Adds an representation to the manager
	/// \param	representation	The representation to be added.
	/// Only allows OsgRepresentation components, any other will not be set and it will return false.
	/// \return	True if the representation was not in this manager and has been successfully added, false if it fails.
	/// \post	The representation is added to the default group.
	virtual bool addRepresentation(std::shared_ptr<Representation> representation) override;

	/// Adds a group to the manager
	/// \param	group	The group to be added.
	/// Only allows OsgGroup components, any other will not be set and it will return false.
	/// \return	True if the group was not in this manager and has been successfully added, false if it fails.
	/// \post	The group is added to the default group.
	virtual bool addGroup(std::shared_ptr<Group> group) override;

	/// Adds a view to the manager
	/// \param	view	The view to be added.
	/// Only allows OsgView components, any other will not be set and it will return false.
	/// \return	True if the view was not in this manager and has been successfully added, false if it fails.
	/// \post	If the view had no camera, it's camera will be set to the default camera.
	virtual bool addView(std::shared_ptr<View> view) override;

	/// Removes a view from the manager
	/// \param	view	The view to be removed.
	/// \return	True if the view was in this manager and has been successfully removed, false if it fails.
	/// \post	The view is removed from the manager and the osgViewer::CompositeViewer.
	virtual bool removeView(std::shared_ptr<View> view) override;

	virtual std::shared_ptr<Group> getOrCreateGroup(const std::string& name) override;

private:

	/// Prepares the manager for its execution to be stopped
	/// \note	Called from this thread before joined
	void doBeforeStop();

	/// OSG CompositeViewer to manage and render the individual views
	osg::ref_ptr<osgViewer::CompositeViewer> m_viewer;

	/// Builtin RenderPass that can be used for HUD functionality, uses Group "ossHud"
	std::shared_ptr<OsgScreenSpacePass> m_hudElement;
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGMANAGER_H
