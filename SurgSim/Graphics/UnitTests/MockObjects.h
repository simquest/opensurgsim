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

#ifndef SURGSIM_GRAPHICS_UNITTESTS_MOCKOBJECTS_H
#define SURGSIM_GRAPHICS_UNITTESTS_MOCKOBJECTS_H

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/Camera.h>
#include <SurgSim/Graphics/Group.h>
#include <SurgSim/Graphics/Manager.h>
#include <SurgSim/Graphics/View.h>
#include <SurgSim/Graphics/ViewElement.h>

/// Manager class for testing
class MockManager : public SurgSim::Graphics::Manager
{
public:
	/// Constructor
	/// \post m_numUpdates and m_sumDt are initialized to 0
	MockManager() : SurgSim::Graphics::Manager(),
		m_numUpdates(0),
		m_sumDt(0.0)
	{
	}

	/// Returns the number of times the manager has been updated
	int getNumUpdates() const
	{
		return m_numUpdates;
	}
	/// Returns the sum of the dt that the manager has been updated with
	double getSumDt() const
	{
		return m_sumDt;
	}

private:
	/// Updates the manager.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post	m_numUpdates is incremented and dt is added to m_sumDt
	virtual bool doUpdate(double dt)
	{
		if (Manager::doUpdate(dt))
		{
			++m_numUpdates;
			m_sumDt += dt;
			return true;
		}
		else
		{
			return false;
		}
	}

	/// Number of times the manager has been updated
	int m_numUpdates;
	/// Sum of the dt that the manager has been updated with
	double m_sumDt;
};

/// Representation class for testing
class MockRepresentation : public SurgSim::Graphics::Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	/// \post m_numUpdates and m_sumDt are initialized to 0
	/// \post m_transform is set to identity
	/// \post m_isInitialized and m_isAwoken are set to false
	/// \post m_isVisible is set to true
	explicit MockRepresentation(const std::string& name) : SurgSim::Graphics::Representation(name),
		m_isVisible(true),
		m_numUpdates(0),
		m_sumDt(0.0),
		m_isInitialized(false),
		m_isAwoken(false)
	{
		m_transform.setIdentity();
	}

	/// Sets whether the representation is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible)
	{
		m_isVisible = visible;
	}

	/// Gets whether the representation is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const
	{
		return m_isVisible;
	}

	/// Returns the number of times the representation has been updated
	int getNumUpdates() const
	{
		return m_numUpdates;
	}
	/// Returns the sum of the dt that the representation has been updated with
	double getSumDt() const
	{
		return m_sumDt;
	}

	/// Sets the current pose of the representation
	/// \param	transform	Rigid transformation that describes the current pose of the representation
	virtual void setCurrentPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_transform = transform;
	}

	/// Gets the current pose of the representation
	/// \return	Rigid transformation that describes the current pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getCurrentPose() const
	{
		return m_transform;
	}

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		++m_numUpdates;
		m_sumDt += dt;
	}

	/// Gets whether the representation has been initialized
	bool isInitialized() const
	{
		return m_isInitialized;
	}
	/// Gets whether the representation has been awoken
	bool isAwoken() const
	{
		return m_isAwoken;
	}

private:
	/// Initializes the representation
	/// \post m_isInitialized is set to true
	virtual bool doInitialize()
	{
		m_isInitialized = true;
		return true;
	}
	/// Wakes up the representation
	/// \post m_isAwoken is set to true
	virtual bool doWakeUp()
	{
		m_isAwoken = true;
		return true;
	}

	/// Whether this representation is currently visible or not
	bool m_isVisible;

	/// Number of times the representation has been updated
	int m_numUpdates;
	/// Sum of the dt that the representation has been updated with
	double m_sumDt;

	/// Whether the representation has been initialized
	bool m_isInitialized;
	/// Whether the representation has been awoken
	bool m_isAwoken;

	/// Rigid transform describing pose of the representation
	SurgSim::Math::RigidTransform3d m_transform;
};

class MockGroup : public SurgSim::Graphics::Group
{
public:
	/// Constructor. The group is initially empty.
	/// \param	name	Name of the group
	explicit MockGroup(const std::string& name) : SurgSim::Graphics::Group(name)
	{
	}

	/// Sets whether the group is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible)
	{
		m_isVisible = visible;
	}

	/// Gets whether the group is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const
	{
		return m_isVisible;
	}

private:
	/// Whether this group is currently visible or not
	bool m_isVisible;
};

/// Camera class for testing
class MockCamera : public SurgSim::Graphics::Camera
{
public:
	/// Constructor
	/// \param	name	Name of the camera
	/// \post m_numUpdates and m_sumDt are initialized to 0
	/// \post m_transform is set to identity, m_eye to (0,0,0), m_center to (0, 0, -1), and m_up to (0, 1, 0)
	/// \post m_isVisible is set to true
	explicit MockCamera(const std::string& name) : SurgSim::Graphics::Representation(name), SurgSim::Graphics::Camera(name),
		m_numUpdates(0),
		m_sumDt(0.0),
		m_isVisible(true)
	{
		m_pose.setIdentity();
		m_viewMatrix.setIdentity();
		m_projectionMatrix.setIdentity();
	}

	/// Sets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible)
	{
		m_isVisible = visible;
	}

	/// Gets whether the camera is currently visible
	/// When the camera is invisible, it does not produce an image.
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const
	{
		return m_isVisible;
	}

	/// Returns the number of times the representation has been updated
	int getNumUpdates() const
	{
		return m_numUpdates;
	}
	/// Returns the sum of the dt that the representation has been updated with
	double getSumDt() const
	{
		return m_sumDt;
	}

	/// Sets the current pose of the camera
	/// \param	transform	Rigid transformation that describes the current pose of the camera
	virtual void setCurrentPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_pose = transform;
	}

	/// Gets the pose of the camera
	/// \return	Rigid transformation that describes the pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getCurrentPose() const
	{
		return m_pose;
	}

	/// Sets the view matrix of the camera
	/// \param	matrix	View matrix
	virtual void setViewMatrix(const SurgSim::Math::Matrix44d& matrix)
	{
		m_viewMatrix = matrix;
	}

	/// Gets the view matrix of the camera
	/// \return	View matrix
	virtual const SurgSim::Math::Matrix44d& getViewMatrix() const
	{
		return m_viewMatrix;
	}

	/// Sets the projection matrix of the camera
	/// \param	matrix	Projection matrix
	virtual void setProjectionMatrix(const SurgSim::Math::Matrix44d& matrix)
	{
		m_projectionMatrix = matrix;
	}

	/// Gets the projection matrix of the camera
	/// \return	Projection matrix
	virtual const SurgSim::Math::Matrix44d& getProjectionMatrix() const
	{
		return m_projectionMatrix;
	}

	/// Updates the camera.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post	m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		++m_numUpdates;
		m_sumDt += dt;
	}

private:
	/// Number of times the camera has been updated
	int m_numUpdates;
	/// Sum of the dt that the camera has been updated with
	double m_sumDt;

	/// Rigid transform describing pose of the camera
	SurgSim::Math::RigidTransform3d m_pose;

	/// View matrix of the camera
	SurgSim::Math::Matrix44d m_viewMatrix;

	/// Projection matrix of the camera
	SurgSim::Math::Matrix44d m_projectionMatrix;

	/// Whether this camera is currently visible or not
	/// When the camera is invisible, it does not produce an image.
	bool m_isVisible;
};

/// View class for testing
class MockView : public SurgSim::Graphics::View
{
public:
	/// Constructor
	/// \param	name	Name of the view
	/// \post m_x and m_y are initialized to 0
	/// \post m_width is initialized to 800, m_height to 600
	/// \post m_isWindowBorderEnabled is initialized to true
	/// \post m_numUpdates and m_sumDt are initialized to 0
	/// \post m_transform is set to identity
	explicit MockView(const std::string& name) : SurgSim::Graphics::View(name),
		m_x(0),
		m_y(0),
		m_width(800),
		m_height(600),
		m_isWindowBorderEnabled(true),
		m_numUpdates(0),
		m_sumDt(0.0),
		m_isInitialized(false),
		m_isAwoken(false)
	{
	}

	/// Set the position of this view
	/// \param	x,y	Position on the screen (in pixels)
	virtual bool setPosition(int x, int y)
	{
		m_x = x;
		m_y = y;
		return true;
	}

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	virtual void getPosition(int* x, int* y) const
	{
		*x = m_x;
		*y = m_y;
	}

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	virtual bool setDimensions(int width, int height)
	{
		m_width = width;
		m_height = height;
		return true;
	}

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	virtual void getDimensions(int* width, int* height) const
	{
		*width = m_width;
		*height = m_height;
	}

	/// Sets whether the view window has a border
	/// \param	enabled	True to enable the border around the window; false for no border
	virtual void setWindowBorderEnabled(bool enabled)
	{
		m_isWindowBorderEnabled = enabled;
	}
	/// Returns whether the view window has a border
	/// \return	True to enable the border around the window; false for no border
	virtual bool isWindowBorderEnabled() const
	{
		return m_isWindowBorderEnabled;
	}

	/// Returns the number of times the view has been updated
	int getNumUpdates() const
	{
		return m_numUpdates;
	}
	/// Returns the sum of the dt that the view has been updated with
	double getSumDt() const
	{
		return m_sumDt;
	}

	/// Updates the view.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post	m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		++m_numUpdates;
		m_sumDt += dt;
	}

	/// Gets whether the view has been initialized
	bool isInitialized() const
	{
		return m_isInitialized;
	}
	/// Gets whether the view has been awoken
	bool isAwoken() const
	{
		return m_isAwoken;
	}

private:
	/// Initialize the view
	/// \post m_isInitialized is set to true
	virtual bool doInitialize()
	{
		m_isInitialized = true;
		return true;
	}
	/// Wake up the view
	/// \post m_isAwoken is set to true
	virtual bool doWakeUp()
	{
		m_isAwoken = true;
		return true;
	}

	/// Position of the view on the screen (in pixels)
	int m_x, m_y;
	/// Dimensions of the view on the screen (in pixels)
	int m_width, m_height;
	/// Whether the view window has a border
	bool m_isWindowBorderEnabled;

	/// Number of times the view has been updated
	int m_numUpdates;
	/// Sum of the dt that the view has been updated with
	double m_sumDt;

	/// Whether the view has been initialized
	bool m_isInitialized;
	/// Whether the view has been awoken
	bool m_isAwoken;
};

/// Representation that does not subclass any graphics components
class NonGraphicsRepresentation : public SurgSim::Framework::Representation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit NonGraphicsRepresentation(const std::string& name) : SurgSim::Framework::Representation(name),
		m_initialPose(SurgSim::Math::RigidTransform3d::Identity()),
		m_currentPose(SurgSim::Math::RigidTransform3d::Identity())
	{
	}

	/// Sets the initial pose of the representation
	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_initialPose = transform;
		setCurrentPose(transform);
	}
	/// Returns the initial pose of the representation
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

	/// Returns the final pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getFinalPose() const
	{
		return getCurrentPose();
	}
private:
	/// Initial pose of the representation
	SurgSim::Math::RigidTransform3d m_initialPose;
	/// Current pose of the representation
	SurgSim::Math::RigidTransform3d m_currentPose;
};

#endif  // SURGSIM_GRAPHICS_UNITTESTS_MOCKOBJECTS_H
