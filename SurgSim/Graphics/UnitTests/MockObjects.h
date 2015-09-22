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

#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/Group.h"
#include "SurgSim/Graphics/Manager.h"
#include "SurgSim/Graphics/Material.h"
#include "SurgSim/Graphics/Program.h"
#include "SurgSim/Graphics/RenderTarget.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/Texture.h"
#include "SurgSim/Graphics/UniformBase.h"
#include "SurgSim/Graphics/View.h"
#include "SurgSim/Graphics/ViewElement.h"
#include "SurgSim/Math/Vector.h"

#include <array>

class MockGroup : public SurgSim::Graphics::Group
{
public:
	/// Constructor. The group is initially empty.
	/// \param	name	Name of the group
	explicit MockGroup(const std::string& name) : SurgSim::Graphics::Group(name)
	{
	}

	/// Sets whether the group is currently visible
	/// \param    visible    True for visible, false for invisible
	virtual void setVisible(bool visible)
	{
		m_isVisible = visible;
	}

	/// Gets whether the group is currently visible
	/// \return    visible    True for visible, false for invisible
	virtual bool isVisible() const
	{
		return m_isVisible;
	}

private:
	/// Whether this group is currently visible or not
	bool m_isVisible;
};

/// Manager class for testing
class MockManager : public SurgSim::Graphics::Manager
{
public:

	friend class GraphicsManagerTest;

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

	void dumpDebugInfo() const
	{
		return;
	}

	int getType() const override
	{
		return SurgSim::Framework::MANAGER_TYPE_NONE;
	}

	virtual std::shared_ptr<SurgSim::Graphics::Group> getOrCreateGroup(const std::string& name)
	{
		if (getGroups().find(name) == std::end(getGroups()))
		{
			addGroup(std::make_shared<MockGroup>(name));
		}
		return getGroups().at(name);
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
	explicit MockRepresentation(const std::string& name) : SurgSim::Graphics::Representation(name),
		m_numUpdates(0),
		m_sumDt(0.0),
		m_isInitialized(false),
		m_isAwoken(false),
		m_drawAsWireFrame(false)
	{
		m_transform.setIdentity();
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

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		if (isActive())
		{
			++m_numUpdates;
			m_sumDt += dt;
		}
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

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	virtual bool setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
	{
		return false;
	}

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	virtual std::shared_ptr<SurgSim::Graphics::Material> getMaterial() const
	{
		return nullptr;
	}

	/// Removes the material from the representation
	virtual void clearMaterial()
	{
	}

	virtual void setDrawAsWireFrame(bool val)
	{
		m_drawAsWireFrame = val;
	}

	virtual bool getDrawAsWireFrame() const
	{
		return m_drawAsWireFrame;
	}


	void setGenerateTangents(bool value)
	{

	}

	bool isGeneratingTangents() const
	{
		return false;
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


	/// Number of times the representation has been updated
	int m_numUpdates;
	/// Sum of the dt that the representation has been updated with
	double m_sumDt;

	/// Whether the representation has been initialized
	bool m_isInitialized;
	/// Whether the representation has been awoken
	bool m_isAwoken;

	/// Indicates if the representation is rendered as a wireframe.
	bool m_drawAsWireFrame;

	/// Rigid transform describing pose of the representation
	SurgSim::Math::RigidTransform3d m_transform;
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
	explicit MockCamera(const std::string& name) :
		SurgSim::Graphics::Representation(name),
		SurgSim::Graphics::Camera(name),
		m_numUpdates(0),
		m_sumDt(0.0)
	{
		m_pose.setIdentity();
		m_viewMatrix.setIdentity();
		m_projectionMatrix.setIdentity();
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
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform)
	{
		m_pose = transform;
	}

	/// Gets the pose of the camera
	/// \return	Rigid transformation that describes the pose of the representation
	virtual SurgSim::Math::RigidTransform3d getPose() const
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
	virtual SurgSim::Math::Matrix44d getViewMatrix() const
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

	virtual SurgSim::Math::Matrix44d getInverseProjectionMatrix() const
	{
		return m_projectionMatrix.inverse();
	}

	/// Updates the camera.
	/// \param	dt	The time in seconds of the preceding timestep.
	/// \post	m_numUpdates is incremented and dt is added to m_sumDt
	virtual void update(double dt)
	{
		++m_numUpdates;
		m_sumDt += dt;
	}

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	virtual bool setMaterial(std::shared_ptr<SurgSim::Graphics::Material> material)
	{
		return false;
	}

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	virtual std::shared_ptr<SurgSim::Graphics::Material> getMaterial() const
	{
		return nullptr;
	}

	/// Removes the material from the representation
	virtual void clearMaterial()
	{
	}

	virtual void setDrawAsWireFrame(bool val)
	{
	}

	virtual bool getDrawAsWireFrame() const
	{
		return false;
	}

	virtual bool setColorRenderTexture(std::shared_ptr<SurgSim::Graphics::Texture> texture)
	{
		return true;
	}

	virtual std::shared_ptr<SurgSim::Graphics::Texture> getColorRenderTexture() const
	{
		return nullptr;
	}

	virtual bool setRenderTarget(std::shared_ptr<SurgSim::Graphics::RenderTarget> renderTarget)
	{
		return true;
	}

	virtual std::shared_ptr<SurgSim::Graphics::RenderTarget> getRenderTarget() const
	{
		return nullptr;
	}

	void setRenderOrder(RenderOrder bin, int value) override
	{

	}
	
	void setMainCamera(bool val) override
	{
		
	}
	
	bool isMainCamera() override
	{
		return false;
	}

	virtual SurgSim::Math::Matrix44d getInverseViewMatrix() const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setAmbientColor(const SurgSim::Math::Vector4d& color)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	SurgSim::Math::Vector4d getAmbientColor()
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setGenerateTangents(bool value)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	bool isGeneratingTangents() const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setPerspectiveProjection(double fovy, double aspect, double near, double far)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setOrthogonalProjection(double left, double right, double bottom, double top, double near, double far)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setViewport(int x, int y, int width, int height)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	std::array<int, 4> getViewport() const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void getViewport(int* x, int* y, int* width, int* height) const
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	void setViewportSize(std::array<double, 2> dimensions)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	std::array<double, 2> getViewportSize() const
	{
		throw std::logic_error("The method or operation is not implemented.");
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
	void setPosition(const std::array<int, 2>& position) override
	{
		m_x = position[0];
		m_y = position[1];
	}

	/// Get the position of this view
	/// \param[out]	x,y	Position on the screen (in pixels)
	std::array<int, 2> getPosition() const override
	{
		std::array<int, 2> result = {m_x, m_y};
		return result;
	}

	/// Set the dimensions of this view
	/// \param	width,height	Dimensions on the screen (in pixels)
	void setDimensions(const std::array<int, 2>& dimensions) override
	{
		m_width = dimensions[0];
		m_height = dimensions[1];
	}

	/// Set the dimensions of this view
	/// \param[out]	width,height	Dimensions on the screen (in pixels)
	std::array<int, 2> getDimensions() const override
	{
		std::array<int, 2> result = {m_width, m_height};
		return result;
	}

	void setDimensionsDouble(const std::array<double, 2>& dimensions)
	{
		m_width = dimensions[0];
		m_height = dimensions[1];
	}

	std::array<double, 2> getDimensionsDouble() const
	{
		std::array<double, 2> result = {static_cast<double>(m_width), static_cast<double>(m_height)};
		return std::move(result);
	}

	/// Sets whether the view window has a border
	/// \param	enabled	True to enable the border around the window; false for no border
	void setWindowBorderEnabled(bool enabled) override
	{
		m_isWindowBorderEnabled = enabled;
	}
	/// Returns whether the view window has a border
	/// \return	True to enable the border around the window; false for no border
	bool isWindowBorderEnabled() const override
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
	explicit NonGraphicsRepresentation(const std::string& name) : SurgSim::Framework::Representation(name)
	{
	}
};

#endif  // SURGSIM_GRAPHICS_UNITTESTS_MOCKOBJECTS_H
