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

#ifndef SURGSIM_GRAPHICS_OSGBOXREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGBOXREPRESENTATION_H

#include <SurgSim/Graphics/BoxRepresentation.h>
#include <SurgSim/Graphics/OsgRepresentation.h>

#include <SurgSim/Framework/SharedInstance.h>
#include <SurgSim/Math/RigidTransform.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Switch>

namespace SurgSim
{

namespace Graphics
{

class OsgUnitBox;

/// OSG implementation of a graphics box representation.
class OsgBoxRepresentation : public BoxRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgBoxRepresentation(const std::string& name);

	/// Sets whether the representation is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible);

	/// Gets whether the representation is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	/// \note	OsgBoxRepresentation only accepts subclasses of OsgMaterial.
	virtual bool setMaterial(std::shared_ptr<Material> material);

	/// Removes the material from the representation
	virtual void clearMaterial();

	/// Sets the size along X-axis of the box
	/// \param sizeX Size along X-axis of the box
	virtual void setSizeX(double sizeX);
	/// Returns the size along X-axis of the box
	/// \return Size along X-axis of the box
	virtual double getSizeX() const;

	/// Sets the size along Y-axis of the box
	/// \param sizeY Size along Y-axis of the box
	virtual void setSizeY(double sizeX);
	/// Returns the size along Y-axis of the box
	/// \return Size along Y-axis of the box
	virtual double getSizeY() const;

	/// Sets the size along Z-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	virtual void setSizeZ(double sizeX);
	/// Returns the size along Z-axis of the box
	/// \return Size along Z-axis of the box
	virtual double getSizeZ() const;

	/// Sets the size of the box
	/// \param sizeX Size along X-axis of the box
	/// \param sizeY Size along Y-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	virtual void setSize(double sizeX, double sizeY, double sizeZ);
	/// Gets the size of the box
	/// \param sizeX Reference to store the size along X-axis of the box
	/// \param sizeY Reference to store the size along Y-axis of the box
	/// \param sizeZ Reference to store the size along Z-axis of the box
	virtual void getSize(double& sizeX, double& sizeY, double& sizeZ);

	/// Sets the size of the box
	/// \param size Size of the box
	virtual void setSize(SurgSim::Math::Vector3d size);
	/// Returns the radius of the sphere
	/// \return Size of the box
	virtual SurgSim::Math::Vector3d getSize() const;

	/// Sets the current pose of the representation
	/// \param	transform	Rigid transformation that describes the current pose of the representation
	virtual void setPose(const SurgSim::Math::RigidTransform3d& transform);

	/// Gets the current pose of the representation
	/// \return	Rigid transformation that describes the current pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt);

private:
	/// Pose of the box
	SurgSim::Math::RigidTransform3d m_pose;
	/// OSG switch to set the visibility of the box
	osg::ref_ptr<osg::Switch> m_switch;
	/// OSG transform to set the pose and scale of the box
	/// The OSG box shape is a unit box and this transform scales it to the size set.
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;
	osg::Vec3d m_scale;

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;

	/// Returns the shared unit sphere
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGBOXREPRESENTATION_H
