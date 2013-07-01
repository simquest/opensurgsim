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

#ifndef SURGSIM_GRAPHICS_OSGREPRESENTATIONBASE_H
#define SURGSIM_GRAPHICS_OSGREPRESENTATIONBASE_H

#include <memory>

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/Representation.h>

#include <osg/ref_ptr>

namespace osg
{
	class Switch;
	class Node;
	class PositionAttitudeTransform;
}

namespace SurgSim
{
namespace Graphics
{

class OsgMaterial;

class OsgRepresentation : public virtual Representation
{
public:

	/// Constructor
	OsgRepresentation(const std::string& name);
	virtual ~OsgRepresentation();

	osg::ref_ptr<osg::Node> getOsgNode() const;

	/// Sets whether the representation is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible);

	/// Gets whether the representation is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const;

	virtual void setInitialPose(const SurgSim::Math::RigidTransform3d& pose);

	virtual const SurgSim::Math::RigidTransform3d& getInitialPose() const;

	/// Sets the current pose of the representation
	/// \param	transform	Rigid transformation that describes the current pose of the representation
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Gets the current pose of the representation
	/// \return	Rigid transformation that describes the current pose of the representation
	virtual const SurgSim::Math::RigidTransform3d& getPose() const;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	/// \note	OsgPlaneRepresentation only accepts subclasses of OsgMaterial.
	virtual bool setMaterial(std::shared_ptr<Material> material);

	virtual std::shared_ptr<Material> getMaterial() const;

	/// Removes the material from the representation
	virtual void clearMaterial();

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	virtual void update(double dt);

protected:
	osg::ref_ptr<osg::Switch> m_switch;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;

private:
	virtual void doUpdate(double dt);

	/// Initial pose of the representation
	SurgSim::Math::RigidTransform3d m_initialPose;
	SurgSim::Math::RigidTransform3d m_pose;

	std::shared_ptr<OsgMaterial> m_material;

};

}; // Graphics
}; // SurgSim

#endif