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

#ifndef SURGSIM_GRAPHICS_OSGSPHEREREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGSPHEREREPRESENTATION_H

#include <SurgSim/Graphics/SphereRepresentation.h>
#include <SurgSim/Graphics/OsgRepresentation.h>

#include <SurgSim/Framework/SharedInstance.h>
#include <SurgSim/Math/RigidTransform.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Switch>

namespace SurgSim
{

namespace Graphics
{

class OsgUnitSphere;

/// OSG implementation of a graphics sphere representation.
class OsgSphereRepresentation : public SphereRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgSphereRepresentation(const std::string& name);

	/// Sets whether the representation is currently visible
	/// \param	visible	True for visible, false for invisible
	virtual void setVisible(bool visible);

	/// Gets whether the representation is currently visible
	/// \return	visible	True for visible, false for invisible
	virtual bool isVisible() const;

	/// Sets the radius of the sphere
	/// \param	radius	Radius of the sphere
	virtual void setRadius(double radius);
	/// Returns the radius of the sphere
	/// \return	Radius of the sphere
	virtual double getRadius() const;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	/// \note	OsgSphereRepresentation only accepts subclasses of OsgMaterial.
	virtual bool setMaterial(std::shared_ptr<Material> material);

	/// Removes the material from the representation
	virtual void clearMaterial();

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
	/// Pose of the sphere
	SurgSim::Math::RigidTransform3d m_pose;
	/// OSG switch to set the visibility of the sphere
	osg::ref_ptr<osg::Switch> m_switch;
	/// OSG transform to set the pose and scale of the sphere
	/// The OSG sphere shape is a unit sphere and this transform scales it to the radius set.
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;

	/// Shared unit sphere, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitSphere> m_sharedUnitSphere;

	/// Returns the shared unit sphere
	static std::shared_ptr<OsgUnitSphere> getSharedUnitSphere();
};

};  // namespace Graphics

};  // namespace SurgSim

#endif  // SURGSIM_GRAPHICS_OSGSPHEREREPRESENTATION_H
