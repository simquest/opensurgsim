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

#ifndef SURGSIM_GRAPHICS_OSGCYLINDERREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGCYLINDERREPRESENTATION_H

#include <SurgSim/Graphics/CylinderRepresentation.h>
#include <SurgSim/Graphics/OsgRepresentation.h>

#include <SurgSim/Framework/SharedInstance.h>
#include <SurgSim/Math/RigidTransform.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Switch>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

class OsgUnitCylinder;

/// OSG implementation of a graphics Cylinder representation.
class OsgCylinderRepresentation : public OsgRepresentation, public CylinderRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgCylinderRepresentation(const std::string& name);

	/// Sets the radius of the cylinder
	virtual void setRadius(double radius);
	/// Returns the radius of the cylinder
	virtual double getRadius() const;

	/// Sets the height of the cylinder
	virtual void setHeight(double height);
	/// Returns the height of the cylinder
	virtual double getHeight() const;

	/// Sets the size of the cylinder
	/// \param radius Size along X-axis of the cylinder
	/// \param height Size along Y-axis of the cylinder
	virtual void setSize(double radius, double height);
	/// Gets the size of the cylinder
	/// \param radius Reference to store the size along X-axis of the cylinder
	/// \param height Reference to store the size along Y-axis of the cylinder
	virtual void getSize(double* radius, double* height);

	/// Sets the size of the cylinder
	/// \param size Size of the cylinder
	virtual void setSize(SurgSim::Math::Vector2d size);
	/// Returns the radius of the cylinder
	/// \return Size of the cylinder
	virtual SurgSim::Math::Vector2d getSize() const;

private:
	/// The OSG Cylinder shape is a unit Cylinder and this transform scales it to the size set.
	osg::Vec2d m_scale;

	/// Shared unit Cylinder, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitCylinder> m_sharedUnitCylinder;
	/// Returns the shared unit cylinder
	static std::shared_ptr<OsgUnitCylinder> getSharedUnitCylinder();
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCYLINDERREPRESENTATION_H
