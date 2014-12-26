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

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/CylinderRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"

#include <osg/PositionAttitudeTransform>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{
class OsgUnitCylinder;

SURGSIM_STATIC_REGISTRATION(OsgCylinderRepresentation);

/// OSG implementation of a graphics Cylinder representation.
class OsgCylinderRepresentation : public OsgRepresentation, public CylinderRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgCylinderRepresentation(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgCylinderRepresentation);

	/// Sets the radius of the cylinder
	/// \param	radius	Radius along X-axis and Z-axis of the cylinder
	void setRadius(double radius) override;
	/// Returns the radius of the cylinder
	/// \return	Radius along X-axis and Z-axis of cylinder
	double getRadius() const override;

	/// Sets the height of the cylinder
	/// \param	height	Height along Y-axis of the cylinder
	void setHeight(double height) override;
	/// Returns the height of the cylinder
	/// \return	Height along Y-axis of the cylinder
	double getHeight() const override;

	/// Sets the size of the cylinder
	/// \param radius Size along X-axis and Z-axis of the cylinder
	/// \param height Size along Y-axis of the cylinder
	void setSize(double radius, double height) override;
	/// Gets the size of the cylinder
	/// \param [out] radius Variable to receive the size along X-axis and Z-axis of the cylinder
	/// \param [out] height Variable to receive the size along Y-axis of the cylinder
	void getSize(double* radius, double* height) override;

	/// Sets the size of the cylinder
	/// \param size Size of the cylinder
	void setSize(const SurgSim::Math::Vector2d& size) override;
	/// Returns the size of the cylinder
	/// \return Size of the cylinder
	SurgSim::Math::Vector2d getSize() const override;

private:
	/// The OSG Cylinder shape is a unit Cylinder and this transform scales it to the size set.
	osg::Vec2d m_scale;

	/// Shared unit Cylinder, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitCylinder> m_sharedUnitCylinder;
	/// Returns the shared unit cylinder
	static std::shared_ptr<OsgUnitCylinder> getSharedUnitCylinder();

	osg::ref_ptr<osg::PositionAttitudeTransform> m_patCylinder;
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCYLINDERREPRESENTATION_H
