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

#ifndef SURGSIM_GRAPHICS_OSGCAPSULEREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGCAPSULEREPRESENTATION_H

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/CapsuleRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"

#include "SurgSim/Framework/SharedInstance.h"

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
class OsgUnitSphere;

SURGSIM_STATIC_REGISTRATION(OsgCapsuleRepresentation);

/// OSG implementation of a graphics capsule representation.
class OsgCapsuleRepresentation : public OsgRepresentation, public CapsuleRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgCapsuleRepresentation(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgCapsuleRepresentation);

	/// Sets the radius of the capsule
	/// \param radius Radius of the capsule
	void setRadius(double radius) override;
	/// Returns the radius of the capsule
	/// \return Radius along X-axis and Z-axis of the capsule
	double getRadius() const override;

	/// Sets the height of the capsule
	/// \param height Height of the capsule
	void setHeight(double height) override;
	/// Returns the height of the capsule
	/// \return Height along Y-axis of the capsule
	double getHeight() const override;

	/// Sets the size of the capsule
	/// \param radius Size along X-axis and Z-axis of the capsule
	/// \param height Size along Y-axis of the capsule
	void setSize(double radius, double height) override;
	/// Gets the size of the capsule
	/// \param [out] radius Variable to receive the size along X-axis and Z-axis of the capsule
	/// \param [out] height Variable to receive the size along Y-axis of the capsule
	void getSize(double* radius, double* height) override;

	/// Sets the size of the capsule
	/// \param size Size of the capsule
	void setSize(const SurgSim::Math::Vector2d& size) override;
	/// Returns the radius of the capsule
	/// \return Size of the capsule
	SurgSim::Math::Vector2d getSize() const override;

private:
	/// The OSG Capsule shape consist of one unit cylinder and two unit spheres
	/// This transform scales it to the size set.
	osg::Vec2d m_scale;

	/// Shared capsule, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitCylinder> m_sharedUnitCylinder;
	std::shared_ptr<OsgUnitSphere> m_sharedUnitSphere;
	/// Returns the shared geometry
	static std::shared_ptr<OsgUnitCylinder> getSharedUnitCylinder();
	static std::shared_ptr<OsgUnitSphere> getSharedUnitSphere();

	osg::ref_ptr<osg::PositionAttitudeTransform> m_patCylinder;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_patSphere1;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_patSphere2;
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCAPSULEREPRESENTATION_H
