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

#include <SurgSim/Graphics/CapsuleRepresentation.h>
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
class OsgUnitSphere;

/// OSG implementation of a graphics Capsule representation.
class OsgCapsuleRepresentation : public OsgRepresentation, public CapsuleRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgCapsuleRepresentation(const std::string& name);

	/// Sets the radius of the capsule
	/// \param radius Radius of the capsule
	virtual void setRadius(double radius);
	/// Returns the radius of the capsule
	/// \return Radius along X-axis and Z-axis of the capsule
	virtual double getRadius() const;

	/// Sets the height of the capsule
	/// \param height Height of the capsule
	virtual void setHeight(double height);
	/// Returns the height of the capsule
	/// \return Height along Y-axis of the capsule
	virtual double getHeight() const;

	/// Sets the size of the capsule
	/// \param radius Size along X-axis and Z-axis of the capsule
	/// \param height Size along Y-axis of the capsule
	virtual void setSize(double radius, double height);
	/// Gets the size of the capsule
	/// \param [out] radius Variable to receive the size along X-axis and Z-axis of the capsule
	/// \param [out] height Variable to receive the size along Y-axis of the capsule
	virtual void getSize(double* radius, double* height);

	/// Sets the size of the capsule
	/// \param size Size of the capsule
	virtual void setSize(SurgSim::Math::Vector2d size);
	/// Returns the radius of the capsule
	/// \return Size of the capsule
	virtual SurgSim::Math::Vector2d getSize() const;

private:
	/// The OSG Capsule shape consist of one unit cylinder and two unit spheres
	/// This transform scales it to the size set.
	osg::Vec2d m_scale;

	/// Shared capsule, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitCylinder> m_sharedUnitCylinder;
	std::shared_ptr<OsgUnitSphere> m_sharedUnitSphere1;
	std::shared_ptr<OsgUnitSphere> m_sharedUnitSphere2;
	/// Returns the shared capsule
	static std::shared_ptr<OsgUnitCylinder> getSharedUnitCylinder();
	static std::shared_ptr<OsgUnitSphere> getSharedUnitSphere1();
	static std::shared_ptr<OsgUnitSphere> getSharedUnitSphere2();

	osg::ref_ptr<osg::PositionAttitudeTransform> m_PatCylinder;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_PatSphere1;
	osg::ref_ptr<osg::PositionAttitudeTransform> m_PatSphere2;
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGCAPSULEREPRESENTATION_H
