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

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace SurgSim
{

namespace Graphics
{

class OsgUnitSphere;

/// OSG implementation of a graphics sphere representation.
class OsgSphereRepresentation : public OsgRepresentation, public SphereRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgSphereRepresentation(const std::string& name);

	/// Sets the radius of the sphere
	/// \param	radius	Radius of the sphere
	virtual void setRadius(double radius);

	/// Returns the radius of the sphere
	/// \return	Radius of the sphere
	virtual double getRadius() const;

private:
	/// Shared unit sphere, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitSphere> m_sharedUnitSphere;

	/// Returns the shared unit sphere
	static std::shared_ptr<OsgUnitSphere> getSharedUnitSphere();
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGSPHEREREPRESENTATION_H
