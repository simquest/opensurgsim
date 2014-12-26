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

#include "SurgSim/Framework/SharedInstance.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/BoxRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Math/RigidTransform.h"

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
class OsgUnitBox;

SURGSIM_STATIC_REGISTRATION(OsgBoxRepresentation);

/// OSG implementation of a graphics box representation.
class OsgBoxRepresentation : public OsgRepresentation, public BoxRepresentation
{
public:
	/// Constructor
	/// \param	name	Name of the representation
	explicit OsgBoxRepresentation(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgBoxRepresentation);

	/// Sets the size along X-axis of the box
	/// \param sizeX Size along X-axis of the box
	void setSizeX(double sizeX) override;

	/// Returns the size along X-axis of the box
	/// \return Size along X-axis of the box
	double getSizeX() const override;

	/// Sets the size along Y-axis of the box
	/// \param sizeY Size along Y-axis of the box
	void setSizeY(double sizeY) override;

	/// Returns the size along Y-axis of the box
	/// \return Size along Y-axis of the box
	double getSizeY() const override;

	/// Sets the size along Z-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	void setSizeZ(double sizeZ) override;

	/// Returns the size along Z-axis of the box
	/// \return Size along Z-axis of the box
	double getSizeZ() const override;

	/// Sets the size of the box
	/// \param sizeX Size along X-axis of the box
	/// \param sizeY Size along Y-axis of the box
	/// \param sizeZ Size along Z-axis of the box
	virtual void setSizeXYZ(double sizeX, double sizeY, double sizeZ);
	/// Gets the size of the box
	/// \param sizeX Reference to store the size along X-axis of the box
	/// \param sizeY Reference to store the size along Y-axis of the box
	/// \param sizeZ Reference to store the size along Z-axis of the box
	virtual void getSizeXYZ(double* sizeX, double* sizeY, double* sizeZ) const;

	/// Sets the size of the box
	/// \param size Size of the box
	void setSize(const SurgSim::Math::Vector3d& size) override;

	/// Returns the extents of the box
	/// \return Size of the box
	SurgSim::Math::Vector3d getSize() const override;

private:
	/// The OSG box shape is a unit box and this transform scales it to the size set.
	osg::Vec3d m_scale;

	/// Shared unit box, so that the geometry can be instanced rather than having multiple copies.
	std::shared_ptr<OsgUnitBox> m_sharedUnitBox;
	/// Returns the shared unit box
	static std::shared_ptr<OsgUnitBox> getSharedUnitBox();
};

};  // namespace Graphics

};  // namespace SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif  // SURGSIM_GRAPHICS_OSGBOXREPRESENTATION_H
