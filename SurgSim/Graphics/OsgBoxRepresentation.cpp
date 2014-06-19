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

#include "SurgSim/Graphics/OsgBoxRepresentation.h"

#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"
#include "SurgSim/Graphics/OsgUnitBox.h"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgBoxRepresentation, OsgBoxRepresentation);

OsgBoxRepresentation::OsgBoxRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	BoxRepresentation(name),
	m_scale(1.0, 1.0, 1.0),
	m_sharedUnitBox(getSharedUnitBox())
{
	m_transform->addChild(m_sharedUnitBox->getNode());
}


void OsgBoxRepresentation::setSizeX(double sizeX)
{
	m_scale.x() = sizeX;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeX() const
{
	return m_scale.x();
}

void OsgBoxRepresentation::setSizeY(double sizeY)
{
	m_scale.y() = sizeY;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeY() const
{
	return m_scale.y();
}

void OsgBoxRepresentation::setSizeZ(double sizeZ)
{
	m_scale.z() = sizeZ;
	m_transform->setScale(m_scale);
}
double OsgBoxRepresentation::getSizeZ() const
{
	return m_scale.z();
}

void OsgBoxRepresentation::setSizeXYZ(double sizeX, double sizeY, double sizeZ)
{
	m_scale.x() = sizeX;
	m_scale.y() = sizeY;
	m_scale.z() = sizeZ;
	m_transform->setScale(m_scale);
}
void OsgBoxRepresentation::getSizeXYZ(double* sizeX, double* sizeY, double* sizeZ) const
{
	*sizeX = m_scale.x();
	*sizeY = m_scale.y();
	*sizeZ = m_scale.z();
}

void OsgBoxRepresentation::setSize(const SurgSim::Math::Vector3d& size)
{
	m_scale.set(size.x(), size.y(), size.z());
	m_transform->setScale(m_scale);
}
SurgSim::Math::Vector3d OsgBoxRepresentation::getSize() const
{
	return SurgSim::Math::Vector3d(m_scale._v);
}

std::shared_ptr<OsgUnitBox> OsgBoxRepresentation::getSharedUnitBox()
{
	static SurgSim::Framework::SharedInstance<OsgUnitBox> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim