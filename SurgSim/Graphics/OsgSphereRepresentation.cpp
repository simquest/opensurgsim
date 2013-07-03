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

#include <SurgSim/Graphics/OsgSphereRepresentation.h>

#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgRigidTransformConversions.h>
#include <SurgSim/Graphics/OsgUnitSphere.h>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Graphics::OsgUnitSphere;

OsgSphereRepresentation::OsgSphereRepresentation(const std::string& name) :
	Representation(name),
	SphereRepresentation(name),
	OsgRepresentation(name),
	m_sharedUnitSphere(getSharedUnitSphere())
{
	m_transform->addChild(m_sharedUnitSphere->getNode());
}

void OsgSphereRepresentation::setRadius(double radius)
{
	m_transform->setScale(osg::Vec3d(radius, radius, radius));
}

double OsgSphereRepresentation::getRadius() const
{
	SURGSIM_ASSERT(m_transform->getScale().x() == m_transform->getScale().y() &&
				   m_transform->getScale().x() == m_transform->getScale().z()) <<
		"Sphere should be scaled equally in all directions!";
	return m_transform->getScale().x();
}

std::shared_ptr<OsgUnitSphere> OsgSphereRepresentation::getSharedUnitSphere()
{
	static SurgSim::Framework::SharedInstance<OsgUnitSphere> shared;
	return shared.get();
}
