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

#include "SurgSim/Graphics/OsgPlaneRepresentation.h"

#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgPlane.h"
#include "SurgSim/Graphics/OsgRigidTransformConversions.h"

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

namespace SurgSim
{
namespace Graphics
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Graphics::OsgPlaneRepresentation, OsgPlaneRepresentation);

OsgPlaneRepresentation::OsgPlaneRepresentation(const std::string& name) :
	Representation(name),
	OsgRepresentation(name),
	PlaneRepresentation(name),
	m_sharedPlane(getSharedPlane())
{
	m_transform->addChild(m_sharedPlane->getNode());
}

std::shared_ptr<OsgPlane> OsgPlaneRepresentation::getSharedPlane()
{
	static SurgSim::Framework::SharedInstance<OsgPlane> shared;
	return shared.get();
}

}; // Graphics
}; // SurgSim