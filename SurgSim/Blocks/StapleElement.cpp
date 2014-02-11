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

#include <string>

#include "SurgSim/Blocks/StapleElement.h"
#include "SurgSim/Blocks/TransferPoseBehavior.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Math/CylinderShape.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"

using SurgSim::Blocks::StapleElement;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Math::CylinderShape;
using SurgSim::Physics::RigidRepresentationParameters;

StapleElement::StapleElement(const std::string& name):
	SurgSim::Framework::SceneElement(name),
	m_name(name)
{
	m_pose.setIdentity();
}


StapleElement::~StapleElement()
{
}

void StapleElement::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_pose = pose;
}

bool StapleElement::doInitialize()
{
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel
	params.setLinearDamping(2.0);

	// Shape of a cylinder is used to model the staple with length: 4.8mm and radius: 1.8mm
	auto shape = std::make_shared<CylinderShape>(0.0048, 0.0018);
	params.setShapeUsedForMassInertia(shape);

	auto physicsRepresentation = std::make_shared<SurgSim::Physics::RigidRepresentation>(m_name + " Physics");
	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(m_pose);

	// Graphics Representation: Load staple object from external file. 
	auto graphicsRepresentation = std::make_shared<SurgSim::Graphics::OsgSceneryRepresentation>(m_name + "Graphics");
	graphicsRepresentation->setFileName("staple_collision.obj");
	graphicsRepresentation->setInitialPose(m_pose);

	auto transferPose = std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose");
	transferPose->setPoseSender(physicsRepresentation);
	transferPose->setPoseReceiver(graphicsRepresentation);

	addComponent(physicsRepresentation);
	addComponent(graphicsRepresentation);
	addComponent(transferPose);

	return true;
}
