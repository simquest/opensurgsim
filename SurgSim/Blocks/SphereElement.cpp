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

#include <SurgSim/Blocks/SphereElement.h>
#include <SurgSim/Physics/RigidRepresentation.h>
#include <SurgSim/Physics/RigidRepresentationParameters.h>
#include <SurgSim/Graphics/OsgMaterial.h>
#include <SurgSim/Graphics/OsgShader.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>
#include <SurgSim/Blocks/TransferPoseBehavior.h>
#include <SurgSim/Collision/RigidCollisionRepresentation.h>


using SurgSim::Blocks::SphereElement;
using SurgSim::Blocks::TransferPoseBehavior;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Collision::RigidCollisionRepresentation;
using SurgSim::Math::SphereShape;
using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgShader;
using SurgSim::Graphics::OsgSphereRepresentation;

SphereElement::SphereElement(const std::string& name, const SurgSim::Math::RigidTransform3d& pose):
	SurgSim::Framework::SceneElement(name), m_name(name), m_pose(pose)
{
}


SphereElement::~SphereElement()
{
}

bool SphereElement::doInitialize()
{
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(m_name + " Physics");

	RigidRepresentationParameters params;
	params.setDensity(700.0); // Wood
	params.setLinearDamping(0.1);

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.1); // 1cm Sphere
	params.setShapeUsedForMassInertia(shape);

	physicsRepresentation->setInitialParameters(params);
	physicsRepresentation->setInitialPose(m_pose);

	std::shared_ptr<OsgSphereRepresentation> graphicsRepresentation =
		std::make_shared<OsgSphereRepresentation>(m_name + " Graphics");
	graphicsRepresentation->setRadius(shape->getRadius());
	graphicsRepresentation->setInitialPose(m_pose);

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>();
	std::shared_ptr<OsgShader> shader = std::make_shared<OsgShader>();

	shader->setVertexShaderSource(
		"varying vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		"	color.rgb = gl_Normal;\n"
		"	color.a = 1.0;\n"
		"}");
	shader->setFragmentShaderSource(
		"varying vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_FragColor = color;\n"
		"}");
	material->setShader(shader);
	graphicsRepresentation->setMaterial(material);

	addComponent(physicsRepresentation);
	addComponent(graphicsRepresentation);
	addComponent(std::make_shared<TransferPoseBehavior>("Physics to Graphics Pose",
		physicsRepresentation, graphicsRepresentation));
	addComponent(std::make_shared<SurgSim::Collision::RigidCollisionRepresentation>
		("Sphere Collision Representation", physicsRepresentation));

	return true;
}

bool SphereElement::doWakeUp()
{
	return true;
}
