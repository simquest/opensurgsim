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

#include "SurgSim/Blocks/SphereElement.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgProgram.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"

using SurgSim::Graphics::OsgMaterial;
using SurgSim::Graphics::OsgProgram;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Math::SphereShape;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;


namespace SurgSim
{

namespace Blocks
{

SphereElement::SphereElement(const std::string& name) :
	SurgSim::Framework::SceneElement(name)
{
}

bool SphereElement::doInitialize()
{
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(getName() + " Physics");

	physicsRepresentation->setDensity(700.0); // Wood
	physicsRepresentation->setLinearDamping(0.1);

	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(0.1); // 1cm Sphere
	physicsRepresentation->setShape(shape);

	std::shared_ptr<OsgSphereRepresentation> graphicsRepresentation =
		std::make_shared<OsgSphereRepresentation>(getName() + " Graphics");
	graphicsRepresentation->setRadius(shape->getRadius());

	std::shared_ptr<OsgMaterial> material = std::make_shared<OsgMaterial>("material");
	std::shared_ptr<OsgProgram> program = std::make_shared<OsgProgram>();

	program->setVertexShaderSource(
		"varying vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
		"	color.rgb = gl_Normal;\n"
		"	color.a = 1.0;\n"
		"}");
	program->setFragmentShaderSource(
		"varying vec4 color;\n"
		"void main(void)\n"
		"{\n"
		"	gl_FragColor = color;\n"
		"}");
	material->setProgram(program);
	graphicsRepresentation->setMaterial(material);

	addComponent(physicsRepresentation);
	addComponent(graphicsRepresentation);

	auto rigidCollision = std::make_shared<RigidCollisionRepresentation>("Sphere Collision Representation");
	physicsRepresentation->setCollisionRepresentation(rigidCollision);
	addComponent(rigidCollision);

	return true;
}

};
};
