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

#include "Examples/ExampleStapling/StapleElement.h"

#include "SurgSim/Blocks/DriveElementBehavior.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"

using SurgSim::Blocks::DriveElementBehavior;
using SurgSim::DataStructures::PlyReader;
using SurgSim::DataStructures::TriangleMeshPlyReaderDelegate;
using SurgSim::Framework::ApplicationData;
using SurgSim::Graphics::SceneryRepresentation;
using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;

StapleElement::StapleElement(const std::string& name) :
	SurgSim::Framework::SceneElement(name),
	m_name(name)
{
}

StapleElement::~StapleElement()
{
}

bool StapleElement::doInitialize()
{
	std::vector<std::string> paths;
	paths.push_back("Data/Geometry");
	ApplicationData data(paths);

	std::shared_ptr<TriangleMeshPlyReaderDelegate> delegate = std::make_shared<TriangleMeshPlyReaderDelegate>();
	PlyReader reader(data.findFile("staple_collision.ply"));
	reader.setDelegate(delegate);
	reader.parseFile();

	// Stapler collision mesh
	std::shared_ptr<MeshShape> meshShape = std::make_shared<MeshShape>(*delegate->getMesh());
	RigidRepresentationParameters params;
	params.setDensity(8050); // Stainless steel (in Kg.m-3)
	params.setShapeUsedForMassInertia(meshShape);

	std::shared_ptr<RigidRepresentation> physicsRepresentation = std::make_shared<RigidRepresentation>("Physics");
	physicsRepresentation->setInitialParameters(params);

	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	collisionRepresentation->setRigidRepresentation(physicsRepresentation);

	std::shared_ptr<SceneryRepresentation> graphicsRepresentation =
		std::make_shared<OsgSceneryRepresentation>("Graphics");
	graphicsRepresentation->setFileName("Geometry/staple.obj");

	std::shared_ptr<DriveElementBehavior> driver;
	driver = std::make_shared<DriveElementBehavior>("Driver");
	driver->setFrom(physicsRepresentation);

	addComponent(physicsRepresentation);
	addComponent(collisionRepresentation);
	addComponent(graphicsRepresentation);
	addComponent(driver);

	return true;
}
