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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Blocks/TransferPhysicsToGraphicsMeshBehavior.h"
#include "SurgSim/Blocks/TransferPhysicsToPointCloudBehavior.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/Mesh.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPointCloudRepresentation.h"
#include "SurgSim/Math/OdeSolver.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem3DPlyReaderDelegate.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

static std::shared_ptr<SurgSim::Framework::SceneElement> createFemSceneElement(
	const std::string& name,
	const std::string& filename,
	SurgSim::Math::IntegrationScheme integrationScheme)
{
	// Create a SceneElement that bundles the pieces associated with the finite element model
	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>(name);

	// Add the Fem3d component
	// Note that we only specify the filename that contains the full geometrical and physical description.
	auto fem = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("fem3d");
	fem->loadMesh(filename);
	fem->setIntegrationScheme(integrationScheme);
	sceneElement->addComponent(fem);

	// Add the graphics mesh used to display the Fem3d
	auto graphics = std::make_shared<SurgSim::Graphics::OsgMeshRepresentation>("fem graphics");
	graphics->loadMesh(filename);
	graphics->setDrawAsWireFrame(true);
	sceneElement->addComponent(graphics);

	// Create a behavior which transfers the position of the vertices in the FEM to locations in the triangle mesh
	auto femToMesh =
		std::make_shared<SurgSim::Blocks::TransferPhysicsToGraphicsMeshBehavior>("physics to triangle mesh");
	femToMesh->setSource(fem);
	femToMesh->setTarget(graphics);
	sceneElement->addComponent(femToMesh);

	// The point-cloud for visualizing the nodes of the finite element model
	auto pointCloud
		= std::make_shared<SurgSim::Graphics::OsgPointCloudRepresentation>("point cloud");
	pointCloud->setColor(SurgSim::Math::Vector4d(0.2, 0.2, 1.0, 1.0));
	pointCloud->setPointSize(3.0f);
	sceneElement->addComponent(pointCloud);

	// The behavior which transfers the position of the vertices in the FEM to locations in the point cloud
	auto femToCloud = std::make_shared<SurgSim::Blocks::TransferPhysicsToPointCloudBehavior>("fem to point cloud");
	femToCloud->setSource(fem);
	femToCloud->setTarget(pointCloud);
	sceneElement->addComponent(femToCloud);

	return sceneElement;
}

TEST_F(RenderTests, SimulatedWoundRenderTest)
{
	runtime->getScene()->addSceneElement(createFemSceneElement("Fem",
										 "Geometry/wound_deformable.ply",
										 SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER));

	runTest(Vector3d(0.0, 0.0, 0.2), Vector3d::Zero(), 5000.0);
}

} // namespace Physics
} // namespace SurgSim
