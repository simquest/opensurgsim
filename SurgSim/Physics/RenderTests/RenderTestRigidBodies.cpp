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

///\file RenderTestRigidBodies.cpp render test for RigidRepresentation

#include <memory>

#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgMeshRepresentation.h"
#include "SurgSim/Graphics/OsgPlaneRepresentation.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RenderTests/RenderTest.h"

using SurgSim::DataStructures::loadTriangleMesh;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Math::BoxShape;
using SurgSim::Math::MeshShape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationParameters;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidSphereSceneElement(const std::string& name, double radius)
{
	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + "Physics");
	RigidRepresentationParameters params;
	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(radius);
	params.setDensity(750.0); // Average mass density of
							  // Oak wood (http://www.engineeringtoolbox.com/wood-density-d_40.html)
	params.setShapeUsedForMassInertia(shape);
	params.setLinearDamping(1e-2);
	params.setAngularDamping(1e-4);
	physicsRepresentation->setInitialParameters(params);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgSphereRepresentation> osgRepresentation =
		std::make_shared<OsgSphereRepresentation>(name + "OsgRepresentation");
	osgRepresentation->setRadius(radius);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidBoxSceneElement(const std::string& name, Vector3d size)
{
	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + "Physics");
	RigidRepresentationParameters params;
	std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(size[0], size[1], size[2]);
	params.setDensity(750.0); // Average mass density of
							  // Oak wood (http://www.engineeringtoolbox.com/wood-density-d_40.html)
	params.setShapeUsedForMassInertia(shape);
	params.setLinearDamping(1e-2);
	params.setAngularDamping(1e-4);
	physicsRepresentation->setInitialParameters(params);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgBoxRepresentation> osgRepresentation =
		std::make_shared<OsgBoxRepresentation>(name + "OsgRepresentation");
	osgRepresentation->setSize(size);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createFixedPlaneSceneElement(const std::string& name)
{
	// Physics representation
	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>(name + "Physics");
	RigidRepresentationParameters params;
	std::shared_ptr<PlaneShape> shape = std::make_shared<PlaneShape>();
	params.setShapeUsedForMassInertia(shape);
	physicsRepresentation->setInitialParameters(params);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgPlaneRepresentation> osgRepresentation =
		std::make_shared<OsgPlaneRepresentation>(name + "OsgRepresentation");

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidMeshSceneElement(
	const std::string& name, std::string plyFilename, double scale = 1.0)
{
	std::vector<std::string> paths;
	paths.push_back("Data");
	SurgSim::Framework::ApplicationData data(paths);

	std::string foundFilename = data.findFile(plyFilename);
	SURGSIM_ASSERT(!foundFilename.empty()) << "Ply file '" << plyFilename << "' could not be located";
	auto mesh = loadTriangleMesh(foundFilename);
	for (size_t vertexId = 0; vertexId < mesh->getNumVertices(); ++vertexId)
	{
		mesh->setVertexPosition(vertexId, mesh->getVertexPosition(vertexId) * scale);
	}

	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>(name + "Physics");
	RigidRepresentationParameters params;
	std::shared_ptr<MeshShape> shape = std::make_shared<MeshShape>(*mesh);
	params.setDensity(750.0); // Average mass density of
							  // Oak wood (http://www.engineeringtoolbox.com/wood-density-d_40.html)
	params.setShapeUsedForMassInertia(shape);
	params.setLinearDamping(1e-2);
	params.setAngularDamping(1e-4);
	physicsRepresentation->setInitialParameters(params);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>(name + "Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic representation of the physics model
	std::shared_ptr<OsgMeshRepresentation> osgRepresentation =
		std::make_shared<OsgMeshRepresentation>(name + "OsgRepresentation");
	*osgRepresentation->getMesh() = SurgSim::Graphics::Mesh(*mesh);
	osgRepresentation->setDrawAsWireFrame(true);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name + "SceneElement");
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

}; // anonymous namespace

namespace SurgSim
{

namespace Physics
{

TEST_F(RenderTests, VisualTestFallingRigidBodies)
{
	using SurgSim::Math::makeRigidTranslation;

	// Mesh-base objects
	std::shared_ptr<SurgSim::Framework::SceneElement> boxMesh =
		createRigidMeshSceneElement("boxMesh1", "box.ply", 1.0);
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.35, 0.3, 0.0)));

	boxMesh = createRigidMeshSceneElement("boxMesh2", "box.ply", 0.5);
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.25, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
		createRigidMeshSceneElement("sphereMesh1", "sphere.ply", 1.0);
	scene->addSceneElement(sphereMesh);
	sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.15, 0.3, 0.0)));

	sphereMesh = createRigidMeshSceneElement("sphereMesh2", "sphere.ply", 0.5);
	scene->addSceneElement(sphereMesh);
	sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.05, 0.3, 0.0)));

	// Shape-base objects
	std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
		createRigidSphereSceneElement("sphereShape1", 0.05 * 0.5);
	scene->addSceneElement(sphereShape);
	sphereShape->setPose(makeRigidTranslation(Vector3d(0.05, 0.3, 0.0)));

	sphereShape = createRigidSphereSceneElement("sphereShape2", 0.05 * 1.0);
	scene->addSceneElement(sphereShape);
	sphereShape->setPose(makeRigidTranslation(Vector3d(0.15, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> boxShape =
		createRigidBoxSceneElement("boxShape1", Vector3d(0.1, 0.1, 0.1) * 0.5);
	scene->addSceneElement(boxShape);
	boxShape->setPose(makeRigidTranslation(Vector3d(0.25, 0.3, 0.0)));

	boxShape = createRigidBoxSceneElement("boxShape2", Vector3d(0.1, 0.1, 0.1) * 1.0);
	scene->addSceneElement(boxShape);
	boxShape->setPose(makeRigidTranslation(Vector3d(0.35, 0.3, 0.0)));

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 2500.0);
}

TEST_F(RenderTests, VisualTestRigidBodiesSlidingOnPlanes)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	Eigen::AngleAxisd aaTiltForward(0.15, Vector3d(1.0, 0.0, 0.0));
	Eigen::AngleAxisd aaTiltBackward(-0.15, Vector3d(1.0, 0.0, 0.0));

	// Mesh-base objects
	std::shared_ptr<SurgSim::Framework::SceneElement> boxMesh =
		createRigidMeshSceneElement("boxMesh", "box.ply");
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.3, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
		createRigidMeshSceneElement("sphereMesh", "sphere.ply");
	scene->addSceneElement(sphereMesh);
	sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.15, 0.3, 0.0)));

	// Shape-base objects
	std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
		createRigidSphereSceneElement("sphereShape", 0.05);
	scene->addSceneElement(sphereShape);
	sphereShape->setPose(makeRigidTranslation(Vector3d(0.15, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> boxShape =
		createRigidBoxSceneElement("boxShape", Vector3d(0.1, 0.1, 0.1));
	scene->addSceneElement(boxShape);
	boxShape->setPose(makeRigidTranslation(Vector3d(0.3, 0.3, 0.0)));

	// Floors on which the objects are falling (both tilted)
	std::shared_ptr<SurgSim::Framework::SceneElement> floor1 = createFixedPlaneSceneElement("floor1");
	scene->addSceneElement(floor1);
	floor1->setPose(makeRigidTransform(SurgSim::Math::Quaterniond(aaTiltForward), Vector3d(0.0, -0.2, -0.1)));

	std::shared_ptr<SurgSim::Framework::SceneElement> floor2 = createFixedPlaneSceneElement("floor2");
	scene->addSceneElement(floor2);
	floor2->setPose(makeRigidTransform(SurgSim::Math::Quaterniond(aaTiltBackward), Vector3d(0.0, -0.2, -0.1)));

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 13000.0);
}

TEST_F(RenderTests, VisualTestRigidBodiesStacking)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	const size_t numBodiesStacked = 3;

	// Mesh-base objects
	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "boxMesh" << i;
		double scale = 1.0 - static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> boxMesh =
			createRigidMeshSceneElement(ss.str(), "box.ply", scale);
		scene->addSceneElement(boxMesh);
		Eigen::AngleAxisd aa(0.15 * i, Vector3d(0.0, 1.0, 0.0));
		boxMesh->setPose(makeRigidTransform(SurgSim::Math::Quaterniond(aa), Vector3d(-0.3, 0.3 + 0.15 * i, 0.0)));
	}

	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "sphereMesh" << i;
		double scale = 1.0 - static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
			createRigidMeshSceneElement(ss.str(), "sphere.ply", scale);
		scene->addSceneElement(sphereMesh);
		sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.15, 0.3 + 0.15 * i, 0.0)));
	}

	// Shape-base objects
	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "sphereShape" << i;
		double scale = 1.0 - static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
			createRigidSphereSceneElement(ss.str(), 0.05 * scale);
		scene->addSceneElement(sphereShape);
		sphereShape->setPose(makeRigidTranslation(Vector3d(0.15, 0.3 + 0.15 * i, 0.0)));
	}

	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "boxShape" << i;
		double scale = 1.0 - static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> boxShape =
			createRigidBoxSceneElement(ss.str(), Vector3d::Ones() * 0.1 * scale);
		scene->addSceneElement(boxShape);
		boxShape->setPose(makeRigidTranslation(Vector3d(0.3, 0.3 + 0.15 * i, 0.0)));
	}

	// Floor on which the objects are falling
	std::shared_ptr<SurgSim::Framework::SceneElement> floor = createFixedPlaneSceneElement("floor");
	scene->addSceneElement(floor);
	floor->setPose(makeRigidTranslation(Vector3d(0.0, -0.2, 0.0)));

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 10000.0);
}

TEST_F(RenderTests, VisualTestRigidBodiesStackingReversed)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	const size_t numBodiesStacked = 3;

	// Mesh-base objects
	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "boxMesh" << i;
		double scale = 1.0 / static_cast<double>(numBodiesStacked) +
			static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> boxMesh =
			createRigidMeshSceneElement(ss.str(), "box.ply", scale);
		scene->addSceneElement(boxMesh);
		Eigen::AngleAxisd aa(0.15 * i, Vector3d(0.0, 1.0, 0.0));
		boxMesh->setPose(makeRigidTransform(SurgSim::Math::Quaterniond(aa), Vector3d(-0.3, 0.3 + 0.15 * i, 0.0)));
	}

	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "sphereMesh" << i;
		double scale = 1.0 / static_cast<double>(numBodiesStacked) +
			static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
			createRigidMeshSceneElement(ss.str(), "sphere.ply", scale);
		scene->addSceneElement(sphereMesh);
		sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.15, 0.3 + 0.15 * i, 0.0)));
	}

	// Shape-base objects
	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "sphereShape" << i;
		double scale = 1.0 / static_cast<double>(numBodiesStacked) +
			static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
			createRigidSphereSceneElement(ss.str(), 0.05 * scale);
		scene->addSceneElement(sphereShape);
		sphereShape->setPose(makeRigidTranslation(Vector3d(0.15, 0.3 + 0.15 * i, 0.0)));
	}

	for (size_t i = 0; i < numBodiesStacked; ++i)
	{
		std::stringstream ss;
		ss << "boxShape" << i;
		double scale = 1.0 / static_cast<double>(numBodiesStacked) +
			static_cast<double>(i) / static_cast<double>(numBodiesStacked);
		std::shared_ptr<SurgSim::Framework::SceneElement> boxShape =
			createRigidBoxSceneElement(ss.str(), Vector3d::Ones() * 0.1 * scale);
		scene->addSceneElement(boxShape);
		boxShape->setPose(makeRigidTranslation(Vector3d(0.3, 0.3 + 0.15 * i, 0.0)));
	}

	// Floor on which the objects are falling
	std::shared_ptr<SurgSim::Framework::SceneElement> floor = createFixedPlaneSceneElement("floor");
	scene->addSceneElement(floor);
	floor->setPose(makeRigidTranslation(Vector3d(0.0, -0.2, 0.0)));

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 10000.0);
}

}; // namespace Physics

}; // namespace SurgSim
