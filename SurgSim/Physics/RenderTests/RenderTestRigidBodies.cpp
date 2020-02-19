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

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/Mesh.h"
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

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Graphics::OsgBoxRepresentation;
using SurgSim::Graphics::OsgMeshRepresentation;
using SurgSim::Graphics::OsgPlaneRepresentation;
using SurgSim::Graphics::OsgSphereRepresentation;
using SurgSim::Math::BoxShape;
using SurgSim::Math::MeshShape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::SphereShape;
using SurgSim::Math::SurfaceMeshShape;
using SurgSim::Math::Vector3d;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::RigidCollisionRepresentation;
using SurgSim::Physics::RigidRepresentation;

namespace
{

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidSphereSceneElement(const std::string& name, double radius)
{
	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>("Physics");
	std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(radius);
	physicsRepresentation->setShape(shape);
	physicsRepresentation->setDensity(750.0); // Average mass density of Oak Wood
	physicsRepresentation->setLinearDamping(1e-2);
	physicsRepresentation->setAngularDamping(1e-4);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgSphereRepresentation> osgRepresentation =
		std::make_shared<OsgSphereRepresentation>("OsgRepresentation");
	osgRepresentation->setRadius(radius);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidBoxSceneElement(const std::string& name, Vector3d size)
{
	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>("Physics");
	std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(size[0], size[1], size[2]);
	physicsRepresentation->setShape(shape);
	physicsRepresentation->setDensity(750.0); // Average mass density of oak wood
	physicsRepresentation->setLinearDamping(1e-2);
	physicsRepresentation->setAngularDamping(1e-4);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgBoxRepresentation> osgRepresentation =
		std::make_shared<OsgBoxRepresentation>("OsgRepresentation");
	osgRepresentation->setSize(size);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createFixedPlaneSceneElement(const std::string& name)
{
	// Physics representation
	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>("Physics");
	std::shared_ptr<PlaneShape> shape = std::make_shared<PlaneShape>();
	physicsRepresentation->setShape(shape);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic model
	std::shared_ptr<OsgPlaneRepresentation> osgRepresentation =
		std::make_shared<OsgPlaneRepresentation>("OsgRepresentation");

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createRigidMeshSceneElement(
	const std::string& name, std::string plyFilename, double scale = 1.0)
{
	auto mesh = std::make_shared<MeshShape>();
	mesh->load(plyFilename);
	for (auto& vertex : mesh->getVertices())
	{
		vertex.position *= scale;
	}

	// Physics representation
	std::shared_ptr<RigidRepresentation> physicsRepresentation =
		std::make_shared<RigidRepresentation>("Physics");
	physicsRepresentation->setShape(mesh);
	physicsRepresentation->setDensity(750.0); // Average mass density of oak wood
	physicsRepresentation->setLinearDamping(1e-2);
	physicsRepresentation->setAngularDamping(1e-4);

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic representation of the physics model
	std::shared_ptr<OsgMeshRepresentation> osgRepresentation =
		std::make_shared<OsgMeshRepresentation>("OsgRepresentation");
	osgRepresentation->setShape(mesh);
	osgRepresentation->setDrawAsWireFrame(true);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createFixedMeshSceneElement(
	const std::string& name, std::string plyFilename, double scale = 1.0)
{
	auto mesh = std::make_shared<MeshShape>();
	mesh->load(plyFilename);
	for (auto& vertex : mesh->getVertices())
	{
		vertex.position *= scale;
	}

	// Physics representation
	auto physicsRepresentation = std::make_shared<FixedRepresentation>("Physics");
	physicsRepresentation->setShape(mesh);
	physicsRepresentation->setDensity(750.0); // Average mass density of oak wood

	// Collision representation
	auto collisionRepresentation = std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic representation of the physics model
	auto osgRepresentation = std::make_shared<OsgMeshRepresentation>("OsgRepresentation");
	osgRepresentation->setShape(mesh);
	osgRepresentation->setDrawAsWireFrame(true);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
	sceneElement->addComponent(osgRepresentation);
	sceneElement->addComponent(collisionRepresentation);
	sceneElement->addComponent(physicsRepresentation);

	return sceneElement;
}

std::shared_ptr<SurgSim::Framework::SceneElement> createFixedSurfaceMeshSceneElement(
	const std::string& name, std::string plyFilename, double scale = 1.0)
{
	auto mesh = std::make_shared<SurfaceMeshShape>();
	mesh->load(plyFilename);
	for (auto& vertex : mesh->getVertices())
	{
		vertex.position *= scale;
	}

	// Physics representation
	std::shared_ptr<FixedRepresentation> physicsRepresentation =
		std::make_shared<FixedRepresentation>("Physics");
	physicsRepresentation->setShape(mesh);
	physicsRepresentation->setDensity(750.0); // Average mass density of oak wood

	// Collision representation
	std::shared_ptr<RigidCollisionRepresentation> collisionRepresentation =
		std::make_shared<RigidCollisionRepresentation>("Collision");
	physicsRepresentation->setCollisionRepresentation(collisionRepresentation);

	// Graphic representation of the physics model
	std::shared_ptr<OsgMeshRepresentation> osgRepresentation =
		std::make_shared<OsgMeshRepresentation>("OsgRepresentation");
	osgRepresentation->setShape(mesh);
	osgRepresentation->setDrawAsWireFrame(true);

	std::shared_ptr<BasicSceneElement> sceneElement = std::make_shared<BasicSceneElement>(name);
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
		createRigidMeshSceneElement("boxMesh1", "Geometry/box.ply", 1.0);
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.35, 0.3, 0.0)));

	boxMesh = createRigidMeshSceneElement("boxMesh2", "Geometry/box.ply", 0.5);
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.25, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
		createRigidMeshSceneElement("sphereMesh1", "Geometry/sphere.ply", 1.0);
	scene->addSceneElement(sphereMesh);
	sphereMesh->setPose(makeRigidTranslation(Vector3d(-0.15, 0.3, 0.0)));

	sphereMesh = createRigidMeshSceneElement("sphereMesh2", "Geometry/sphere.ply", 0.5);
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
		createRigidMeshSceneElement("boxMesh", "Geometry/box.ply");
	scene->addSceneElement(boxMesh);
	boxMesh->setPose(makeRigidTranslation(Vector3d(-0.3, 0.3, 0.0)));

	std::shared_ptr<SurgSim::Framework::SceneElement> sphereMesh =
		createRigidMeshSceneElement("sphereMesh", "Geometry/sphere.ply");
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
			createRigidMeshSceneElement(ss.str(), "Geometry/box.ply", scale);
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
			createRigidMeshSceneElement(ss.str(), "Geometry/sphere.ply", scale);
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
			createRigidMeshSceneElement(ss.str(), "Geometry/box.ply", scale);
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
			createRigidMeshSceneElement(ss.str(), "Geometry/sphere.ply", scale);
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

TEST_F(RenderTests, VisualTestFallingRigidShapesOnPlane)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	const size_t numSphere = 10;
	const double radius = 0.05;
	const double distanceBetweenSphere = radius / 2.0;

	// Shape-base objects
	for (size_t sphere = 0; sphere < numSphere; ++sphere)
	{
		std::stringstream ss;
		ss << "sphereShape " << sphere;
		std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
			createRigidSphereSceneElement(ss.str(), radius);
		scene->addSceneElement(sphereShape);
		sphereShape->setPose(makeRigidTranslation(Vector3d(0.0, (2.0 * radius + distanceBetweenSphere) * sphere, 0.0)));
	}

	// Floor on which the objects are falling
	std::shared_ptr<SurgSim::Framework::SceneElement> floor = createFixedPlaneSceneElement("floor");
	scene->addSceneElement(floor);
	floor->setPose(makeRigidTranslation(Vector3d(0.0, -radius - distanceBetweenSphere, 0.0)));

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 10000.0);
}

TEST_F(RenderTests, VisualTestFallingRigidMeshesOnPlane)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	const size_t numSphere = 10;
	const double radius = 0.05;
	const double distanceBetweenSphere = radius / 2.0;

	double xAxis[2] = {-0.26, 0.26};
	double xAngle[2] = {-M_PI_2, M_PI_2};

	for (int i = 0; i < 2; ++i)
	{
		// Mesh-base objects
		for (size_t sphere = 0; sphere < numSphere; ++sphere)
		{
			std::stringstream ss;
			ss << "sphereShape " << sphere;
			std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
				createRigidMeshSceneElement(ss.str(), "Geometry/sphere.ply", 10.0 * radius);
			scene->addSceneElement(sphereShape);
			sphereShape->setPose(
				makeRigidTranslation(Vector3d(xAxis[i], (2.0 * radius + distanceBetweenSphere) * sphere, 0.0)));
		}

		// Floor on which the objects are falling
		{
			std::shared_ptr<SurgSim::Framework::SceneElement> planeMesh =
				createFixedSurfaceMeshSceneElement("floor", "Geometry/plane.ply", 0.25);
			scene->addSceneElement(planeMesh);
			Eigen::AngleAxisd aa(xAngle[i], Vector3d(1.0, 0.0, 0.0));
			planeMesh->setPose(
				makeRigidTransform(SurgSim::Math::Quaterniond(aa),
								   Vector3d(xAxis[i], -radius - distanceBetweenSphere, 0.0)));
		}
	}

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 10000.0);
}

TEST_F(RenderTests, VisualTestFallingSphereOnMesh)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;

	const size_t numSphere = 10;
	const double radius = 0.05;
	const double distanceBetweenSphere = radius / 2.0;

	double xAxis[2] = { -0.26, 0.26 };
	double xAngle[2] = { -M_PI_2, M_PI_2 };

	for (int i = 0; i < 2; ++i)
	{
		// Mesh-base objects
		for (size_t sphere = 0; sphere < numSphere; ++sphere)
		{
			std::stringstream ss;
			ss << "sphereShape " << sphere;
			std::shared_ptr<SurgSim::Framework::SceneElement> sphereShape =
				createRigidSphereSceneElement(ss.str(), radius);
			scene->addSceneElement(sphereShape);
			sphereShape->setPose(
				makeRigidTranslation(Vector3d(xAxis[i], (2.0 * radius + distanceBetweenSphere) * sphere, 0.0)));
		}

		// Floor on which the objects are falling
		{
			auto planeMesh = createFixedMeshSceneElement("floor", "Geometry/plane.ply", 0.25);
			scene->addSceneElement(planeMesh);
			Eigen::AngleAxisd aa(xAngle[i], Vector3d(1.0, 0.0, 0.0));
			planeMesh->setPose(
				makeRigidTransform(SurgSim::Math::Quaterniond(aa),
					Vector3d(xAxis[i], -radius - distanceBetweenSphere, 0.0)));
		}
	}

	runTest(Vector3d(0.0, 0.0, 1.0), Vector3d::Zero(), 10000.0);
}
}; // namespace Physics

}; // namespace SurgSim
