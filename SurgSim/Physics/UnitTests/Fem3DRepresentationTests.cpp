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

/// \file Fem3DRepresentationTests.cpp
/// This file tests the non-abstract functionalities of the base class FemRepresentation

#include <gtest/gtest.h>

#include "SurgSim/Collision/Location.h"
#include "SurgSim/DataStructures/PlyReader.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/DataStructures/TriangleMeshUtilities.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h" //< Used to initialize the Component Fem3DRepresentation
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"

namespace SurgSim
{

namespace Physics
{

TEST(Fem3DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW(std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("Fem3D"));
}

TEST(Fem3DRepresentationTests, GetTypeTest)
{
	std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("Fem3D");
	EXPECT_EQ(REPRESENTATION_TYPE_FEM3D, fem->getType());
}

TEST(Fem3DRepresentationTests, GetNumDofPerNodeTest)
{
	std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("Fem3D");
	EXPECT_EQ(3u, fem->getNumDofPerNode());
}

TEST(Fem3DRepresentationTests, TransformInitialStateTest)
{
	using SurgSim::Math::Vector;

	std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("Fem3D");

	const size_t numNodes = 4;
	const size_t numDofPerNode = fem->getNumDofPerNode();
	const size_t numDof = numDofPerNode * numNodes;

	SurgSim::Math::RigidTransform3d initialPose;
	SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
	SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
	q.normalize();
	initialPose = SurgSim::Math::makeRigidTransform(q, t);
	fem->setLocalPose(initialPose);

	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(numDofPerNode, numNodes);
	Vector x = Vector::LinSpaced(numDof, 1.0, static_cast<double>(numDof));
	Vector v = Vector::Ones(numDof);
	initialState->getPositions() = x;
	initialState->getVelocities() = v;
	fem->setInitialState(initialState);

	Vector expectedX = x, expectedV = v;
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		expectedX.segment<3>(numDofPerNode * nodeId) = initialPose * x.segment<3>(numDofPerNode * nodeId);
		expectedV.segment<3>(numDofPerNode * nodeId) = initialPose.linear() * v.segment<3>(numDofPerNode * nodeId);
	}

	// Initialize the component
	ASSERT_TRUE(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	// Wake-up the component => apply the pose to the initial state
	ASSERT_TRUE(fem->wakeUp());

	EXPECT_TRUE(fem->getInitialState()->getPositions().isApprox(expectedX));
	EXPECT_TRUE(fem->getInitialState()->getVelocities().isApprox(expectedV));
}

TEST(Fem3DRepresentationTests, SetGetFilenameTest)
{
	auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

	ASSERT_NO_THROW(fem->setFilename("Data/PlyReaderTests/Tetrahedron.ply"));
	ASSERT_NO_THROW(fem->getFilename());
	ASSERT_EQ("Data/PlyReaderTests/Tetrahedron.ply", fem->getFilename());
}

TEST(Fem3DRepresentationTests, DoInitializeTest)
{
	{
		SCOPED_TRACE("Initialize with a valid file name");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");
		fem->setFilename("PlyReaderTests/Tetrahedron.ply");

		// Fem3DRepresentation::initialize() will call Fem3DRepresentation::doInitialize(), which should load the file.
		ASSERT_NO_THROW(ASSERT_TRUE(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt"))));

		EXPECT_EQ(3u, fem->getNumDofPerNode());
		EXPECT_EQ(3u * 26u, fem->getNumDof());
		EXPECT_EQ(24u, fem->getInitialState()->getNumBoundaryConditions());
	}

	{
		SCOPED_TRACE("Initialize with an invalid file name");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");
		fem->setFilename("Non existent fake name");
		EXPECT_FALSE(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Initialize with file name not set");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");
		// Fem3DRepresentation will not try to load file, but FemRepresentation::doInitialize() will throw.
		EXPECT_ANY_THROW(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Initialization called on object instance");
		Fem3DRepresentation fem("fem3d");

		fem.setFilename("PlyReaderTests/Tetrahedron.ply");
		// It throws because within doInitialize(), 'this' Fem3DRepresentation will be passed as a shared_ptr<> to
		// the Fem3DRepresentationPlyReaderDelegate.
		EXPECT_ANY_THROW(fem.initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Loading file with incorrect PLY format");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		fem->setFilename("PlyReaderTests/WrongPlyTetrahedron.ply");
		EXPECT_FALSE(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Loading file with incorrect data");
		auto fem = std::make_shared<Fem3DRepresentation>("fem3d");

		fem->setFilename("PlyReaderTests/WrongDataTetrahedron.ply");
		EXPECT_ANY_THROW(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}
}

TEST(Fem3DRepresentationTests, CreateLocalizationTest)
{
	using SurgSim::DataStructures::EmptyData;

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto fem = std::make_shared<Fem3DRepresentation>("fem3d");
	ASSERT_NO_THROW(fem->setFilename("Geometry/wound_deformable.ply"));

	std::string path = runtime->getApplicationData()->findFile("Geometry/wound_deformable.ply");
	std::shared_ptr<SurgSim::DataStructures::TriangleMeshBase<EmptyData, EmptyData, EmptyData>> triangleMesh =
				SurgSim::DataStructures::loadTriangleMesh(path);

	// Create the collision mesh for the surface of the finite element model
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	collisionRepresentation->setMesh(std::make_shared<SurgSim::DataStructures::TriangleMesh>(*triangleMesh));
	fem->setCollisionRepresentation(collisionRepresentation);

	bool wokeUp = false;
	ASSERT_TRUE(fem->initialize(runtime));
	EXPECT_NO_THROW(wokeUp = fem->wakeUp(););
	EXPECT_TRUE(wokeUp);

	auto& meshTriangles = triangleMesh->getTriangles();
	size_t triangleId = 0;
	SurgSim::Math::Vector3d centroid;
	for (auto triangle = meshTriangles.cbegin(); triangle != meshTriangles.cend(); ++triangle, ++triangleId)
	{
		std::array<size_t, 3> triangleNodeIds = triangle->verticesId;
		centroid = triangleMesh->getVertexPosition(triangleNodeIds[0]);
		centroid += triangleMesh->getVertexPosition(triangleNodeIds[1]);
		centroid += triangleMesh->getVertexPosition(triangleNodeIds[2]);
		centroid /= 3.0;

		// Test the localization with each of the triangle vertices and the triangle centroid.
		std::array<SurgSim::Math::Vector3d, 4> points = {centroid,
														 triangleMesh->getVertexPosition(triangleNodeIds[0]),
														 triangleMesh->getVertexPosition(triangleNodeIds[1]),
														 triangleMesh->getVertexPosition(triangleNodeIds[2])
														};

		for (auto point = points.cbegin(); point != points.cend(); ++point)
		{
			SurgSim::Collision::Location location;
			std::shared_ptr<SurgSim::Physics::Fem3DRepresentationLocalization> localization;

			location.triangleId.setValue(triangleId);
			location.globalPosition.setValue(*point);
			EXPECT_NO_THROW(localization =
								std::dynamic_pointer_cast<SurgSim::Physics::Fem3DRepresentationLocalization>(
									fem->createLocalization(location)););
			EXPECT_TRUE(localization != nullptr);

			SurgSim::Math::Vector globalPosition;
			SurgSim::Physics::FemRepresentationCoordinate coordinate = localization->getLocalPosition();
			EXPECT_NO_THROW(globalPosition =
								fem->getFemElement(coordinate.elementId)->computeCartesianCoordinate(*fem->getCurrentState(),
										coordinate.naturalCoordinate););
			EXPECT_EQ(3, globalPosition.size());
			EXPECT_TRUE(globalPosition.isApprox(*point));
		}
	}
}

TEST(Fem3DRepresentationTests, SerializationTest)
{
	auto fem3DRepresentation = std::make_shared<SurgSim::Physics::Fem3DRepresentation>("Test-Fem3D");
	const std::string filename = "TestFilename";
	fem3DRepresentation->setFilename(filename);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	fem3DRepresentation->setCollisionRepresentation(collisionRepresentation);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*fem3DRepresentation));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	YAML::Node data = node["SurgSim::Physics::Fem3DRepresentation"];
	EXPECT_EQ(10u, data.size());

	std::shared_ptr<Fem3DRepresentation> newRepresentation;
	ASSERT_NO_THROW(newRepresentation =
						std::dynamic_pointer_cast<Fem3DRepresentation>(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ("SurgSim::Physics::Fem3DRepresentation", newRepresentation->getClassName());
	EXPECT_EQ(filename, newRepresentation->getValue<std::string>("Filename"));
}

} // namespace Physics
} // namespace SurgSim
