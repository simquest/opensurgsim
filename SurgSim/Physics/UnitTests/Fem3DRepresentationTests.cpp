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

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h" ///< Used to initialize the Component Fem3DRepresentation
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{

namespace Physics
{

class Fem3DRepresentationTests : public ::testing::Test
{
public:
	void SetUp() override
	{
		m_numNodes = 10;
	}

	void createFem()
	{
		m_fem = std::make_shared<Fem3DRepresentation>("Fem3d");

		m_initialState = std::make_shared<SurgSim::Math::OdeState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), m_numNodes);
		m_initialState->getPositions().setLinSpaced(1.23, 9.43);
		m_initialState->getVelocities().setLinSpaced(-0.94, 1.09);

		SurgSim::Math::Quaterniond q(1.0, 2.0, 3.0, 4.0);
		SurgSim::Math::Vector3d t(1.0, 2.0, 3.0);
		q.normalize();
		m_initialPose = SurgSim::Math::makeRigidTransform(q, t);
	}

	void addFemElement()
	{
		std::array<size_t, 4> elementNodeIds = {{0, 1, 2, 3}};
		std::shared_ptr<Fem3DElementTetrahedron> element = std::make_shared<Fem3DElementTetrahedron>(elementNodeIds);
		element->setYoungModulus(1e9);
		element->setPoissonRatio(0.45);
		element->setMassDensity(1000.0);
		m_fem->addFemElement(element);
	}

	void createLocalization()
	{
		SurgSim::DataStructures::IndexedLocalCoordinate femRepCoordinate;
		femRepCoordinate.index = 0;
		femRepCoordinate.coordinate = SurgSim::Math::Vector::Zero(4);
		femRepCoordinate.coordinate[0] = 1.0;
		m_localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem, femRepCoordinate);

		m_wrongLocalizationType = std::make_shared<MockLocalization>();
		m_wrongLocalizationType->setRepresentation(m_fem);
	}

protected:
	size_t m_numNodes;
	std::shared_ptr<Fem3DRepresentation> m_fem;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;
	SurgSim::Math::RigidTransform3d m_initialPose;
	std::shared_ptr<Fem3DRepresentationLocalization> m_localization;
	std::shared_ptr<MockLocalization> m_wrongLocalizationType;
};

TEST_F(Fem3DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW(std::shared_ptr<Fem3DRepresentation> fem = std::make_shared<Fem3DRepresentation>("Fem3D"));
}

TEST_F(Fem3DRepresentationTests, GetTypeTest)
{
	createFem();
	EXPECT_EQ(REPRESENTATION_TYPE_FEM3D, m_fem->getType());
}

TEST_F(Fem3DRepresentationTests, GetNumDofPerNodeTest)
{
	createFem();
	EXPECT_EQ(3u, m_fem->getNumDofPerNode());
}

TEST_F(Fem3DRepresentationTests, TransformInitialStateTest)
{
	using SurgSim::Math::Vector;

	createFem();
	m_fem->setLocalPose(m_initialPose);
	m_fem->setInitialState(m_initialState);

	Vector expectedX = Vector::Zero(m_fem->getNumDof()), expectedV = Vector::Zero(m_fem->getNumDof());
	for (size_t nodeId = 0; nodeId < m_numNodes; nodeId++)
	{
		expectedX.segment<3>(m_fem->getNumDofPerNode() * nodeId) =
			m_initialPose * m_initialState->getPosition(nodeId);
		expectedV.segment<3>(m_fem->getNumDofPerNode() * nodeId) =
			m_initialPose.linear() * m_initialState->getVelocity(nodeId);
	}

	// Initialize the component
	ASSERT_TRUE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	// Wake-up the component => apply the pose to the initial state
	ASSERT_TRUE(m_fem->wakeUp());

	EXPECT_TRUE(m_fem->getInitialState()->getPositions().isApprox(expectedX));
	EXPECT_TRUE(m_fem->getInitialState()->getVelocities().isApprox(expectedV));
}

TEST_F(Fem3DRepresentationTests, SetGetFilenameTest)
{
	createFem();
	ASSERT_NO_THROW(m_fem->setFilename("Data/PlyReaderTests/Tetrahedron.ply"));
	ASSERT_NO_THROW(m_fem->getFilename());
	ASSERT_EQ("Data/PlyReaderTests/Tetrahedron.ply", m_fem->getFilename());
}

TEST_F(Fem3DRepresentationTests, DoInitializeTest)
{
	{
		SCOPED_TRACE("Initialize with a valid file name");
		createFem();
		m_fem->setFilename("PlyReaderTests/Tetrahedron.ply");
		// Fem3DRepresentation::initialize() will call Fem3DRepresentation::doInitialize(), which should load the file.
		ASSERT_NO_THROW(ASSERT_TRUE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt"))));
		EXPECT_EQ(3u, m_fem->getNumDofPerNode());
		EXPECT_EQ(3u * 26u, m_fem->getNumDof());
		EXPECT_EQ(24u, m_fem->getInitialState()->getNumBoundaryConditions());
	}

	{
		SCOPED_TRACE("Initialize with an invalid file name");
		createFem();
		m_fem->setFilename("Non existent fake name");
		EXPECT_FALSE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Initialize with file name not set");
		createFem();
		// Fem3DRepresentation will not try to load file, but FemRepresentation::doInitialize() will throw.
		EXPECT_ANY_THROW(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
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
		createFem();
		m_fem->setFilename("PlyReaderTests/WrongPlyTetrahedron.ply");
		EXPECT_FALSE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}

	{
		SCOPED_TRACE("Loading file with incorrect data");
		createFem();
		m_fem->setFilename("PlyReaderTests/WrongDataTetrahedron.ply");
		EXPECT_ANY_THROW(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>("config.txt")));
	}
}

TEST_F(Fem3DRepresentationTests, CreateLocalizationTest)
{
	using SurgSim::DataStructures::EmptyData;

	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	createFem();
	ASSERT_NO_THROW(m_fem->setFilename("Geometry/wound_deformable.ply"));

	auto triangleMesh = std::make_shared<SurgSim::Math::MeshShape>();
	triangleMesh->load("Geometry/wound_deformable.ply");

	// Create the collision mesh for the surface of the finite element model
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	collisionRepresentation->setShape(triangleMesh);
	m_fem->setCollisionRepresentation(collisionRepresentation);

	bool wokeUp = false;
	ASSERT_TRUE(m_fem->initialize(runtime));
	EXPECT_NO_THROW(wokeUp = m_fem->wakeUp(););
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

		std::array<SurgSim::Math::Vector3d, 4> barycentricCoordinates = {SurgSim::Math::Vector3d::Ones() / 3.0,
																		 SurgSim::Math::Vector3d::UnitX(),
																		 SurgSim::Math::Vector3d::UnitY(),
																		 SurgSim::Math::Vector3d::UnitZ()};

		auto barycentricCoordinate = barycentricCoordinates.cbegin();
		for (auto point = points.cbegin(); point != points.cend(); ++point, ++barycentricCoordinate)
		{
			SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition(triangleId, *barycentricCoordinate);
			SurgSim::DataStructures::Location location(triangleLocalPosition);
			std::shared_ptr<SurgSim::Physics::Fem3DRepresentationLocalization> localization;

			EXPECT_NO_THROW(localization =
							std::dynamic_pointer_cast<SurgSim::Physics::Fem3DRepresentationLocalization>(
							m_fem->createLocalization(location)););
			EXPECT_TRUE(localization != nullptr);

			SurgSim::Math::Vector globalPosition;
			SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
			EXPECT_NO_THROW(globalPosition =
							m_fem->getFemElement(coordinate.index)->computeCartesianCoordinate(
							*m_fem->getCurrentState(), coordinate.coordinate););
			EXPECT_EQ(3, globalPosition.size());
			EXPECT_TRUE(globalPosition.isApprox(*point));
		}
	}
}

TEST_F(Fem3DRepresentationTests, ExternalForceAPITest)
{
	createFem();

	// External force vector not initialized until the initial state has been set (it contains the #dof...)
	EXPECT_EQ(0, m_fem->getExternalGeneralizedForce().size());
	EXPECT_EQ(0, m_fem->getExternalGeneralizedStiffness().rows());
	EXPECT_EQ(0, m_fem->getExternalGeneralizedStiffness().cols());
	EXPECT_EQ(0, m_fem->getExternalGeneralizedDamping().rows());
	EXPECT_EQ(0, m_fem->getExternalGeneralizedDamping().cols());

	m_fem->setInitialState(m_initialState);

	// Vector initialized (properly sized and zeroed)
	EXPECT_NE(0, m_fem->getExternalGeneralizedForce().size());
	EXPECT_NE(0, m_fem->getExternalGeneralizedStiffness().rows());
	EXPECT_NE(0, m_fem->getExternalGeneralizedStiffness().cols());
	EXPECT_NE(0, m_fem->getExternalGeneralizedDamping().rows());
	EXPECT_NE(0, m_fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(m_fem->getNumDof(), m_fem->getExternalGeneralizedForce().size());
	EXPECT_EQ(m_fem->getNumDof(), m_fem->getExternalGeneralizedStiffness().cols());
	EXPECT_EQ(m_fem->getNumDof(), m_fem->getExternalGeneralizedStiffness().rows());
	EXPECT_EQ(m_fem->getNumDof(), m_fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(m_fem->getNumDof(), m_fem->getExternalGeneralizedDamping().rows());
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isZero());
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isZero());
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isZero());

	addFemElement();
	createLocalization();

	Vector FLocalWrongSize = Vector::Ones(2 * m_fem->getNumDofPerNode());
	Matrix KLocalWrongSize = Matrix::Ones(3 * m_fem->getNumDofPerNode(), 3 * m_fem->getNumDofPerNode());
	Matrix DLocalWrongSize = Matrix::Ones(4 * m_fem->getNumDofPerNode(), 4 * m_fem->getNumDofPerNode());
	Vector Flocal = Vector::LinSpaced(m_fem->getNumDofPerNode(), -3.12, 4.09);
	Matrix Klocal = Matrix::Ones(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) * 0.34;
	Matrix Dlocal = Klocal + Matrix::Identity(m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode());
	Vector F = Vector::Zero(m_fem->getNumDof());
	F.segment(0, m_fem->getNumDofPerNode()) = Flocal;
	Matrix K = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
	K.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = Klocal;
	Matrix D = Matrix::Zero(m_fem->getNumDof(), m_fem->getNumDof());
	D.block(0, 0, m_fem->getNumDofPerNode(), m_fem->getNumDofPerNode()) = Dlocal;

	// Test invalid localization nullptr
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(nullptr, Flocal),
		SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(nullptr, Flocal, Klocal, Dlocal),
		SurgSim::Framework::AssertionFailure);
	// Test invalid localization type
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_wrongLocalizationType, Flocal),
		SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_wrongLocalizationType, Flocal, Klocal, Dlocal),
		SurgSim::Framework::AssertionFailure);
	// Test invalid force size
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_localization, FLocalWrongSize),
		SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_localization, FLocalWrongSize, Klocal, Dlocal),
		SurgSim::Framework::AssertionFailure);
	// Test invalid stiffness size
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_localization, Flocal, KLocalWrongSize, Dlocal),
		SurgSim::Framework::AssertionFailure);
	// Test invalid damping size
	ASSERT_THROW(m_fem->addExternalGeneralizedForce(m_localization, Flocal, Klocal, DLocalWrongSize),
		SurgSim::Framework::AssertionFailure);

	// Test valid call to addExternalGeneralizedForce
	m_fem->addExternalGeneralizedForce(m_localization, Flocal, Klocal, Dlocal);
	EXPECT_FALSE(m_fem->getExternalGeneralizedForce().isZero());
	EXPECT_FALSE(m_fem->getExternalGeneralizedStiffness().isZero());
	EXPECT_FALSE(m_fem->getExternalGeneralizedDamping().isZero());
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isApprox(F));
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isApprox(K));
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isApprox(D));

	// Test valid call to addExternalGeneralizedForce to add things up
	m_fem->addExternalGeneralizedForce(m_localization, Flocal, Klocal, Dlocal);
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isApprox(2.0 * F));
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isApprox(2.0 * K));
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isApprox(2.0 * D));
}

TEST_F(Fem3DRepresentationTests, SerializationTest)
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
	ASSERT_NO_THROW(newRepresentation = std::dynamic_pointer_cast<Fem3DRepresentation>(
											node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	EXPECT_EQ("SurgSim::Physics::Fem3DRepresentation", newRepresentation->getClassName());
	EXPECT_EQ(filename, newRepresentation->getValue<std::string>("Filename"));
}

} // namespace Physics
} // namespace SurgSim
