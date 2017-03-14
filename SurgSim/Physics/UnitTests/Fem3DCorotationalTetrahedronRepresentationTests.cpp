// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

/// \file Fem3DCorotationalTetrahedronRepresentationTests.cpp
/// This file tests the non-abstract functionalities of the Fem3DCorotationalTetrahedronRepresentation class

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Framework/Runtime.h" ///< Used to initialize the Component Fem3DCorotationalTetrahedronRepresentation
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem3DElementCorotationalTetrahedron.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DCorotationalTetrahedronRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{

namespace Physics
{

class Fem3DCorotationalTetrahedronRepresentationTests : public ::testing::Test
{
public:
	void SetUp() override
	{
		m_numNodes = 10;
	}

	void createFem()
	{
		m_fem = std::make_shared<Fem3DCorotationalTetrahedronRepresentation>("Fem3d");

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
		std::shared_ptr<Fem3DElementCorotationalTetrahedron> element = std::make_shared<Fem3DElementCorotationalTetrahedron>(elementNodeIds);
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
		m_localization = std::make_shared<Fem3DLocalization>(m_fem, femRepCoordinate);

		m_wrongLocalizationType = std::make_shared<MockLocalization>();
		m_wrongLocalizationType->setRepresentation(m_fem);
	}

protected:
	size_t m_numNodes;
	std::shared_ptr<Fem3DCorotationalTetrahedronRepresentation> m_fem;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;
	SurgSim::Math::RigidTransform3d m_initialPose;
	std::shared_ptr<Fem3DLocalization> m_localization;
	std::shared_ptr<MockLocalization> m_wrongLocalizationType;
};

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW(std::shared_ptr<Fem3DCorotationalTetrahedronRepresentation> fem =
							std::make_shared<Fem3DCorotationalTetrahedronRepresentation>("Fem3D"));
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, GetNumDofPerNodeTest)
{
	createFem();
	EXPECT_EQ(3u, m_fem->getNumDofPerNode());
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, TransformInitialStateTest)
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

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, DoInitializeTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	{
		SCOPED_TRACE("Initialize with a valid file name");
		createFem();
		m_fem->loadFem("PlyReaderTests/Tetrahedron.ply");
		// Fem3DRepresentation::initialize() will call Fem3DRepresentation::doInitialize(), which should load the file.
		ASSERT_NO_THROW(ASSERT_TRUE(m_fem->initialize(runtime)));
		EXPECT_EQ(3u, m_fem->getNumDofPerNode());
		EXPECT_EQ(3u * 26u, m_fem->getNumDof());
		EXPECT_EQ(12u, m_fem->getInitialState()->getNumBoundaryConditions());
	}

	{
		SCOPED_TRACE("Initialize with an invalid file name");
		createFem();
		EXPECT_ANY_THROW(m_fem->loadFem("Non existent fake name"));
	}

	{
		SCOPED_TRACE("Initialize with file name not set");
		createFem();
		// Fem3DRepresentation will not try to load file, but fem3DRepresentation::doInitialize() will throw.
		EXPECT_ANY_THROW(m_fem->initialize(runtime));
	}

	{
		SCOPED_TRACE("Loading file with incorrect PLY format");
		createFem();
		EXPECT_ANY_THROW(m_fem->loadFem("PlyReaderTests/WrongPlyTetrahedron.ply"));
	}

	{
		SCOPED_TRACE("Loading file with incorrect data");
		createFem();
		EXPECT_ANY_THROW(m_fem->loadFem("PlyReaderTests/WrongDataTetrahedron.ply"););
	}
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, CreateLocalizationTest)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	ASSERT_NO_THROW(createFem());
	ASSERT_NO_THROW(m_fem->loadFem("Geometry/wound_deformable.ply"));

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
																		 SurgSim::Math::Vector3d::UnitZ()
																		};

		auto barycentricCoordinate = barycentricCoordinates.cbegin();
		for (auto point = points.cbegin(); point != points.cend(); ++point, ++barycentricCoordinate)
		{
			SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition(triangleId, *barycentricCoordinate);
			SurgSim::DataStructures::Location location(triangleLocalPosition,
				SurgSim::DataStructures::Location::TRIANGLE);
			std::shared_ptr<SurgSim::Physics::Fem3DLocalization> localization;
			EXPECT_NO_THROW(localization =
				std::dynamic_pointer_cast<SurgSim::Physics::Fem3DLocalization>(m_fem->createLocalization(location)););
			EXPECT_TRUE(localization != nullptr);
			EXPECT_TRUE(localization->getRepresentation() == m_fem);

			SurgSim::Math::Vector globalPosition;
			SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
			EXPECT_NO_THROW(globalPosition =
								m_fem->getFemElement(coordinate.index)->computeCartesianCoordinate(
									*m_fem->getCurrentState(), coordinate.coordinate););
			EXPECT_EQ(3, globalPosition.size());
			EXPECT_TRUE(globalPosition.isApprox(*point));
		}
	}

	// Localization on an invalid node
	{
		SCOPED_TRACE("Invalid node");

		SurgSim::DataStructures::Location location(1000);
		std::shared_ptr<SurgSim::Physics::Localization> localization;
		EXPECT_THROW(localization = m_fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
	}

	// Localization on a valid node
	{
		SCOPED_TRACE("Valid node");

		SurgSim::DataStructures::Location location(0);
		std::shared_ptr<SurgSim::Physics::Fem3DLocalization> localization;
		EXPECT_NO_THROW(localization =
			std::dynamic_pointer_cast<SurgSim::Physics::Fem3DLocalization>(
			m_fem->createLocalization(location)););
		EXPECT_TRUE(localization != nullptr);
		EXPECT_TRUE(localization->getRepresentation() == m_fem);

		SurgSim::Math::Vector3d globalPosition;
		SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
		EXPECT_NO_THROW(globalPosition = m_fem->getCurrentState()->getPosition(coordinate.index););
		EXPECT_TRUE(globalPosition.isApprox(localization->calculatePosition()));
	}

	// Localization on an invalid tetrahedron
	{
		SCOPED_TRACE("Invalid tetrahedron index");

		Vector barycentricCoordinates = Vector::Zero(4);
		barycentricCoordinates[0] = 1.0;
		SurgSim::DataStructures::IndexedLocalCoordinate coord(10000, barycentricCoordinates);
		SurgSim::DataStructures::Location location(coord, SurgSim::DataStructures::Location::ELEMENT);
		std::shared_ptr<SurgSim::Physics::Localization> localization;
		EXPECT_THROW(localization = m_fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
	}

	// Localization on an valid tetrahedron but invalid barycentric coordinate size
	{
		SCOPED_TRACE("Invalid tetrahedron barycentric coordinate size");

		Vector barycentricCoordinates = Vector::Zero(5);
		barycentricCoordinates[0] = 1.0;
		SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);
		SurgSim::DataStructures::Location location(coord, SurgSim::DataStructures::Location::ELEMENT);
		std::shared_ptr<SurgSim::Physics::Localization> localization;
		EXPECT_THROW(localization = m_fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
	}

	// Localization on an valid tetrahedron but invalid barycentric coordinate
	{
		SCOPED_TRACE("Invalid tetrahedron barycentric coordinate");

		Vector barycentricCoordinates = Vector::Ones(4);
		SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);
		SurgSim::DataStructures::Location location(coord, SurgSim::DataStructures::Location::ELEMENT);
		std::shared_ptr<SurgSim::Physics::Localization> localization;
		EXPECT_THROW(localization = m_fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
	}

	// Localization on a valid tetrahedron
	{
		SCOPED_TRACE("Valid tetrahedron");

		Vector barycentricCoordinates = Vector::Zero(4);
		barycentricCoordinates.setConstant(1.0 / 4.0);
		SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);
		SurgSim::DataStructures::Location location(coord, SurgSim::DataStructures::Location::ELEMENT);
		std::shared_ptr<SurgSim::Physics::Fem3DLocalization> localization;
		EXPECT_NO_THROW(localization =
			std::dynamic_pointer_cast<SurgSim::Physics::Fem3DLocalization>(m_fem->createLocalization(location)));
		EXPECT_TRUE(localization != nullptr);
		EXPECT_TRUE(localization->getRepresentation() == m_fem);

		SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
		auto element = m_fem->getFemElement(coordinate.index);
		SurgSim::Math::Vector3d globalPosition =
			1.0 / 4.0 * m_fem->getCurrentState()->getPosition(element->getNodeId(0)) +
			1.0 / 4.0 * m_fem->getCurrentState()->getPosition(element->getNodeId(1)) +
			1.0 / 4.0 * m_fem->getCurrentState()->getPosition(element->getNodeId(2)) +
			1.0 / 4.0 * m_fem->getCurrentState()->getPosition(element->getNodeId(3));
		EXPECT_TRUE(globalPosition.isApprox(localization->calculatePosition()));
	}
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, ExternalForceAPITest)
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
	Math::SparseMatrix zeroMatrix(static_cast<SparseMatrix::Index>(m_fem->getNumDof()),
								  static_cast<SparseMatrix::Index>(m_fem->getNumDof()));
	zeroMatrix.setZero();
	EXPECT_NE(0, m_fem->getExternalGeneralizedForce().size());
	EXPECT_NE(0, m_fem->getExternalGeneralizedStiffness().rows());
	EXPECT_NE(0, m_fem->getExternalGeneralizedStiffness().cols());
	EXPECT_NE(0, m_fem->getExternalGeneralizedDamping().rows());
	EXPECT_NE(0, m_fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(static_cast<Math::Vector6d::Index>(m_fem->getNumDof()), m_fem->getExternalGeneralizedForce().size());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(m_fem->getNumDof()), m_fem->getExternalGeneralizedStiffness().cols());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(m_fem->getNumDof()), m_fem->getExternalGeneralizedStiffness().rows());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(m_fem->getNumDof()), m_fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(m_fem->getNumDof()), m_fem->getExternalGeneralizedDamping().rows());
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isZero());
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isApprox(zeroMatrix));
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isApprox(zeroMatrix));

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
	EXPECT_FALSE(m_fem->getExternalGeneralizedStiffness().isApprox(zeroMatrix));
	EXPECT_FALSE(m_fem->getExternalGeneralizedDamping().isApprox(zeroMatrix));
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isApprox(F));
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isApprox(K));
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isApprox(D));

	// Test valid call to addExternalGeneralizedForce to add things up
	m_fem->addExternalGeneralizedForce(m_localization, Flocal, Klocal, Dlocal);
	EXPECT_TRUE(m_fem->getExternalGeneralizedForce().isApprox(2.0 * F));
	EXPECT_TRUE(m_fem->getExternalGeneralizedStiffness().isApprox(2.0 * K));
	EXPECT_TRUE(m_fem->getExternalGeneralizedDamping().isApprox(2.0 * D));
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, LoadMeshTest)
{
	auto fem = std::make_shared<Fem3DCorotationalTetrahedronRepresentation>("Representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	fem->loadFem("PlyReaderTests/Tetrahedron.ply");
	fem->initialize(runtime);

	// Vertices
	ASSERT_EQ(3u, fem->getNumDofPerNode());
	ASSERT_EQ(3u * 26u, fem->getNumDof());

	Vector3d vertex0(-1.0, 0.0, 0.0);
	Vector3d vertex25(1.0, 1.0, 1.0);

	EXPECT_TRUE(vertex0.isApprox(fem->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex25.isApprox(fem->getInitialState()->getPosition(25)));

	// Tetrahedrons
	ASSERT_EQ(44u, fem->getNumFemElements());

	std::array<size_t, 4> tetrahedron0 = {16, 0, 7, 5};
	std::array<size_t, 4> tetrahedron2 = {13, 14, 16, 2};

	EXPECT_TRUE(std::equal(std::begin(tetrahedron0), std::end(tetrahedron0),
						   std::begin(fem->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(tetrahedron2), std::end(tetrahedron2),
						   std::begin(fem->getFemElement(11)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(12u, fem->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8
	size_t boundaryNode0 = 19;
	size_t boundaryNode7 = 25;

	EXPECT_EQ(3 * boundaryNode0, fem->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(3 * boundaryNode0 + 1, fem->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(3 * boundaryNode0 + 2, fem->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(3 * boundaryNode7, fem->getInitialState()->getBoundaryConditions().at(9));
	EXPECT_EQ(3 * boundaryNode7 + 1, fem->getInitialState()->getBoundaryConditions().at(10));
	EXPECT_EQ(3 * boundaryNode7 + 2, fem->getInitialState()->getBoundaryConditions().at(11));

	// Material
	auto fem2 = fem->getFemElement(2);
	EXPECT_DOUBLE_EQ(900.0, fem2->getMassDensity());
	EXPECT_DOUBLE_EQ(0.45, fem2->getPoissonRatio());
	EXPECT_DOUBLE_EQ(1750000000, fem2->getYoungModulus());

	auto fem8 = fem->getFemElement(8);
	EXPECT_DOUBLE_EQ(900.0, fem8->getMassDensity());
	EXPECT_DOUBLE_EQ(0.45, fem8->getPoissonRatio());
	EXPECT_DOUBLE_EQ(1750000000, fem8->getYoungModulus());
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, SerializationTest)
{
	auto fem = std::make_shared<Fem3DCorotationalTetrahedronRepresentation>("Test-Fem3D");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string filename = "PlyReaderTests/Tetrahedron.ply";
	fem->loadFem(filename);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	fem->setCollisionRepresentation(collisionRepresentation);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*fem));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	std::shared_ptr<Fem3DCorotationalTetrahedronRepresentation> newRepresentation;
	ASSERT_NO_THROW(newRepresentation = std::dynamic_pointer_cast<Fem3DCorotationalTetrahedronRepresentation>(
											node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	ASSERT_NE(nullptr, newRepresentation);

	EXPECT_EQ("SurgSim::Physics::Fem3DCorotationalTetrahedronRepresentation", newRepresentation->getClassName());
	EXPECT_EQ(filename, newRepresentation->getFem()->getValue<std::string>("FileName"));
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, FemElementTypeTest)
{
	auto fem = std::make_shared<SurgSim::Physics::Fem3DCorotationalTetrahedronRepresentation>("Test-Fem3D");
	EXPECT_ANY_THROW(fem->setFemElementType("NotAnFem3D"));
}

TEST_F(Fem3DCorotationalTetrahedronRepresentationTests, NodeTransformationTest)
{
	auto fem = std::make_shared<SurgSim::Physics::Fem3DCorotationalTetrahedronRepresentation>("Test-Fem3D");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string filename = "PlyReaderTests/Tetrahedron.ply";
	fem->loadFem(filename);
}


} // namespace Physics
} // namespace SurgSim
