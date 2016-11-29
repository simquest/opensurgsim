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

/// \file Fem2DRepresentationTests.cpp
/// This file tests the functionalities of the class Fem2DRepresentation.

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DLocalization.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{

namespace Physics
{

TEST(Fem2DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Fem2DRepresentation> fem = std::make_shared<Fem2DRepresentation>("Fem2D");});
}

TEST(Fem2DRepresentationTests, GetNumDofPerNodeTest)
{
	std::shared_ptr<Fem2DRepresentation> fem = std::make_shared<Fem2DRepresentation>("Fem2D");
	EXPECT_EQ(6u, fem->getNumDofPerNode());
}

TEST(Fem2DRepresentationTests, TransformInitialStateTest)
{
	using SurgSim::Math::Vector;

	std::shared_ptr<Fem2DRepresentation> fem = std::make_shared<Fem2DRepresentation>("Fem2D");

	const size_t numNodes = 3;
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
	Vector a = Vector::Ones(numDof) * 2.0;
	initialState->getPositions() = x;
	initialState->getVelocities() = v;
	fem->setInitialState(initialState);

	Vector expectedX = x, expectedV = v, expectedA = a;
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		expectedX.segment<3>(numDofPerNode * nodeId) = initialPose * x.segment<3>(numDofPerNode * nodeId);
		expectedV.segment<3>(numDofPerNode * nodeId) = initialPose.linear() * v.segment<3>(numDofPerNode * nodeId);
		expectedA.segment<3>(numDofPerNode * nodeId) = initialPose.linear() * a.segment<3>(numDofPerNode * nodeId);
	}

	// Initialize the component
	ASSERT_TRUE(fem->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	// Wake-up the component => apply the pose to the initial state
	ASSERT_TRUE(fem->wakeUp());

	EXPECT_TRUE(fem->getInitialState()->getPositions().isApprox(expectedX));
	EXPECT_TRUE(fem->getInitialState()->getVelocities().isApprox(expectedV));
}

TEST(Fem2DRepresentationTests, ExternalForceAPITest)
{
	std::shared_ptr<Fem2DRepresentation> fem = std::make_shared<Fem2DRepresentation>("Fem");
	std::shared_ptr<SurgSim::Math::OdeState> initialState = std::make_shared<SurgSim::Math::OdeState>();
	initialState->setNumDof(6, 4);

	// External force vector not initialized until the initial state has been set (it contains the #dof...)
	EXPECT_EQ(0, fem->getExternalGeneralizedForce().size());
	EXPECT_EQ(0, fem->getExternalGeneralizedStiffness().rows());
	EXPECT_EQ(0, fem->getExternalGeneralizedStiffness().cols());
	EXPECT_EQ(0, fem->getExternalGeneralizedDamping().rows());
	EXPECT_EQ(0, fem->getExternalGeneralizedDamping().cols());

	fem->setInitialState(initialState);

	Math::SparseMatrix zeroMat(static_cast<SparseMatrix::Index>(fem->getNumDof()),
							   static_cast<SparseMatrix::Index>(fem->getNumDof()));
	zeroMat.setZero();

	// Vector initialized (properly sized and zeroed)
	EXPECT_NE(0, fem->getExternalGeneralizedForce().size());
	EXPECT_NE(0, fem->getExternalGeneralizedStiffness().rows());
	EXPECT_NE(0, fem->getExternalGeneralizedStiffness().cols());
	EXPECT_NE(0, fem->getExternalGeneralizedDamping().rows());
	EXPECT_NE(0, fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(static_cast<Math::Vector6d::Index>(fem->getNumDof()), fem->getExternalGeneralizedForce().size());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(fem->getNumDof()), fem->getExternalGeneralizedStiffness().cols());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(fem->getNumDof()), fem->getExternalGeneralizedStiffness().rows());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(fem->getNumDof()), fem->getExternalGeneralizedDamping().cols());
	EXPECT_EQ(static_cast<SparseMatrix::Index>(fem->getNumDof()), fem->getExternalGeneralizedDamping().rows());
	EXPECT_TRUE(fem->getExternalGeneralizedForce().isZero());
	EXPECT_TRUE(fem->getExternalGeneralizedStiffness().isApprox(zeroMat));
	EXPECT_TRUE(fem->getExternalGeneralizedDamping().isApprox(zeroMat));

	std::array<size_t, 3> element1NodeIds = {{0, 1, 2}};
	auto element1 = std::make_shared<Fem2DElementTriangle>(element1NodeIds);
	fem->addFemElement(element1);
	std::array<size_t, 3> element2NodeIds = {{1, 2, 3}};
	auto element2 = std::make_shared<Fem2DElementTriangle>(element2NodeIds);
	fem->addFemElement(element2);

	SurgSim::DataStructures::IndexedLocalCoordinate femRepCoordinate;
	femRepCoordinate.index = 0;
	femRepCoordinate.coordinate = SurgSim::Math::Vector::Zero(3);
	femRepCoordinate.coordinate[0] = 1.0;
	auto localization = std::make_shared<Fem2DLocalization>(fem, femRepCoordinate);
	auto wrongLocalizationType = std::make_shared<MockLocalization>();

	Vector FLocalWrongSize = Vector::Ones(2 * fem->getNumDofPerNode());
	Matrix KLocalWrongSize = Matrix::Ones(3 * fem->getNumDofPerNode(), 3 * fem->getNumDofPerNode());
	Matrix DLocalWrongSize = Matrix::Ones(4 * fem->getNumDofPerNode(), 4 * fem->getNumDofPerNode());
	Vector Flocal = Vector::LinSpaced(fem->getNumDofPerNode(), -3.12, 4.09);
	Matrix Klocal = Matrix::Ones(fem->getNumDofPerNode(), fem->getNumDofPerNode()) * 0.34;
	Matrix Dlocal = Klocal + Matrix::Identity(fem->getNumDofPerNode(), fem->getNumDofPerNode());
	Vector F = Vector::Zero(fem->getNumDof());
	F.segment(0, fem->getNumDofPerNode()) = Flocal;
	SparseMatrix K(static_cast<SparseMatrix::Index>(fem->getNumDof()),
				   static_cast<SparseMatrix::Index>(fem->getNumDof()));
	K.setZero();
	Math::addSubMatrix(Klocal, 0, 0, &K, true);
	K.makeCompressed();
	SparseMatrix D(static_cast<SparseMatrix::Index>(fem->getNumDof()),
				   static_cast<SparseMatrix::Index>(fem->getNumDof()));
	D.setZero();
	Math::addSubMatrix(Dlocal, 0, 0, &D, true);
	D.makeCompressed();

	// Test invalid localization nullptr
	ASSERT_THROW(fem->addExternalGeneralizedForce(nullptr, Flocal),
				 SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(fem->addExternalGeneralizedForce(nullptr, Flocal, Klocal, Dlocal),
				 SurgSim::Framework::AssertionFailure);
	// Test invalid localization type
	ASSERT_THROW(fem->addExternalGeneralizedForce(wrongLocalizationType, Flocal),
				 SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(fem->addExternalGeneralizedForce(wrongLocalizationType, Flocal, Klocal, Dlocal),
				 SurgSim::Framework::AssertionFailure);
	// Test invalid force size
	ASSERT_THROW(fem->addExternalGeneralizedForce(localization, FLocalWrongSize),
				 SurgSim::Framework::AssertionFailure);
	ASSERT_THROW(fem->addExternalGeneralizedForce(localization, FLocalWrongSize, Klocal, Dlocal),
				 SurgSim::Framework::AssertionFailure);
	// Test invalid stiffness size
	ASSERT_THROW(fem->addExternalGeneralizedForce(localization, Flocal, KLocalWrongSize, Dlocal),
				 SurgSim::Framework::AssertionFailure);
	// Test invalid damping size
	ASSERT_THROW(fem->addExternalGeneralizedForce(localization, Flocal, Klocal, DLocalWrongSize),
				 SurgSim::Framework::AssertionFailure);

	// Test valid call to addExternalGeneralizedForce
	Math::SparseMatrix zeroMatrix(static_cast<SparseMatrix::Index>(fem->getNumDof()),
								  static_cast<SparseMatrix::Index>(fem->getNumDof()));
	zeroMatrix.setZero();
	fem->addExternalGeneralizedForce(localization, Flocal, Klocal, Dlocal);
	EXPECT_FALSE(fem->getExternalGeneralizedForce().isZero());
	EXPECT_FALSE(fem->getExternalGeneralizedStiffness().isApprox(zeroMatrix));
	EXPECT_FALSE(fem->getExternalGeneralizedDamping().isApprox(zeroMatrix));
	EXPECT_TRUE(fem->getExternalGeneralizedForce().isApprox(F));
	EXPECT_TRUE(fem->getExternalGeneralizedStiffness().isApprox(K));
	EXPECT_TRUE(fem->getExternalGeneralizedDamping().isApprox(D));

	// Test valid call to addExternalGeneralizedForce to add things up
	fem->addExternalGeneralizedForce(localization, Flocal, Klocal, Dlocal);
	EXPECT_TRUE(fem->getExternalGeneralizedForce().isApprox(2.0 * F));
	EXPECT_TRUE(fem->getExternalGeneralizedStiffness().isApprox(2.0 * K));
	EXPECT_TRUE(fem->getExternalGeneralizedDamping().isApprox(2.0 * D));
}

TEST(Fem2DRepresentationTests, LoadMeshTest)
{
	auto femRepresentation = std::make_shared<Fem2DRepresentation>("Representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");

	femRepresentation->loadFem("PlyReaderTests/Fem2D.ply");
	ASSERT_TRUE(femRepresentation->initialize(runtime));

	// Vertices
	ASSERT_EQ(6u, femRepresentation->getNumDofPerNode());
	ASSERT_EQ(6u * 6u, femRepresentation->getNumDof());

	Vector3d vertex0(1.0, 1.0, -1.0);
	Vector3d vertex5(0.999999, -1.000001, 1.0);

	EXPECT_TRUE(vertex0.isApprox(femRepresentation->getInitialState()->getPosition(0)));
	EXPECT_TRUE(vertex5.isApprox(femRepresentation->getInitialState()->getPosition(5)));

	// Number of triangles
	ASSERT_EQ(3u, femRepresentation->getNumFemElements());

	std::array<size_t, 3> triangle0 = {0, 1, 2};
	std::array<size_t, 3> triangle2 = {3, 4, 5};

	EXPECT_TRUE(std::equal(std::begin(triangle0), std::end(triangle0),
						   std::begin(femRepresentation->getFemElement(0)->getNodeIds())));
	EXPECT_TRUE(std::equal(std::begin(triangle2), std::end(triangle2),
						   std::begin(femRepresentation->getFemElement(2)->getNodeIds())));

	// Boundary conditions
	ASSERT_EQ(2u * 6u, femRepresentation->getInitialState()->getNumBoundaryConditions());

	// Boundary condition 0 is on node 8
	size_t boundaryNode0 = 3;
	size_t boundaryNode1 = 2;

	EXPECT_EQ(6 * boundaryNode0, femRepresentation->getInitialState()->getBoundaryConditions().at(0));
	EXPECT_EQ(6 * boundaryNode0 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(1));
	EXPECT_EQ(6 * boundaryNode0 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(2));
	EXPECT_EQ(6 * boundaryNode1, femRepresentation->getInitialState()->getBoundaryConditions().at(6));
	EXPECT_EQ(6 * boundaryNode1 + 1, femRepresentation->getInitialState()->getBoundaryConditions().at(7));
	EXPECT_EQ(6 * boundaryNode1 + 2, femRepresentation->getInitialState()->getBoundaryConditions().at(8));

	// Material
	for (size_t i = 0; i < femRepresentation->getNumFemElements(); ++i)
	{
		auto fem = femRepresentation->getFemElement(i);
		EXPECT_DOUBLE_EQ(0.2, fem->getMassDensity());
		EXPECT_DOUBLE_EQ(0.3, fem->getPoissonRatio());
		EXPECT_DOUBLE_EQ(0.4, fem->getYoungModulus());

		auto fem2DTriganle = std::dynamic_pointer_cast<SurgSim::Physics::Fem2DElementTriangle>(fem);
		ASSERT_NE(nullptr, fem2DTriganle);
		EXPECT_DOUBLE_EQ(0.1, fem2DTriganle->getThickness());
	}
}

TEST(Fem2DRepresentationTests, CreateLocalizationTest)
{
	auto fem = std::make_shared<Fem2DRepresentation>("Representation");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	ASSERT_NO_THROW(fem->loadFem("PlyReaderTests/Fem2D.ply"));
	ASSERT_NO_THROW(ASSERT_TRUE(fem->initialize(runtime)));

	// Localization on an invalid node
	{
		SCOPED_TRACE("Invalid node");

		SurgSim::DataStructures::Location location(1000);
		std::shared_ptr<SurgSim::Physics::Localization> localization;
		EXPECT_THROW(localization = fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
	}

	// Localization on a valid node
	{
		SCOPED_TRACE("Valid node");

		SurgSim::DataStructures::Location location(0);
		std::shared_ptr<SurgSim::Physics::Fem2DLocalization> localization;
		EXPECT_NO_THROW(localization =
			std::dynamic_pointer_cast<SurgSim::Physics::Fem2DLocalization>(fem->createLocalization(location)););
		EXPECT_TRUE(localization != nullptr);
		EXPECT_TRUE(localization->getRepresentation() == fem);

		SurgSim::Math::Vector3d globalPosition;
		SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
		EXPECT_NO_THROW(globalPosition = fem->getCurrentState()->getPosition(coordinate.index););
		EXPECT_TRUE(globalPosition.isApprox(localization->calculatePosition()));
	}

	// In the 2d case, location of type triangle and element are the same and should behave the same.
	// Hence, we are testing both but factorizing the code.
	std::vector<std::pair<SurgSim::DataStructures::Location::Type, std::string>> locationTypesToTest;
	locationTypesToTest.push_back(std::make_pair(SurgSim::DataStructures::Location::TRIANGLE, "triangle"));
	locationTypesToTest.push_back(std::make_pair(SurgSim::DataStructures::Location::ELEMENT, "element"));
	for (auto const& locationPairTypeString : locationTypesToTest)
	{
		auto const& locationType = locationPairTypeString.first;

		SCOPED_TRACE("Location of type " + locationPairTypeString.second);

		// Localization on an invalid triangle/element
		{
			SCOPED_TRACE("Invalid triangle/element index");

			Vector barycentricCoordinates = Vector::Zero(3);
			barycentricCoordinates[0] = 1.0;
			SurgSim::DataStructures::IndexedLocalCoordinate coord(10000, barycentricCoordinates);
			SurgSim::DataStructures::Location location(coord, locationType);
			std::shared_ptr<SurgSim::Physics::Localization> localization;
			EXPECT_THROW(localization = fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
		}

		// Localization on an valid triangle/element but invalid barycentric coordinate size
		{
			SCOPED_TRACE("Invalid triangle/element barycentric coordinate size");

			Vector barycentricCoordinates = Vector::Zero(4);
			barycentricCoordinates[0] = 1.0;
			SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);

			SurgSim::DataStructures::Location location(coord, locationType);
			std::shared_ptr<SurgSim::Physics::Localization> localization;
			EXPECT_THROW(localization = fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
		}

		// Localization on an valid triangle/element but invalid barycentric coordinate
		{
			SCOPED_TRACE("Invalid triangle/element barycentric coordinate");

			Vector barycentricCoordinates = Vector::Ones(3);
			SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);
			SurgSim::DataStructures::Location location(coord, locationType);
			std::shared_ptr<SurgSim::Physics::Localization> localization;
			EXPECT_THROW(localization = fem->createLocalization(location), SurgSim::Framework::AssertionFailure);
		}

		// Localization on a valid triangle/element
		{
			SCOPED_TRACE("Valid triangle/element");

			Vector barycentricCoordinates = Vector::Zero(3);
			barycentricCoordinates.setConstant(1.0 / 3.0);
			SurgSim::DataStructures::IndexedLocalCoordinate coord(0, barycentricCoordinates);
			SurgSim::DataStructures::Location location(coord, locationType);
			std::shared_ptr<SurgSim::Physics::Fem2DLocalization> localization;
			EXPECT_NO_THROW(localization =
				std::dynamic_pointer_cast<SurgSim::Physics::Fem2DLocalization>(
				fem->createLocalization(location)));
			EXPECT_TRUE(localization != nullptr);
			EXPECT_TRUE(localization->getRepresentation() == fem);

			SurgSim::DataStructures::IndexedLocalCoordinate coordinate = localization->getLocalPosition();
			auto element = fem->getFemElement(coordinate.index);
			SurgSim::Math::Vector3d globalPosition =
				1.0 / 3.0 * fem->getCurrentState()->getPosition(element->getNodeId(0)) +
				1.0 / 3.0 * fem->getCurrentState()->getPosition(element->getNodeId(1)) +
				1.0 / 3.0 * fem->getCurrentState()->getPosition(element->getNodeId(2));
			EXPECT_TRUE(globalPosition.isApprox(localization->calculatePosition()));
		}
	}
}

TEST(Fem2DRepresentationTests, SerializationTest)
{
	auto fem2DRepresentation = std::make_shared<SurgSim::Physics::Fem2DRepresentation>("Test-Fem2D");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	const std::string filename = "PlyReaderTests/Fem2D.ply";
	fem2DRepresentation->loadFem(filename);
	auto collisionRepresentation = std::make_shared<DeformableCollisionRepresentation>("Collision");
	fem2DRepresentation->setCollisionRepresentation(collisionRepresentation);

	YAML::Node node;
	ASSERT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*fem2DRepresentation));
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(1u, node.size());

	std::shared_ptr<Fem2DRepresentation> newRepresentation;
	ASSERT_NO_THROW(newRepresentation = std::dynamic_pointer_cast<Fem2DRepresentation>
										(node.as<std::shared_ptr<SurgSim::Framework::Component>>()));
	ASSERT_NE(nullptr, newRepresentation);

	EXPECT_EQ("SurgSim::Physics::Fem2DRepresentation", newRepresentation->getClassName());
	EXPECT_EQ(filename, newRepresentation->getFem()->getValue<std::string>("FileName"));
}

} // namespace Physics

} // namespace SurgSim
