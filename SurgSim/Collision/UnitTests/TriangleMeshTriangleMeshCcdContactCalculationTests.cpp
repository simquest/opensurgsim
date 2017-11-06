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

#include <memory>

#include "SurgSim/Collision/TriangleMeshTriangleMeshContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

/*	PRISM
	  3
	  /\
	0/  \
	|\   \
	| \   \
	|  \   \
	|4. \   \.5
	|    \  /
	1*-----*2
*/
static const int prismNumPoints = 6;
static const Vector3d prismPoints[6] =
{
	Vector3d(0.0, 1.0, 0.5),
	Vector3d(0.0, 0.0, 0.5),
	Vector3d(1.0, 0.0, 0.5),
	Vector3d(0.0, 1.0, -0.5),
	Vector3d(0.0, 0.0, -0.5),
	Vector3d(1.0, 0.0, -0.5)
};

static const int prismNumEdges = 9;
static const int prismEdges[9][2] =
{
	{0, 1}, {1, 2}, {2, 0},
	{0, 3}, {1, 4}, {2, 5},
	{3, 4}, {4, 5}, {5, 3}
};

static const int prismNumTriangles = 8;
static const int prismTrianglesCCW[8][3] =
{
	{0, 1, 2},
	{0, 2, 3}, {3, 2, 5},
	{2, 1, 4}, {5, 2, 4},
	{4, 1, 0}, {4, 0, 3},
	{3, 4, 5}
};

/*	TRIANGLE
		   0
		  /\
		 /  \
		/    \
	   /      \
	  /        \
	1*----------*2
*/
static const int triangleNumPoints = 3;
static const Vector3d trianglePoints[3] =
{
	Vector3d(0.0, 0.0, 0.0),
	Vector3d(-0.5, 0.0, 0.5),
	Vector3d(0.5, 0.0, 0.5)
};

static const int triangleNumEdges = 3;
static const int triangleEdges[3][2] =
{
	{0, 1}, {1, 2}, {2, 0}
};

static const int triangleNumTriangles = 1;
static const int triangleTrianglesCCW[1][3] =
{
	{0, 1, 2}
};

std::shared_ptr<Math::MeshShape> createPrismShape(const Math::RigidTransform3d& pose, const int startId = 0)
{
	auto mesh = std::make_shared<TriangleMeshPlain>();
	for (int i = 0; i < prismNumPoints; ++i)
	{
		Vector3d p;
		p[0] = prismPoints[i][0];
		p[1] = prismPoints[i][1];
		p[2] = prismPoints[i][2];
		TriangleMeshPlain::VertexType v(pose * p);
		mesh->addVertex(v);
	}
	for (int i = 0; i < prismNumEdges; ++i)
	{
		std::array<size_t, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = prismEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		mesh->addEdge(e);
	}
	for (int i = 0; i < prismNumTriangles; ++i)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[(startId + j) % 3] = prismTrianglesCCW[i][j];
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
		mesh->addTriangle(t);
	}
	return std::make_shared<Math::MeshShape>(*mesh);
}

std::shared_ptr<Math::MeshShape> createTriangleShape(const Math::RigidTransform3d& pose, const int startId = 0)
{
	auto mesh = std::make_shared<TriangleMeshPlain>();
	for (int i = 0; i < triangleNumPoints; ++i)
	{
		Vector3d p;
		p[0] = trianglePoints[i][0];
		p[1] = trianglePoints[i][1];
		p[2] = trianglePoints[i][2];
		TriangleMeshPlain::VertexType v(pose * p);
		mesh->addVertex(v);
	}
	for (int i = 0; i < triangleNumEdges; ++i)
	{
		std::array<size_t, 2> edgePoints;
		for (int j = 0; j < 2; j++)
		{
			edgePoints[j] = triangleEdges[i][j];
		}
		TriangleMeshPlain::EdgeType e(edgePoints);
		mesh->addEdge(e);
	}
	for (int i = 0; i < triangleNumTriangles; ++i)
	{
		std::array<size_t, 3> trianglePoints;
		for (int j = 0; j < 3; j++)
		{
			trianglePoints[(startId + j) % 3] = triangleTrianglesCCW[i][j];
		}
		TriangleMeshPlain::TriangleType t(trianglePoints);
		mesh->addTriangle(t);
	}
	return std::make_shared<Math::MeshShape>(*mesh);
}

class MockCollisionRepresentation : public ShapeCollisionRepresentation
{
public:
	explicit MockCollisionRepresentation(const std::string& name)
		: ShapeCollisionRepresentation(name) {}

	void setPosedShapeMotionPublic(const Math::PosedShapeMotion<std::shared_ptr<Math::Shape>>& posedShape)
	{
		setPosedShapeMotion(posedShape);
	}
};

void doTriangleMeshTriangleMeshTest(const std::shared_ptr<Math::MeshShape>& shapeA1,
									const std::shared_ptr<Math::MeshShape>& shapeA2,
									const std::shared_ptr<Math::MeshShape>& shapeB1,
									const std::shared_ptr<Math::MeshShape>& shapeB2,
									const std::list<std::shared_ptr<Contact>> expectedContacts)
{
	using Math::PosedShape;
	using Math::Shape;
	using Math::RigidTransform3d;

	Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeAMotion(
		PosedShape<std::shared_ptr<Shape>>(std::static_pointer_cast<Shape>(shapeA1), RigidTransform3d::Identity()),
		PosedShape<std::shared_ptr<Shape>>(std::static_pointer_cast<Shape>(shapeA2), RigidTransform3d::Identity()));
	auto meshARep = std::make_shared<MockCollisionRepresentation>("Collision Mesh 0");
	meshARep->setPosedShapeMotionPublic(posedShapeAMotion);
	meshARep->setShape(shapeA1);
	meshARep->setCollisionDetectionType(COLLISION_DETECTION_TYPE_CONTINUOUS);

	Math::PosedShapeMotion<std::shared_ptr<Math::Shape>> posedShapeBMotion(
		PosedShape<std::shared_ptr<Shape>>(std::static_pointer_cast<Shape>(shapeB1), RigidTransform3d::Identity()),
		PosedShape<std::shared_ptr<Shape>>(std::static_pointer_cast<Shape>(shapeB2), RigidTransform3d::Identity()));
	auto meshBRep = std::make_shared<MockCollisionRepresentation>("Collision Mesh 1");
	meshBRep->setPosedShapeMotionPublic(posedShapeBMotion);
	meshBRep->setShape(shapeB1);
	meshBRep->setCollisionDetectionType(COLLISION_DETECTION_TYPE_CONTINUOUS);

	// Perform collision detection.
	TriangleMeshTriangleMeshContact calcContact;
	std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(meshARep, meshBRep);
	calcContact.calculateContact(pair);

	contactsInfoEqualityTest(expectedContacts, pair->getContacts(), true);
}

TEST(TriangleMeshTriangleMeshCcdContactCalculationTests, NonintersectionTest)
{
	using Math::makeRigidTranslation;
	using Math::makeRigidTransform;
	using Math::makeRotationQuaternion;

	Math::RigidTransform3d shapeATransform1;
	Math::RigidTransform3d shapeATransform2;
	Math::RigidTransform3d shapeBTransform1;
	Math::RigidTransform3d shapeBTransform2;
	Math::RigidTransform3d globalTransform;
	const std::list<std::shared_ptr<Contact>> emptyContacts;

	const double safeDistance = 1.0;
	const double epsilonTrans = 0.001;
	{
		SCOPED_TRACE("No intersection, prismB above prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(0.0, -epsilonTrans, 0.0));
		shapeBTransform1 = makeRigidTranslation(Vector3d(0.0, safeDistance + epsilonTrans, 0.0));
		shapeBTransform2 = makeRigidTranslation(Vector3d(0.0, safeDistance + 2.0 * epsilonTrans, 0.0));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.234, Vector3d(1.2, 3.4, 5.6).normalized()),
											 Vector3d(34.4, 567.6, 234.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, prismB below prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(0.0, epsilonTrans, 0.0));
		shapeBTransform1 = makeRigidTranslation(Vector3d(0.0, -(safeDistance + epsilonTrans), 0.0));
		shapeBTransform2 = makeRigidTranslation(Vector3d(0.0, -(safeDistance + 2.0 * epsilonTrans), 0.0));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(10.2, 34.4, 15.6).normalized()),
											 Vector3d(3.4, 6.6, 2.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, prismB to the left of prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(epsilonTrans, 0.0, 0.0));
		shapeBTransform1 = makeRigidTranslation(Vector3d(-(safeDistance + epsilonTrans), 0.0, 0.0));
		shapeBTransform2 = makeRigidTranslation(Vector3d(-(safeDistance + 2.0 * epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.4, Vector3d(1.2, 3.4, 5.6).normalized()),
											 Vector3d(340.4, 567.6, 234.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, prismB to the right of prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(-epsilonTrans, 0.0, 0.0));
		shapeBTransform1 = makeRigidTranslation(Vector3d((safeDistance + epsilonTrans), 0.0, 0.0));
		shapeBTransform2 = makeRigidTranslation(Vector3d((safeDistance + 2.0 * epsilonTrans), 0.0, 0.0));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.2, Vector3d(11.2, 13.4, 15.6).normalized()),
											 Vector3d(3.4, 5.6, 2.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, prismB in front of prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(0.0, 0.0, -epsilonTrans));
		shapeBTransform1 = makeRigidTranslation(Vector3d(0.0, 0.0, (safeDistance + epsilonTrans)));
		shapeBTransform2 = makeRigidTranslation(Vector3d(0.0, 0.0, (safeDistance + 2.0 * epsilonTrans)));
		globalTransform = makeRigidTransform(makeRotationQuaternion(2.234, Vector3d(10.2, 30.4, 50.6).normalized()),
											 Vector3d(84.4, 56.6, 24.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}

	{
		SCOPED_TRACE("No intersection, prismB behind prismA");
		shapeATransform1.setIdentity();
		shapeATransform2 = makeRigidTranslation(Vector3d(0.0, 0.0, epsilonTrans));
		shapeBTransform1 = makeRigidTranslation(Vector3d(0.0, 0.0, -(safeDistance + epsilonTrans)));
		shapeBTransform2 = makeRigidTranslation(Vector3d(0.0, 0.0, -(safeDistance + 2.0 * epsilonTrans)));
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.24, Vector3d(9.2, 7.4, 5.6).normalized()),
											 Vector3d(39.4, 67.6, 34.5));
		shapeATransform1 = globalTransform * shapeATransform1;
		shapeATransform2 = globalTransform * shapeATransform2;
		shapeBTransform1 = globalTransform * shapeBTransform1;
		shapeBTransform2 = globalTransform * shapeBTransform2;

		doTriangleMeshTriangleMeshTest(createPrismShape(shapeATransform1), createPrismShape(shapeATransform2),
			createPrismShape(shapeBTransform1), createPrismShape(shapeBTransform2), emptyContacts);
	}
}

namespace
{
/// Calculate the expected contact data from the given input.
/// \param shapeA1, shapeA2 First Mesh shape (A) at time 0 and time 1.
/// \param triangleIdA The triangle that will be in contact in shapeA.
/// \param contactA1, contactA2 The point of contact on shapeA (same point at time 0 and time 1)
/// \param shapeB1, shapeB2 Second Mesh shape (B) at time 0 and time 1.
/// \param triangleIdB The triangle that will be in contact in shapeB.
/// \param contactB1, contactB2 The point of contact on shapeB (same point at time 0 and time 1)
/// \param normal The expected contact normal.
/// \note Based on contact{A/B}{1/2} the time of impact is calculated bu solving for t in the eqns:
///       A1 + t*(A2 - A1) = B1 + t*(B2 - B1)
std::shared_ptr<TriangleContact> createTriangleContact(
	const std::shared_ptr<Math::MeshShape>& shapeA1,
	const std::shared_ptr<Math::MeshShape>& shapeA2,
	const size_t triangleIdA,
	const Vector3d& contactA1, const Vector3d& contactA2,
	const std::shared_ptr<Math::MeshShape>& shapeB1,
	const std::shared_ptr<Math::MeshShape>& shapeB2,
	const size_t triangleIdB,
	const Vector3d& contactB1, const Vector3d& contactB2,
	const Vector3d& normal)
{
	std::pair<Location, Location> expectedPenetrationPoints;
	expectedPenetrationPoints.first.rigidLocalPosition.setValue(contactA2);
	expectedPenetrationPoints.second.rigidLocalPosition.setValue(contactB2);
	SurgSim::DataStructures::IndexedLocalCoordinate triangleLocalPosition;
	triangleLocalPosition.index = triangleIdA;
	expectedPenetrationPoints.first.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
	triangleLocalPosition.index = triangleIdB;
	expectedPenetrationPoints.second.triangleMeshLocalCoordinate.setValue(triangleLocalPosition);
	double toi = (contactA2 + contactB1 - contactA1 - contactB2).norm();
	if (std::abs(toi) > Math::Geometry::ScalarEpsilon)
	{
		toi = (contactB1 - contactA1).norm() / toi;
	}
	else
	{
		toi = 0.0;
	}
	auto contact = std::make_shared<TriangleContact>(
		COLLISION_DETECTION_TYPE_CONTINUOUS, std::abs((contactA2 - contactB2).dot(normal)),
		toi, (contactA2 + contactB2) * 0.5, normal, expectedPenetrationPoints);
	contact->firstVertices = shapeA2->getTrianglePositions(triangleIdA);
	contact->secondVertices = shapeB2->getTrianglePositions(triangleIdB);
	return contact;
}
}

TEST(TriangleMeshTriangleMeshCcdContactCalculationTests, IntersectionTest)
{
	using Math::makeRigidTranslation;
	using Math::makeRigidTransform;
	using Math::makeRotationQuaternion;

	Math::RigidTransform3d shapeATransform1;
	Math::RigidTransform3d shapeATransform2;
	Math::RigidTransform3d shapeBTransform1;
	Math::RigidTransform3d shapeBTransform2;
	Math::RigidTransform3d globalTransform;
	globalTransform.setIdentity();

	double epsilonTrans = 0.001;

	{
		SCOPED_TRACE("Vertex of triangle into first face of prism");
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.234, Vector3d(1.2, 3.4, 5.6).normalized()),
			Vector3d(34.4, 567.6, 234.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				shapeATransform1 = globalTransform;
				shapeATransform2 = globalTransform * makeRigidTranslation(Vector3d(0.0, 0.0, epsilonTrans));
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(0.1, 0.1, 1.0));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(0.1, 0.1, 0.5));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.1, 0.1, 0.5),
					globalTransform * Vector3d(0.1, 0.1, 0.501),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.1, 0.1, 1.0),
					globalTransform * Vector3d(0.1, 0.1, 0.5),
					globalTransform.linear() * Vector3d(0.0, 0.0, -1.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Vertex of triangle into fifth face of prism");
		globalTransform = makeRigidTransform(makeRotationQuaternion(0.4, Vector3d(10.2, 34.4, 15.6).normalized()),
			Vector3d(3.4, 6.6, 2.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				shapeATransform1 = globalTransform;
				shapeATransform2 = globalTransform * makeRigidTranslation(Vector3d(-epsilonTrans, 0.0, 0.0));
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-1.0, 0.1, -0.6));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.5, 0.1, -0.6));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.0, 0.1, -0.1),
					globalTransform * Vector3d(-0.001, 0.1, -0.1),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.1, -0.1),
					globalTransform * Vector3d(0.0, 0.1, -0.1),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into first/sixth face of prism");
		globalTransform = makeRigidTransform(makeRotationQuaternion(0.473, Vector3d(4.9, 2.5, -0.6).normalized()),
			Vector3d(3.7, 9.2, 36.2));
		double sqrt2by2 = std::sqrt(2.0) / 2.0;
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.0, 0.0, -0.5));
				shapeATransform2 = globalTransform * makeRigidTranslation(Vector3d(-epsilonTrans, 0.0, -0.499));
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-0.5, 0.1, 0.0));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.2, 0.1, -0.3));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.0, 0.1, 0.0),
					globalTransform * Vector3d(-0.001, 0.1, 0.001),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.25, 0.1, 0.25),
					globalTransform * Vector3d(0.05, 0.1, -0.05),
					globalTransform.linear() * Vector3d(sqrt2by2, 0.0, -sqrt2by2)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.0, 0.1, 0.0),
					globalTransform * Vector3d(-0.001, 0.1, 0.001),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.25, 0.1, 0.25),
					globalTransform * Vector3d(0.05, 0.1, -0.05),
					globalTransform.linear() * Vector3d(sqrt2by2, 0.0, -sqrt2by2)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into first/sixth face of prism at time 0");
		globalTransform = makeRigidTransform(makeRotationQuaternion(1.04, Vector3d(9.2, 7.4, 5.6).normalized()),
			Vector3d(39.4, 67.6, 34.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.0, 0.0, -0.5));
				shapeATransform2 = globalTransform * makeRigidTranslation(Vector3d(0.0, 0.0, -0.5));
				shapeBTransform1 = globalTransform *
					makeRigidTranslation(Vector3d(-0.25, 0.5, -0.35));
				shapeBTransform2 = globalTransform *
					makeRigidTranslation(Vector3d(-0.25, 0.5, -0.35));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.0, 0.5, 0.0),
					globalTransform * Vector3d(0.0, 0.5, 0.0),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.0, 0.5, -0.1),
					globalTransform * Vector3d(0.0, 0.5, -0.1),
					globalTransform.linear() * Vector3d(0.0, 0.0, -1.0)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.0, 0.5, 0.0),
					globalTransform * Vector3d(0.0, 0.5, 0.0),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.1, 0.5, 0.0),
					globalTransform * Vector3d(0.1, 0.5, 0.0),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into edge of prism, opposite motions");
		globalTransform = makeRigidTransform(makeRotationQuaternion(16.04, Vector3d(0.2, 7.4, 5.6).normalized()),
			Vector3d(39.4, 7.6, 34.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				// collision between prism segment 01 at midpoint, and triangle segment 02 at midpoint.
				// prism moves -x
				// triangle moves +x
				// expect normal to be +x (opposite of prism motion)
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.5, 0.0, 0.0));
				shapeATransform2 = globalTransform;
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-0.75, 0.5, 0.25));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into edge of prism, parallel motions");
		globalTransform = makeRigidTransform(makeRotationQuaternion(0.34, Vector3d(9.1, 7.4, 5.6).normalized()),
			Vector3d(39.4, 7.6, 3.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				// collision between prism segment 01 at midpoint, and triangle segment 02 at midpoint.
				// prism moves -x
				// triangle moves -x
				// expect normal to be +x (opposite of prism motion)
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.5, 0.0, 0.0));
				shapeATransform2 = globalTransform;
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(0.0, 0.5, 0.25));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.25, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.25, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into edge of prism, orthogonal motions");
		globalTransform = makeRigidTransform(makeRotationQuaternion(0.81, Vector3d(0.2, 7.4, 0.6).normalized()),
			Vector3d(392.4, 7.6, 34.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				// collision between prism segment 01 at midpoint, and triangle segment 02 at midpoint.
				// prism moves +y
				// triangle moves +x
				// expect normal to be +x-y: slow the prism and push it in the direction of triangle motion,
				//   while the triangle will be adjusted in the -x+y direction so it slows and is pushed in the
				//   direction of the prism's motion.
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.0, -1.0, 0.0));
				shapeATransform2 = globalTransform;
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-0.75, 0.5, 0.25));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				const double prismTravelPostContact = 0.001996007984031999;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.0, -0.5 + prismTravelPostContact, 0.5),
					globalTransform * Vector3d(0.0, 0.5 + prismTravelPostContact, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, -1.0, 0.0).normalized()));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.0, -0.5 + prismTravelPostContact, 0.5),
					globalTransform * Vector3d(0.0, 0.5 + prismTravelPostContact, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, -1.0, 0.0).normalized()));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into edge of prism, only triangle moves");
		globalTransform = makeRigidTransform(makeRotationQuaternion(0.1, Vector3d(0.9, 1.7, 0.9).normalized()),
			Vector3d(2.4, 0.0, 3.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				// collision between prism segment 01 at midpoint, and triangle segment 02 at midpoint.
				// prism does not move
				// triangle moves +x
				// expect normal to be +x, opposite direction of prism's relative motion
				shapeATransform1 = globalTransform;
				shapeATransform2 = globalTransform;
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-0.75, 0.5, 0.25));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(-0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}

	{
		SCOPED_TRACE("Edge of triangle into edge of prism, only prism moves");
		globalTransform = makeRigidTransform(makeRotationQuaternion(3.3, Vector3d(20.2, 17.4, 20.6).normalized()),
			Vector3d(7.4, 7.6, 3.5));
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				// collision between prism segment 01 at midpoint, and triangle segment 02 at midpoint.
				// prism moves -x
				// triangle does not move
				// expect normal to be +x, opposite direction of prism's relative motion
				shapeATransform1 = globalTransform * makeRigidTranslation(Vector3d(0.5, 0.0, 0.0));
				shapeATransform2 = globalTransform;
				shapeBTransform1 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));
				shapeBTransform2 = globalTransform * makeRigidTranslation(Vector3d(-0.25 + epsilonTrans, 0.5, 0.25));

				auto shapeA1 = createPrismShape(shapeATransform1, i);
				auto shapeA2 = createPrismShape(shapeATransform2, i);
				auto shapeB1 = createTriangleShape(shapeBTransform1, j);
				auto shapeB2 = createTriangleShape(shapeBTransform2, j);

				std::list<std::shared_ptr<Contact>> contacts;
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 0,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));
				contacts.push_back(createTriangleContact(
					shapeA1, shapeA2, 5,
					globalTransform * Vector3d(0.5, 0.5, 0.5),
					globalTransform * Vector3d(0.0, 0.5, 0.5),
					shapeB1, shapeB2, 0,
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform * Vector3d(0.001, 0.5, 0.5),
					globalTransform.linear() * Vector3d(1.0, 0.0, 0.0)));

				doTriangleMeshTriangleMeshTest(shapeA1, shapeA2, shapeB1, shapeB2, contacts);
			}
		}
	}
}

} // namespace Collision
} // namespace SurgSim
