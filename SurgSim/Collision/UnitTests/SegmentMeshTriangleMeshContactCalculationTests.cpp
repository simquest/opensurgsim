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

#include <sstream>

#include "SurgSim/Collision/SegmentMeshTriangleMeshContact.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/SegmentMesh.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::DataStructures::TriangleMeshPlain;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

class SegmentMeshTriangleMeshContactCalculationTests : public ::testing::Test
{
protected:
	RigidTransform3d buildRigidTransform(double angle, double axisX, double axisY, double axisZ,
		double translationX, double translationY, double translationZ)
	{
		using SurgSim::Math::makeRigidTransform;
		using SurgSim::Math::makeRotationQuaternion;
		return makeRigidTransform(makeRotationQuaternion(angle, Vector3d(axisX, axisY, axisZ).normalized()),
			Vector3d(translationX, translationY, translationZ));
	}

	void SetUp() override
	{
		m_cubeSize = 1.0;
		m_segmentRadius = 0.1;

		m_transforms.push_back(std::pair<RigidTransform3d, std::string>(
			RigidTransform3d::Identity(), "Identity"));
		m_transforms.push_back(std::pair<RigidTransform3d, std::string>(
			buildRigidTransform(1.234, 17.04, 2.047, 3.052, 23.34, 42.45, 83.68), "Transform 1"));
		m_transforms.push_back(std::pair<RigidTransform3d, std::string>(
			buildRigidTransform(-5.34, 41.03, -2.52, -3.84, -3.45, 66.47, 29.34), "Transform 2"));
		m_transforms.push_back(std::pair<RigidTransform3d, std::string>(
			buildRigidTransform(0.246, -9.42, -4.86, 2.469, 37.68, -34.6, -17.1), "Transform 3"));
		m_transforms.push_back(std::pair<RigidTransform3d, std::string>(
			buildRigidTransform(-0.85, 3.344, 8.329, -97.4, 9.465, 0.275, -95.9), "Transform 4"));
	}

	/// \return The SegmentMeshShape with a line mesh.
	std::shared_ptr<SegmentMeshShape> createSegmentMeshShape()
	{
		static const int segmentNumPoints = 3;
		static const double segmentPoints[3][3] =
		{
			{0.0, -0.5, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.5, 0.0}
		};

		static const int segmentNumEdges = 2;
		static const int segmentEdges[2][2] =
		{
			{0, 1}, {1, 2}
		};

		auto mesh = std::make_shared<SegmentMeshPlain>();
		for (int i = 0; i < segmentNumPoints; ++i)
		{
			Vector3d p;
			p[0] = segmentPoints[i][0];
			p[1] = segmentPoints[i][1];
			p[2] = segmentPoints[i][2];
			SegmentMeshPlain::VertexType v(p);
			mesh->addVertex(v);
		}
		for (int i = 0; i < segmentNumEdges; ++i)
		{
			std::array<size_t, 2> edgePoints;
			for (int j = 0; j < 2; j++)
			{
				edgePoints[j] = segmentEdges[i][j];
			}
			SegmentMeshPlain::EdgeType e(edgePoints);
			mesh->addEdge(e);
		}

		return std::make_shared<SegmentMeshShape>(*mesh, m_segmentRadius);
	}

	/// \return The MeshShape with a cube mesh.
	std::shared_ptr<MeshShape> createCubeMeshShape()
	{
		// Create a Mesh Cube
		static const int cubeNumPoints = 8;
		static const double cubePoints[8][3] =
		{
			{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
			{-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
		};

		static const int cubeNumEdges = 12;
		static const int cubeEdges[12][2] =
		{
			{0, 1}, {3, 2}, {4, 5}, {7, 6}, // +X
			{0, 3}, {1, 2}, {4, 7}, {5, 6}, // +Y
			{0, 4}, {1, 5}, {2, 6}, {3, 7}  // +Z
		};

		static const int cubeNumTriangles = 12;
		static const int cubeTrianglesCCW[12][3] =
		{
			{6, 2, 3}, {6, 3, 7}, // Top    ( 0  1  0) [6237]
			{0, 1, 5}, {0, 5, 4}, // Bottom ( 0 -1  0) [0154]
			{4, 5, 6}, {4, 6, 7}, // Front  ( 0  0  1) [4567]
			{0, 3, 2}, {0, 2, 1}, // Back   ( 0  0 -1) [0321]
			{1, 2, 6}, {1, 6, 5}, // Right  ( 1  0  0) [1265]
			{0, 4, 7}, {0, 7, 3}  // Left   (-1  0  0) [0473]
		};

		auto mesh = std::make_shared<TriangleMeshPlain>();
		for (int i = 0; i < cubeNumPoints; ++i)
		{
			Vector3d p;
			p[0] = cubePoints[i][0];
			p[1] = cubePoints[i][1];
			p[2] = cubePoints[i][2];
			TriangleMeshPlain::VertexType v(p);
			mesh->addVertex(v);
		}
		for (int i = 0; i < cubeNumEdges; ++i)
		{
			std::array<size_t, 2> edgePoints;
			for (int j = 0; j < 2; j++)
			{
				edgePoints[j] = cubeEdges[i][j];
			}
			TriangleMeshPlain::EdgeType e(edgePoints);
			mesh->addEdge(e);
		}
		for (int i = 0; i < cubeNumTriangles; ++i)
		{
			std::array<size_t, 3> trianglePoints;
			for (int j = 0; j < 3; j++)
			{
				trianglePoints[j] = cubeTrianglesCCW[i][j];
			}
			TriangleMeshPlain::TriangleType t(trianglePoints);
			mesh->addTriangle(t);
		}

		return std::make_shared<MeshShape>(*mesh);
	}

	void testSegmentMeshTriangleMeshDCD(std::string scenario, std::shared_ptr<SegmentMeshShape> segmentMeshShape,
		RigidTransform3d segmentMeshShapeTransform, std::shared_ptr<MeshShape> meshShape,
		RigidTransform3d meshShapeTransform, int expectedNumContacts = 0)
	{
		for (const auto& transform : m_transforms)
		{
			SCOPED_TRACE("Scenario: " + scenario + ", for transform: " + transform.second);

			std::shared_ptr<ShapeCollisionRepresentation> segmentMeshRep =
				std::make_shared<ShapeCollisionRepresentation>("Collision Mesh - Segment");
			segmentMeshRep->setShape(segmentMeshShape);
			segmentMeshRep->setLocalPose(transform.first * segmentMeshShapeTransform);

			std::shared_ptr<ShapeCollisionRepresentation> triangleMeshRep =
				std::make_shared<ShapeCollisionRepresentation>("Collision Mesh - Triangle");
			triangleMeshRep->setShape(meshShape);
			triangleMeshRep->setLocalPose(transform.first * meshShapeTransform);

			// Perform collision detection.
			SegmentMeshTriangleMeshContact calcContact;
			std::shared_ptr<CollisionPair> pair = std::make_shared<CollisionPair>(segmentMeshRep, triangleMeshRep);
			calcContact.calculateContact(pair);

			// Verify for correct contact generation.
			auto contacts = pair->getContacts();

			// Check if the number of generated contacts was the expected number.
			EXPECT_EQ(expectedNumContacts, contacts.size());

			if (contacts.size() > 0)
			{
				// Find correction that need to be applied to remove all intersections.
				Vector3d correction = Vector3d::Zero();
				for (auto &contact : contacts)
				{
					Vector3d intersection = contact->normal * contact->depth;
					// Project intersection on correction and subtract that component from intersection.
					if (!correction.isZero())
					{
						Vector3d correctionNormalized = correction.normalized();
						intersection -= intersection.dot(correctionNormalized) * correctionNormalized;
					}
					// Add the unique component of intersection to overall correction.
					correction += intersection;
				}

				// Change this correction to a RigidTransform3d
				RigidTransform3d correctionTransform = Math::makeRigidTranslation(correction);

				// Applying this correction to the two shapes should remove all intersections.
				segmentMeshRep->setLocalPose(correctionTransform * transform.first * segmentMeshShapeTransform);

				// Perform collision detection.
				std::shared_ptr<CollisionPair> pair2 = std::make_shared<CollisionPair>(segmentMeshRep, triangleMeshRep);
				calcContact.calculateContact(pair2);

				// There should be no intersections.
				EXPECT_EQ(0, pair2->getContacts().size());
			}
		}
	}

	// Cube size
	double m_cubeSize;

	// Segment radius
	double m_segmentRadius;

private:
	// List of random transformations and a string to identify it.
	std::vector<std::pair<RigidTransform3d, std::string>> m_transforms;
};


TEST_F(SegmentMeshTriangleMeshContactCalculationTests, NonintersectionTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::makeRotationQuaternion;

	double epsilonTrans = 0.001;

	auto segmentMeshShape = createSegmentMeshShape();
	auto meshShape = createCubeMeshShape();

	RigidTransform3d segmentMeshShapeTransform;
	RigidTransform3d meshShapeTransform;

	// No rotations
	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, m_cubeSize + m_segmentRadius + epsilonTrans, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, -m_cubeSize - m_segmentRadius - epsilonTrans, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(-m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	// Rotation about X
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, offset + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh above box)",
			createSegmentMeshShape(), segmentMeshShapeTransform,
			createCubeMeshShape(), meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, -offset - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, offset + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, -offset - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about x-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	// Rotation about Y
	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, m_cubeSize + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, -m_cubeSize - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about y-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	// Rotation about Z
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, offset + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, -offset - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, offset + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, -offset - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("No intersection (rotated (about z-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform);
	}
}

TEST_F(SegmentMeshTriangleMeshContactCalculationTests, IntersectionTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::makeRotationQuaternion;

	double epsilonTrans = -0.001;

	auto segmentMeshShape = createSegmentMeshShape();
	auto meshShape = createCubeMeshShape();

	RigidTransform3d segmentMeshShapeTransform;
	RigidTransform3d meshShapeTransform;

	// No rotations
	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, m_cubeSize + m_segmentRadius + epsilonTrans, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			2); // bottom segment intersects top two triangles
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, -m_cubeSize - m_segmentRadius - epsilonTrans, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			2); // top segment intersects bottom two triangles
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(-m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect left two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect right two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect farthest two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect nearest two, one top and one bottom triangles
	}

	// Rotation about X
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, offset + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh above box)",
			createSegmentMeshShape(), segmentMeshShapeTransform,
			createCubeMeshShape(), meshShapeTransform,
			1); // bottom segment intersect one top triangle
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, -offset - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersect one bottom triangle
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			4); // both segments intersect two right side triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			4); // both segments intersect two left side triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, offset + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the nearer triangle
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, -offset - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about x-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1);  // bottom segment intersects one of the farther triangle
	}

	// Rotation about Y
	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, m_cubeSize + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			2); // bottom segment intersects top two triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, -m_cubeSize - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			2); // top segment intersects bottom two triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect left two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect right two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect nearest two, one top and one bottom triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about y-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			6); // both segments intersect farthest two, one top and one bottom triangles
	}

	// Rotation about Z
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, offset + m_segmentRadius + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersect one of the top triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, -offset - m_segmentRadius - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the bottom triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, offset + m_segmentRadius + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersects one of the right side triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, -offset - m_segmentRadius - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the left side triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, 0.0, m_cubeSize * 0.5 + m_segmentRadius + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			4); // both segments intersect the two nearest triangles
	}

	{
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, 0.0, -m_cubeSize * 0.5 - m_segmentRadius - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection (rotated (about z-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			4); // both segments intersect the two farthest triangles
	}
}

TEST_F(SegmentMeshTriangleMeshContactCalculationTests, IntersectionWithSegmentAxisTest)
{
	using SurgSim::Math::makeRigidTransform;
	using SurgSim::Math::makeRigidTranslation;
	using SurgSim::Math::makeRotationQuaternion;

	double epsilonTrans = -0.001;

	auto segmentMeshShape = createSegmentMeshShape();
	auto meshShape = createCubeMeshShape();

	RigidTransform3d segmentMeshShapeTransform;
	RigidTransform3d meshShapeTransform;

	// No rotations
	{
		double offset = m_segmentRadius + 0.001;
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(offset, m_cubeSize + epsilonTrans, -offset));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersects one of the top two triangles
	}

	{
		double offset = m_segmentRadius + 0.001;
		segmentMeshShapeTransform =
			makeRigidTranslation(Vector3d(offset, -m_cubeSize - epsilonTrans, -offset));
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the bottom two triangles
	}

	// Rotation about X
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, offset + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about x-axis) segment mesh above box)",
			createSegmentMeshShape(), segmentMeshShapeTransform,
			createCubeMeshShape(), meshShapeTransform,
			1); // bottom segment intersects one of the top triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0, 0.0, -offset - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about x-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersect one bottom triangle
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, offset + epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about x-axis) segment mesh in front of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the nearer triangle
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 1.0, 0.0, 0.0,0.0, 0.0, -offset - epsilonTrans);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about x-axis) segment mesh behind box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1);  // bottom segment intersects one of the farther triangle
	}

	// Rotation about Y
	{
		double offset = m_segmentRadius + 0.001;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, offset, m_cubeSize + epsilonTrans, -offset);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about y-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersects one of the top two triangles
	}

	{
		double offset = m_segmentRadius + 0.001;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 1.0, 0.0, offset, -m_cubeSize - epsilonTrans, -offset);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about y-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the bottom two triangles
	}

	// Rotation about Z
	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, offset + epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about z-axis) segment mesh above box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersect one of the top triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, 0.0, -offset - epsilonTrans, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about z-axis) segment mesh below box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the bottom triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, offset + epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about z-axis) segment mesh right of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // bottom segment intersects one of the right side triangles
	}

	{
		double offset = (1.0 + std::sin(M_PI_4)) * m_cubeSize * 0.5;
		segmentMeshShapeTransform =
			buildRigidTransform(M_PI_4, 0.0, 0.0, 1.0, -offset - epsilonTrans, 0.0, 0.0);
		meshShapeTransform.setIdentity();
		testSegmentMeshTriangleMeshDCD("Intersection with axis (rotated (about z-axis) segment mesh left of box)",
			segmentMeshShape, segmentMeshShapeTransform,
			meshShape, meshShapeTransform,
			1); // top segment intersects one of the left side triangles
	}
}


}; // namespace Collision
}; // namespace Surgsim
