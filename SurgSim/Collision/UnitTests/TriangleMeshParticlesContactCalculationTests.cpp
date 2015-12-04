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

#include <gtest/gtest.h>
#include <unordered_set>

#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/TriangleMeshParticlesContact.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/ParticlesShape.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::MeshShape;
using SurgSim::Math::ParticlesShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Collision
{

void doCollisionTest(std::shared_ptr<ParticlesShape> particles, std::shared_ptr<MeshShape> mesh,
		const RigidTransform3d& meshPose, const size_t numContactingParticles)
{
	auto meshRep = std::make_shared<ShapeCollisionRepresentation>("Mesh");
	meshRep->setShape(mesh);
	meshRep->setLocalPose(meshPose);

	auto particlesRep = std::make_shared<ShapeCollisionRepresentation>("Particles");
	particlesRep->setShape(particles);

	TriangleMeshParticlesContact calcContact;
	auto pair = std::make_shared<CollisionPair>(meshRep, particlesRep);
	calcContact.calculateContact(pair);

	std::unordered_set<size_t> contactingParticles;
	for (auto& contact : pair->getContacts())
	{
		size_t particleIndex = contact->penetrationPoints.second.index.getValue();
		ASSERT_LE(0u, particleIndex);
		ASSERT_GT(particles->getVertices().size(), particleIndex);
		contactingParticles.insert(particleIndex);

		Vector3d particlesToMesh = meshPose * mesh->getCenter() - particles->getVertex(particleIndex).position;
		if (!particlesToMesh.isZero())
		{
			// Check that each normal is pointing into the mesh
			EXPECT_LT(0.0, contact->normal.dot(particlesToMesh));
		}

		// Check that the depth is sane
		EXPECT_LT(0.0, contact->depth);
		EXPECT_GE(particles->getRadius() + DistanceEpsilon, contact->depth);
	}
	EXPECT_EQ(numContactingParticles, contactingParticles.size());
}

TEST(TriangleMeshParticlesContactCalculationTests, CanConstruct)
{
	EXPECT_NO_THROW({ TriangleMeshParticlesContact calculation;});
}

TEST(TriangleMeshParticlesContactCalculationTests, CollisionTests)
{
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	auto cube = std::make_shared<DataStructures::TriangleMeshPlain>();
	cube->load("Geometry/Cube.ply");

	{
		SCOPED_TRACE("No intersection, no particles");
		auto mesh = std::make_shared<MeshShape>(*cube);
		auto meshPose = RigidTransform3d::Identity();
		auto particles = std::make_shared<ParticlesShape>(0.5);
		doCollisionTest(particles, mesh, meshPose, 0);
	}
	{
		SCOPED_TRACE("No Intersection with particles");
		auto mesh = std::make_shared<MeshShape>(*cube);
		auto meshPose = RigidTransform3d::Identity();
		auto particles = std::make_shared<ParticlesShape>(0.5);
		particles->addVertex(ParticlesShape::VertexType(Vector3d::Constant(100.0)));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitX() * 2.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitY() * -3.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitZ() * 4.0));
		particles->update();
		doCollisionTest(particles, mesh, meshPose, 0);
	}
	{
		SCOPED_TRACE("Intersection with one particle");
		auto mesh = std::make_shared<MeshShape>(*cube);
		auto meshPose = RigidTransform3d::Identity();
		auto particles = std::make_shared<ParticlesShape>(0.5);
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitX()));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::Constant(100.0)));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitX() * 2.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitY() * -3.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitZ() * 4.0));
		particles->update();
		doCollisionTest(particles, mesh, meshPose, 1);
	}
	{
		SCOPED_TRACE("No intersection with posed mesh");
		auto mesh = std::make_shared<MeshShape>(*cube);
		auto meshPose = makeRigidTransform(Eigen::AngleAxisd(M_PI_4, Vector3d::UnitZ()).matrix(), Vector3d::Zero());
		auto particles = std::make_shared<ParticlesShape>(0.5);
		particles->addVertex(ParticlesShape::VertexType(Vector3d::Ones() * 2.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::Ones() * -2.0));
		particles->addVertex(ParticlesShape::VertexType(Vector3d(1.15, 1.15, 0.0)));
		particles->update();
		doCollisionTest(particles, mesh, meshPose, 0);
	}
	{
		SCOPED_TRACE("Intersection with posed mesh");
		auto mesh = std::make_shared<MeshShape>(*cube);
		auto meshPose = makeRigidTransform(Eigen::AngleAxisd(M_PI_4, Vector3d::UnitZ()).matrix(),
				Vector3d::UnitX() * 10.0);
		auto particles = std::make_shared<ParticlesShape>(0.5);
		particles->addVertex(ParticlesShape::VertexType(Vector3d::UnitX() * 11.6));
		particles->addVertex(ParticlesShape::VertexType(Vector3d(10.0, 1.9, 0.0)));
		particles->addVertex(ParticlesShape::VertexType(Vector3d::Zero()));
		particles->update();
		doCollisionTest(particles, mesh, meshPose, 2);
	}
}

};
};
