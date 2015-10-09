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

#include "SurgSim/Collision/TriangleMeshParticlesDcdContact.h"

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::DataStructures::Location;
using SurgSim::Math::barycentricCoordinates;
using SurgSim::Math::distancePointTriangle;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Collision
{

TriangleMeshParticlesDcdContact::TriangleMeshParticlesDcdContact()
{
}

std::pair<int, int> TriangleMeshParticlesDcdContact::getShapeTypes()
{
	return std::pair<int, int>(Math::SHAPE_TYPE_MESH, Math::SHAPE_TYPE_PARTICLES);
}

void TriangleMeshParticlesDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	auto mesh = std::static_pointer_cast<Math::MeshShape>(pair->getFirst()->getPosedShape());
	auto particles = std::static_pointer_cast<Math::ParticlesShape>(pair->getSecond()->getPosedShape());

	auto contacts = calculateContact(*mesh, *particles);
	for (auto& contact : contacts)
	{
		pair->addContact(contact);
	}
}

std::list<std::shared_ptr<Contact>> TriangleMeshParticlesDcdContact::calculateContact(const Math::MeshShape& mesh,
								 const Math::ParticlesShape& particles)
{
	std::list<std::shared_ptr<Contact>> contacts;
	Vector3d closestPoint;
	Vector3d coordinates;
	const double particleRadius = particles.getRadius();

	auto intersections = mesh.getAabbTree()->spatialJoin(*particles.getAabbTree());
	for (auto& intersection : intersections)
	{
		std::list<size_t> candidateTriangles;
		std::list<size_t> candidateParticles;
		intersection.first->getIntersections(intersection.second->getAabb(), &candidateTriangles);
		intersection.second->getIntersections(intersection.first->getAabb(), &candidateParticles);
		for (auto& triangle : candidateTriangles)
		{
			const Vector3d& normal = mesh.getNormal(triangle);
			if (normal.isZero())
			{
				continue;
			}

			for (auto& particle : candidateParticles)
			{
				const Vector3d& particlePosition = particles.getVertexPosition(particle);
				auto vertices = mesh.getTrianglePositions(triangle);
				double distance = distancePointTriangle(particlePosition, vertices[0], vertices[1], vertices[2],
														&closestPoint);
				if (distance < particleRadius)
				{
					double depth = particleRadius - normal.dot(particlePosition - closestPoint);
					barycentricCoordinates(closestPoint, vertices[0], vertices[1], vertices[2], normal, &coordinates);
					auto penetrationPoints = std::make_pair(
												 Location(IndexedLocalCoordinate(triangle, coordinates),
														  DataStructures::Location::TRIANGLE),
												 Location(particle));
					contacts.push_back(std::make_shared<Contact>(
										   COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
										   Vector3d::Zero(), -normal, penetrationPoints));
				}
			}
		}
	}
	return contacts;
}

};
};

