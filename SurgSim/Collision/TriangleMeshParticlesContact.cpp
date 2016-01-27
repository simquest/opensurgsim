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

#include "SurgSim/Collision/TriangleMeshParticlesContact.h"

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

std::pair<int, int> TriangleMeshParticlesContact::getShapeTypes()
{
	return std::pair<int, int>(Math::SHAPE_TYPE_MESH, Math::SHAPE_TYPE_PARTICLES);
}

std::list<std::shared_ptr<Contact>> TriangleMeshParticlesContact::calculateDcdContact(
									 const Math::MeshShape& mesh,
									 const Math::RigidTransform3d& meshPose,
									 const Math::ParticlesShape& particles,
									 const Math::RigidTransform3d& particlesPose) const
{
	auto meshTransformed = getCachedShape(mesh, meshPose);
	auto particlesTransformed = getCachedShape(particles, particlesPose);
	std::list<std::shared_ptr<Contact>> contacts;
	Vector3d closestPoint;
	Vector3d coordinates;
	const double particleRadius = particlesTransformed->getRadius();

	auto intersections = meshTransformed->getAabbTree()->spatialJoin(*particlesTransformed->getAabbTree());
	for (auto& intersection : intersections)
	{
		std::list<size_t> candidateTriangles;
		std::list<size_t> candidateParticles;
		intersection.first->getIntersections(intersection.second->getAabb(), &candidateTriangles);
		intersection.second->getIntersections(intersection.first->getAabb(), &candidateParticles);
		for (auto& triangle : candidateTriangles)
		{
			const Vector3d& normal = meshTransformed->getNormal(triangle);
			if (normal.isZero())
			{
				continue;
			}

			for (auto& particle : candidateParticles)
			{
				const Vector3d& particlePosition = particlesTransformed->getVertexPosition(particle);
				auto vertices = meshTransformed->getTrianglePositions(triangle);
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

