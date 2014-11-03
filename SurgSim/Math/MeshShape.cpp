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
//

#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/DataStructures/TriangleMeshUtilities.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{
namespace Math
{
SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::MeshShape, MeshShape);

MeshShape::MeshShape() :
	m_volume(0.0),
	m_initialMesh(std::make_shared<SurgSim::DataStructures::TriangleMesh>())
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(
		SurgSim::Math::MeshShape,
		std::shared_ptr<SurgSim::Framework::Asset>,
		InitialMesh,
		getInitialMesh,
		setInitialMesh);

	// Enables the alternative use of the mesh file instead of the actual mesh object
	DecoderType decoder = std::bind(&MeshShape::loadInitialMesh,
									this,
									std::bind(&YAML::Node::as<std::string>, std::placeholders::_1));
	setDecoder("InitialMeshFileName", decoder);

	SetterType setter = std::bind(&MeshShape::loadInitialMesh,
								  this,
								  std::bind(SurgSim::Framework::convert<std::string>, std::placeholders::_1));

	setSetter("InitialMeshFileName", setter);

}

int MeshShape::getType()
{
	return SHAPE_TYPE_MESH;
}

bool MeshShape::isValid() const
{
	return (nullptr != m_mesh) && (m_mesh->isValid());
}

void MeshShape::loadInitialMesh(const std::string& filePath)
{
	auto mesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>();
	mesh->load(filePath);

	SURGSIM_ASSERT(mesh->isValid()) << "Loading failed " << filePath << " contains an invalid mesh.";

	setInitialMesh(mesh);
}

std::shared_ptr<SurgSim::DataStructures::TriangleMesh> MeshShape::getInitialMesh()
{
	return m_initialMesh;
}



std::shared_ptr<SurgSim::DataStructures::TriangleMesh> MeshShape::getMesh()
{
	return m_mesh;
}

double MeshShape::getVolume() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for MeshShape, so it cannot compute volume.";
	}
	return m_volume;
}

SurgSim::Math::Vector3d MeshShape::getCenter() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for MeshShape, so it cannot compute center.";
	}
	return m_center;
}

SurgSim::Math::Matrix33d MeshShape::getSecondMomentOfVolume() const
{
	if (nullptr == m_mesh)
	{
		SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getDefaultLogger()) <<
				"No mesh set for MeshShape, so it cannot compute SecondMomentOfVolume.";
	}
	return m_secondMomentOfVolume;
}

namespace
{
/// Compute various integral terms
/// \note Please refer to http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
/// \note for details about parameters
void computeIntegralTerms(const double w0, const double w1, const double w2,
						  double* f1, double* f2, double* f3, double* g0, double* g1, double* g2)
{
	double temp0 = w0 + w1;
	double temp1 = w0 * w0;
	double temp2 = temp1 + w1 * temp0;
	*f1 = temp0 + w2;
	*f2 = temp2 + w2 * (*f1);
	*f3 = w0 * temp1 + w1 * temp2 + w2 * (*f2);
	*g0 = (*f2) + w0 * ((*f1) + w0);
	*g1 = (*f2) + w1 * ((*f1) + w1);
	*g2 = (*f2) + w2 * ((*f1) + w2);
}
};

void MeshShape::computeVolumeIntegrals()
{
	Eigen::VectorXd multiplier(10);
	multiplier << 1.0 / 6.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 24.0, 1.0 / 60.0,
			   1.0 / 60.0, 1.0 / 60.0, 1.0 / 120.0, 1.0 / 120.0, 1.0 / 120.0;

	Eigen::VectorXd integral(10); // integral order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
	integral.setZero();

	for (auto const& triangle : m_mesh->getTriangles())
	{
		if (! triangle.isValid)
		{
			continue;
		}

		const Vector3d& v0 = m_mesh->getVertexPosition(triangle.verticesId[0]);
		const Vector3d& v1 = m_mesh->getVertexPosition(triangle.verticesId[1]);
		const Vector3d& v2 = m_mesh->getVertexPosition(triangle.verticesId[2]);

		// get edges and cross product of edges
		Vector3d normal = (v1 - v0).cross(v2 - v0);

		// compute integral terms
		double f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z;
		double g0x, g0y, g0z, g1x, g1y, g1z, g2x, g2y, g2z;
		computeIntegralTerms(v0.x(), v1.x(), v2.x(), &f1x, &f2x, &f3x, &g0x, &g1x, &g2x);
		computeIntegralTerms(v0.y(), v1.y(), v2.y(), &f1y, &f2y, &f3y, &g0y, &g1y, &g2y);
		computeIntegralTerms(v0.z(), v1.z(), v2.z(), &f1z, &f2z, &f3z, &g0z, &g1z, &g2z);

		// update integrals
		integral[0] += normal[0] * f1x;
		integral[1] += normal[0] * f2x;
		integral[2] += normal[1] * f2y;
		integral[3] += normal[2] * f2z;
		integral[4] += normal[0] * f3x;
		integral[5] += normal[1] * f3y;
		integral[6] += normal[2] * f3z;
		integral[7] += normal[0] * (v0.y() * g0x + v1.y() * g1x + v2.y() * g2x);
		integral[8] += normal[1] * (v0.z() * g0y + v1.z() * g1y + v2.z() * g2y);
		integral[9] += normal[2] * (v0.x() * g0z + v1.x() * g1z + v2.x() * g2z);
	}
	integral = integral.cwiseProduct(multiplier);

	// volume
	m_volume = integral[0];

	// center of mass
	if (m_volume != 0.0)
	{
		m_center = integral.segment(1, 3) / m_volume;
	}
	else
	{
		m_center.setZero();
	}

	// second moment of volume relative to center
	Vector3d centerSquared = m_center.cwiseProduct(m_center);
	m_secondMomentOfVolume(0, 0) = integral[5] + integral[6] - m_volume * (centerSquared.y() + centerSquared.z());
	m_secondMomentOfVolume(1, 1) = integral[4] + integral[6] - m_volume * (centerSquared.z() + centerSquared.x());
	m_secondMomentOfVolume(2, 2) = integral[4] + integral[5] - m_volume * (centerSquared.x() + centerSquared.y());
	m_secondMomentOfVolume(0, 1) = -(integral[7] - m_volume * m_center.x() * m_center.y());
	m_secondMomentOfVolume(1, 0) = m_secondMomentOfVolume(0, 1);
	m_secondMomentOfVolume(1, 2) = -(integral[8] - m_volume * m_center.y() * m_center.z());
	m_secondMomentOfVolume(2, 1) = m_secondMomentOfVolume(1, 2);
	m_secondMomentOfVolume(0, 2) = -(integral[9] - m_volume * m_center.z() * m_center.x());
	m_secondMomentOfVolume(2, 0) = m_secondMomentOfVolume(0, 2);
}

void MeshShape::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	m_mesh->copyWithTransform(pose, *m_initialMesh);
	updateAabbTree();
}

std::shared_ptr<SurgSim::DataStructures::AabbTree> MeshShape::getAabbTree()
{
	return m_aabbTree;
}

void MeshShape::updateAabbTree()
{
	m_aabbTree = std::make_shared<SurgSim::DataStructures::AabbTree>();

	auto const& triangles = m_mesh->getTriangles();
	for (size_t id = 0; id < triangles.size(); ++id)
	{
		if (triangles[id].isValid)
		{
			auto vertices = m_mesh->getTrianglePositions(id);
			m_aabbTree->add(SurgSim::Math::makeAabb(vertices[0], vertices[1], vertices[2]), id);
		}
	}
}

void MeshShape::setInitialMesh(std::shared_ptr<SurgSim::Framework::Asset> mesh)
{
	auto triangleMesh = std::dynamic_pointer_cast<SurgSim::DataStructures::TriangleMesh>(mesh);
	SURGSIM_ASSERT(triangleMesh != nullptr)
			<< "Mesh for MeshShape needs to be a TriangleMesh";
	SURGSIM_ASSERT(triangleMesh->isValid())
			<< "Mesh for MeshShape needs to be valid";
	m_initialMesh = triangleMesh;

	m_mesh = std::make_shared<SurgSim::DataStructures::TriangleMesh>(*m_initialMesh);

	updateAabbTree();
	computeVolumeIntegrals();
}

}; // namespace Math
}; // namespace SurgSim
