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

#include "SurgSim/Math/MeshShape.h"

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeData.h"
#include "SurgSim/Framework/Assert.h"

using SurgSim::DataStructures::EmptyData;
using SurgSim::DataStructures::NormalData;


template<>
std::string SurgSim::DataStructures::TriangleMesh<EmptyData, EmptyData, NormalData>
::m_className = "SurgSim::Math::MeshShape";

namespace SurgSim
{
namespace Math
{

SURGSIM_REGISTER(SurgSim::Math::Shape, SurgSim::Math::MeshShape, MeshShape);

MeshShape::MeshShape() :
	m_center(Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
	m_volume(std::numeric_limits<double>::quiet_NaN()),
	m_secondMomentOfVolume(Matrix33d::Constant(std::numeric_limits<double>::quiet_NaN())),
	m_initialMesh(std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>()),
	m_validPose(false)
{
	m_pose.linear() = Matrix33d::Constant(std::numeric_limits<double>::quiet_NaN());
	m_pose.translation() = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

const SurgSim::Math::Vector3d& MeshShape::getNormal(size_t triangleId)
{
	return getTriangle(triangleId).data.normal;
}

bool MeshShape::calculateNormals()
{
	bool result = true;
	for (size_t i = 0; i < getNumTriangles(); ++i)
	{
		const SurgSim::Math::Vector3d& vertex0 = getVertexPosition(getTriangle(i).verticesId[0]);
		const SurgSim::Math::Vector3d& vertex1 = getVertexPosition(getTriangle(i).verticesId[1]);
		const SurgSim::Math::Vector3d& vertex2 = getVertexPosition(getTriangle(i).verticesId[2]);

		// Calculate normal vector
		SurgSim::Math::Vector3d normal = (vertex1 - vertex0).cross(vertex2 - vertex0);
		if (normal.isZero())
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getLogger("Math/MeshShape")) <<
					"MeshShape::calculateNormals unable to calculate normals. For example, for triangle #" << i <<
					" with vertices:" << std::endl << "1: " << vertex0.transpose() << std::endl <<
					"2: " << vertex1.transpose() << std::endl << "3: " << vertex2.transpose();
			result = false;
			break;
		}
		normal.normalize();
		getTriangle(i).data.normal = normal;
	}
	return result;
}

bool MeshShape::doUpdate()
{
	updateAabbTree();
	return calculateNormals();
}

bool MeshShape::doLoad(const std::string& fileName)
{
	if (!SurgSim::DataStructures::TriangleMesh<EmptyData, EmptyData, NormalData>::doLoad(fileName))
	{
		return false;
	}
	m_initialMesh = std::make_shared<SurgSim::DataStructures::TriangleMeshPlain>(*shared_from_this());
	updateAabbTree();
	computeVolumeIntegrals();
	return calculateNormals();
}

int MeshShape::getType() const
{
	return SHAPE_TYPE_MESH;
}

double MeshShape::getVolume() const
{
	return m_volume;
}

bool MeshShape::isValid() const
{
	return SurgSim::DataStructures::TriangleMesh<EmptyData, EmptyData, NormalData>::isValid();
}

SurgSim::Math::Vector3d MeshShape::getCenter() const
{
	return m_center;
}

SurgSim::Math::Matrix33d MeshShape::getSecondMomentOfVolume() const
{
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

	for (auto const& triangle : getTriangles())
	{
		if (! triangle.isValid)
		{
			continue;
		}

		const Vector3d& v0 = getVertexPosition(triangle.verticesId[0]);
		const Vector3d& v1 = getVertexPosition(triangle.verticesId[1]);
		const Vector3d& v2 = getVertexPosition(triangle.verticesId[2]);

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

bool MeshShape::setPose(const SurgSim::Math::RigidTransform3d& pose)
{
	SURGSIM_ASSERT(getNumVertices() == m_initialMesh->getNumVertices())
			<< "The initial mesh must have the same number of vertices as the MeshShape for setPose.";
	if (!m_validPose || !pose.isApprox(m_pose))
	{
		m_pose = pose;
		auto targetVertex = getVertices().begin();
		auto const& vertices = m_initialMesh->getVertices();
		for (auto it = vertices.cbegin(); it != vertices.cend(); ++it)
		{
			targetVertex->position = pose * it->position;
			++targetVertex;
		}
		m_validPose = doUpdate();
	}
	return m_validPose;
}

std::shared_ptr<SurgSim::DataStructures::AabbTree> MeshShape::getAabbTree()
{
	return m_aabbTree;
}

void MeshShape::updateAabbTree()
{
	m_aabbTree = std::make_shared<SurgSim::DataStructures::AabbTree>();

	std::list<DataStructures::AabbTreeData::Item> items;

	auto const& triangles = getTriangles();

	for (size_t id = 0, count = triangles.size(); id < count; ++id)
	{
		if (triangles[id].isValid)
		{
			const auto& vertices = getTrianglePositions(id);
			Aabbd aabb(SurgSim::Math::makeAabb(vertices[0], vertices[1], vertices[2]));
			items.emplace_back(std::make_pair(std::move(aabb), id));
		}
	}
	m_aabbTree->set(std::move(items));
}

}; // namespace Math
}; // namespace SurgSim
