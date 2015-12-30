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

#ifndef SURGSIM_MATH_MESHSHAPE_H
#define SURGSIM_MATH_MESHSHAPE_H

#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/NormalData.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{
namespace DataStructures
{
class AabbTree;
}

namespace Math
{

SURGSIM_STATIC_REGISTRATION(MeshShape);

/// Mesh shape: shape made of a triangle mesh
/// The triangle mesh needs to be watertight to produce valid volume, center and second moment of
/// volume. If it is not the case and you need valid geometric properties, use SurfaceMeshShape instead.
/// Various geometrical properties (volume based) are computed from the triangle mesh using
/// David Eberly's work (http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf)
/// which is improving Brian Mirtich previous work (http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html)
/// by making the assumption that the polyhedral mesh is composed of triangles.
///
/// \note The internal mesh should not be modified, otherwise the geometric properties will be invalid.
/// \note Practical use cases:
/// \note * Fixed/Rigid object, the mesh will not change anyway.
/// \note * Deformable  object, the mesh will be updated, but the geometric properties will not be used.
///
/// \sa SurfaceMeshShape
class MeshShape : public Shape, public SurgSim::DataStructures::TriangleMesh<SurgSim::DataStructures::EmptyData,
	SurgSim::DataStructures::EmptyData, SurgSim::DataStructures::NormalData>
{
public:
	/// Constructor
	MeshShape();

	/// Copy constructor
	/// \param other The MeshShape to be copied from
	explicit MeshShape(const MeshShape& other);

	/// Copy constructor when the template data is a different type
	/// \tparam	VertexData Type of extra data stored in each vertex
	/// \tparam	EdgeData Type of extra data stored in each edge
	/// \tparam	TriangleData Type of extra data stored in each triangle
	/// \param other The mesh to be copied from. Vertex, edge and triangle data will not be copied
	template <class V, class E, class T>
	explicit MeshShape(const SurgSim::DataStructures::TriangleMesh<V, E, T>& other);

	SURGSIM_CLASSNAME(SurgSim::Math::MeshShape);

	int getType() const override;

	/// Get normal for triangle.
	/// \param triangleId The triangle to get normal.
	/// \return The normal for the triangle with given ID.
	const SurgSim::Math::Vector3d& getNormal(size_t triangleId) const;

	double getVolume() const override;

	Vector3d getCenter() const override;

	Matrix33d getSecondMomentOfVolume() const override;

	std::shared_ptr<Shape> getTransformed(const RigidTransform3d& pose) const override;

	/// Get the AabbTree
	/// \return The object's associated AabbTree
	const std::shared_ptr<const SurgSim::DataStructures::AabbTree> getAabbTree() const;

	bool isValid() const override;

	bool isTransformable() const override;

protected:
	bool doUpdate() override;
	bool doLoad(const std::string& fileName) override;

	/// Calculate normals for all triangles.
	/// \note Normals will be normalized.
	/// \return true on success, or false if any triangle has an indeterminate normal.
	bool calculateNormals();

	/// Update the AabbTree, which is an axis-aligned bounding box r-tree used to accelerate spatial searches
	void updateAabbTree();

	/// Compute useful volume integrals based on the triangle mesh, which
	/// are used to get the volume , center and second moment of volume.
	virtual void computeVolumeIntegrals();

	/// Center (considering a uniform distribution in the mesh volume)
	SurgSim::Math::Vector3d m_center;

	/// Volume (in m^-3)
	double m_volume;

	/// Second moment of volume
	SurgSim::Math::Matrix33d m_secondMomentOfVolume;

private:
	/// The aabb tree used to accelerate collision detection against the mesh
	std::shared_ptr<SurgSim::DataStructures::AabbTree> m_aabbTree;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/MeshShape-inl.h"

#endif // SURGSIM_MATH_MESHSHAPE_H
