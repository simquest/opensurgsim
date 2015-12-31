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

#ifndef SURGSIM_MATH_SEGMENTMESHSHAPE_H
#define SURGSIM_MATH_SEGMENTMESHSHAPE_H

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/SegmentMesh.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(SegmentMeshShape);

/// SegmentMeshShape defines a shape based on a mesh, like MeshShape.
/// But, unlike MeshShape, the mesh does not have any triangle topology. It only consists of edges.
///
/// \sa MeshShape
class SegmentMeshShape : public Shape, public DataStructures::SegmentMeshPlain
{
public:
	/// Constructor
	SegmentMeshShape();

	/// Copy constructor
	/// \param other The SegmentMeshShape to be copied from
	explicit SegmentMeshShape(const SegmentMeshShape& other);

	/// Constructor
	/// \param mesh The segment mesh to build the shape from
	/// \param radius The radius associated to this surface mesh
	/// \note The default radius is positive EPSILON to be relevant in collision detection calculations.
	template <class VertexData, class EdgeData>
	explicit SegmentMeshShape(const DataStructures::SegmentMesh<VertexData, EdgeData>& mesh,
							  double radius = Geometry::DistanceEpsilon);

	SURGSIM_CLASSNAME(SurgSim::Math::SegmentMeshShape);

	int getType() const override;
	double getVolume() const override;
	Vector3d getCenter() const override;
	Matrix33d getSecondMomentOfVolume() const override;
	bool isValid() const override;

	/// \param radius The radius of the segments.
	void setRadius(double radius);

	/// \return The radius of the segments.
	double getRadius() const;

	/// \return The object's associated AabbTree
	std::shared_ptr<const DataStructures::AabbTree> getAabbTree() const;

	bool isTransformable() const override;

	std::shared_ptr<Shape> getTransformed(const RigidTransform3d& pose) const override;

protected:
	bool doUpdate() override;
	bool doLoad(const std::string& fileName) override;

	/// Update the AabbTree, which is an axis-aligned bounding box r-tree used to accelerate spatial searches
	void updateAabbTree();

private:
	/// Segment radius
	double m_radius;

	/// The aabb tree used to accelerate collision detection against the mesh
	std::shared_ptr<DataStructures::AabbTree> m_aabbTree;

	/// Half extent of the AABB of the sphere at one of the segment end.
	Vector3d m_segmentEndBoundingBoxHalfExtent;
};

} // Math
} // SurgSim

#include "SurgSim/Math/SegmentMeshShape-inl.h"

#endif // SURGSIM_MATH_SEGMENTMESHSHAPE_H
