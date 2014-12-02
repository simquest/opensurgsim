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

// This code is based on David Eberly's paper:
// http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
// which is improving Brian Mirtich previous work (http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html)
// by making the assumption that the polyhedral mesh is composed of triangles.

#ifndef SURGSIM_MATH_MESHSHAPE_H
#define SURGSIM_MATH_MESHSHAPE_H

#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(MeshShape);


/// Mesh shape: shape made of a triangle mesh
/// The triangle mesh needs to be watertight to produce valid volume, center and second moment of
/// volume. If it is not the case and you need valid geometric properties, use SurfaceMeshShape instead.
/// Various geometrical properties (volume based) are computed from the triangle mesh using
/// David Eberly's work:
/// http://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
///
/// \note The internal mesh should not be modified, otherwise the geometric properties will be invalid.
/// \note Practical use cases:
/// \note * Fixed/Rigid object, the mesh will not change anyway.
/// \note * Deformable  object, the mesh will be updated, but the geometric properties will not be used.
///
/// \sa SurfaceMeshShape
class MeshShape : public Shape
{
public:
	/// Constructor
	MeshShape();

	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	/// \exception Raise an exception if the mesh is invalid
	template <class VertexData, class EdgeData, class TriangleData>
	explicit MeshShape(const SurgSim::DataStructures::TriangleMeshBase<VertexData, EdgeData, TriangleData>& mesh);

	SURGSIM_CLASSNAME(SurgSim::Math::MeshShape);

	/// \return the type of the shape
	virtual int getType() const override;

	/// Gets the initial mesh
	/// \return The collision mesh associated to this MeshShape
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getInitialMesh();

	/// Sets the mesh, alternatively you can use \sa load() to just load a mesh with a given name
	/// \param mesh the mesh asset to be used here
	void setInitialMesh(std::shared_ptr<SurgSim::Framework::Asset> mesh);


	/// Utility function, will create a mesh and load it if the path is correct
	/// \param filePath the name of the mesh
	/// \return whether the mesh was successfully loaded
	void loadInitialMesh(const std::string& filePath);



	/// Get mesh
	/// \return The collision mesh associated to this MeshShape
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getMesh();

	/// Get the volume of the shape
	/// \note this parameter is valid with respect to the initial mesh
	/// \return The volume of the shape (in m-3)
	virtual double getVolume() const override;

	/// Get the volumetric center of the shape
	/// \note this parameter is valid with respect to the initial mesh
	/// \return The center of the shape
	virtual Vector3d getCenter() const override;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \note this parameter is valid with respect to the initial mesh
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentOfVolume() const override;

	/// Set the object's global pose
	/// \param pose the rigid transform to apply
	void setPose(const SurgSim::Math::RigidTransform3d& pose);

	/// Update the AabbTree, which is an axis-aligned bounding box r-tree used to accelerate spatial searches
	void updateAabbTree();

	/// Get the AabbTree
	/// \return The object's associated AabbTree
	std::shared_ptr<SurgSim::DataStructures::AabbTree> getAabbTree();

	/// Check if this shape contains a valid mesh.
	/// Equals 'MeshShape::getMesh() != nullptr && MeshShape::getMesh()->isValid()'
	/// \return true if this shape contains a valid mesh; otherwise, false.
	bool isValid() const;

private:
	/// Compute useful volume integrals based on the triangle mesh, which
	/// are used to get the volume , center and second moment of volume.
	void computeVolumeIntegrals();
	/// Center (considering a uniform distribution in the mesh volume)
	SurgSim::Math::Vector3d m_center;

	/// Volume (in m^-3)
	double m_volume;

	/// Second moment of volume
	SurgSim::Math::Matrix33d m_secondMomentOfVolume;

	/// The triangle mesh contained by this shape.
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> m_mesh;

	/// The initial triangle mesh contained by this shape.
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> m_initialMesh;

	/// The aabb tree used to accelerate collision detection against the mesh
	std::shared_ptr<SurgSim::DataStructures::AabbTree> m_aabbTree;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/MeshShape-inl.h"

#endif // SURGSIM_MATH_MESHSHAPE_H
