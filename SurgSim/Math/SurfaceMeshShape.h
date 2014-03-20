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

#ifndef SURGSIM_MATH_SURFACEMESHSHAPE_H
#define SURGSIM_MATH_SURFACEMESHSHAPE_H

#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{

/// SurfaceMeshShape defines a shape based on a mesh, like MeshShape.
/// But, unlike MeshShape, the mesh does not need to be watertight to produce valid volume, center and second moment of
/// volume. In the MeshShape case, these quantities are based on the notion of volume and are therefore undefined
/// if no volume if properly defined. In the SurfaceMeshShape case, these quantities are based on the mesh as a surface
/// mesh, with a thickness (which should be very small compared to the mesh size, i.e. 1e-3 in practice). If the mesh
/// is not closed or has holes, the class will still produce valid geometric properties.
///
/// \note If not used in physics, there is no differences between using a SurfaceMeshShape or a MeshShape.
///
/// \note The internal mesh should not be modified, otherwise the geometric properties will be invalid.
/// \note Practical use cases:
/// \note * Fixed/Rigid object, the mesh will not change anyway.
/// \note * Deformable  object, the mesh will be updated, but the geometric properties will not be used.
///
/// \sa MeshShape
class SurfaceMeshShape : public Shape
{
public:
	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	/// \param thickness The thickness associated to this surface mesh
	/// \exception Raise an exception if the mesh is invalid
	template <class VertexData, class EdgeData, class TriangleData>
	SurfaceMeshShape(
		const SurgSim::DataStructures::TriangleMeshBase<VertexData, EdgeData, TriangleData>& mesh,
		double thickness = 1e-2);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get mesh
	/// \return The collision mesh associated to this MeshShape
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getMesh();

	/// Get the volume of the shape
	/// \return The volume of the shape (in m-3)
	virtual double getVolume() const override;

	/// Get the volumetric center of the shape
	/// \return The center of the shape
	virtual Vector3d getCenter() const override;

	/// Get the second central moment of the volume, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentOfVolume() const override;

	/// Get the complete name of the mesh
	virtual std::string getClassName() override;

private:

	/// Compute useful volume integrals based on the triangle mesh, which
	/// are used to get the volume , center and second moment of volume.
	void computeVolumeIntegrals();

	/// Center (considering a uniform distribution in the mesh volume)
	SurgSim::Math::Vector3d m_center;

	/// Volume (in m-3)
	double m_volume;

	/// Second moment of volume
	SurgSim::Math::Matrix33d m_secondMomentOfVolume;

	/// Collision mesh associated to this MeshShape
	std::shared_ptr<SurgSim::DataStructures::TriangleMesh> m_mesh;

	/// Surface mesh thickness
	double m_thickness;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/SurfaceMeshShape-inl.h"

#endif // SURGSIM_MATH_SURFACEMESHSHAPE_H
