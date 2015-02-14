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
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{
SURGSIM_STATIC_REGISTRATION(SurfaceMeshShape);

/// SurfaceMeshShape defines a shape based on a mesh, like MeshShape.
/// But, unlike MeshShape, the mesh does not need to be watertight to produce valid volume, center and second moment of
/// volume. In the MeshShape case, these quantities are based on the notion of volume and are therefore undefined
/// if no volume if properly defined. In the SurfaceMeshShape case, these quantities are based on a surface
/// mesh, with a thickness (which is considered constant over the surface and should be multiple orders of magnitude
/// smaller than the two other dimesions, i.e. 1e-3 in practice). If the mesh is not closed or has holes, the class
/// will still produce valid geometric properties.
///
/// \note If not used in physics, there is no differences between using a SurfaceMeshShape or a MeshShape.
///
/// \note Any change on the mesh will invalidate the geometric properties.
/// \note Practical use cases:
/// \note * Fixed/Rigid object, the mesh will not change anyway.
/// \note * Deformable  object, the mesh will be updated, but the geometric properties will not be used.
///
/// \note The thickness should be multiple order of magnitude smaller than the other 2 dimensions of the mesh.
/// \note It should also not be smaller than 1e-5 to avoid formal and numerical issues when getting close to 0.
///
/// \note SurfaceMeshShape does not have any collision algorithm associated with it in SurgSim::Collision and
/// \note SurgSim::Physics::DcdCollision so far.
///
/// \sa MeshShape
class SurfaceMeshShape : public MeshShape
{
public:
	/// Constructor
	SurfaceMeshShape();

	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	/// \param thickness The thickness associated to this surface mesh
	/// \exception Raise an exception if the mesh is invalid
	template <class VertexData, class EdgeData, class TriangleData>
	SurfaceMeshShape(
		const SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData>& mesh,
		double thickness = 1e-2);

	SURGSIM_CLASSNAME(SurgSim::Math::SurfaceMeshShape);

	/// \return the type of the shape
	int getType() const override;

	/// Check if this shape contains a valid mesh and the thickness is at least 1e-5 (in meter,
	/// to avoid formal and numerical issues).
	/// \return True if this shape contains a valid mesh and thickness is at least 1e-5; Otherwise, false.
	bool isValid() const override;

protected:
	/// Compute useful volume integrals based on the triangle mesh, which
	/// are used to get the volume , center and second moment of volume.
	void computeVolumeIntegrals() override;

private:
	/// Surface mesh thickness
	double m_thickness;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/SurfaceMeshShape-inl.h"

#endif // SURGSIM_MATH_SURFACEMESHSHAPE_H
