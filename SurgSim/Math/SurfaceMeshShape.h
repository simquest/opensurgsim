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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Framework/Convert.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{

class SurfaceMeshShape : public Shape
{
public:
	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	/// \exception Raise an exception if the mesh is invalid
	template <class VertexData, class EdgeData, class TriangleData>
	explicit SurfaceMeshShape(
		const SurgSim::DataStructures::TriangleMeshBase<VertexData, EdgeData, TriangleData>& mesh,
		double thikcness = 1e-2);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get mesh
	/// \return The collision mesh associated to this MeshShape
	const std::shared_ptr<SurgSim::DataStructures::TriangleMesh> getMesh() const;

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

	/// Volume (in m^-3)
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
