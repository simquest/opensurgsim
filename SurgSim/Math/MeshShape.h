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

// Parts of the code in this file are based on research and
// public-domain code by Brian Mirtich.
// (http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html)


#ifndef SURGSIM_MATH_MESHSHAPE_H
#define SURGSIM_MATH_MESHSHAPE_H

#include "SurgSim/Math/Shape.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

#include "SurgSim/Serialize/Convert.h"

namespace SurgSim
{

namespace Math
{

/// Mesh shape: shape made of a triangle mesh
/// Various physical properties are computed from the triangle mesh using
/// work by Brian Mirtich.
/// http://www.cs.berkeley.edu/~jfc/mirtich/massProps.html
template <class VertexData, class EdgeData, class TriangleData>
class MeshShape : public Shape
{
	/// Type TriMesh for convenience
	typedef SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData> TriMesh;

public:
	/// Constructor
	/// \param mesh The triangle mesh to build the shape from
	explicit MeshShape(const std::shared_ptr<TriMesh> mesh);

	/// \return the type of the shape
	virtual int getType() override;

	/// Get mesh
	/// \return The triangle mesh associated to this MeshShape
	const std::shared_ptr<TriMesh> getMesh() const;

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

	/// Compute various integrations over projection of face
	/// \param face A triangle
	void computeProjectionIntegrals(const typename TriMesh::TriangleType& face);

	/// Compute various integrations on a face
	/// \param face A triangle
	void computeFaceIntegrals(const typename TriMesh::TriangleType& face);

	/// Compute various volume integrals over the triangle mesh
	void computeVolumeIntegrals();

	/// Data structure from Brian Mirtich
	int m_alpha;
	int m_beta;
	int m_gamma;

	//// projection integrals (Data structure from Brian Mirtich)
	double m_P1, m_Pa, m_Pb, m_Paa, m_Pab, m_Pbb, m_Paaa, m_Paab, m_Pabb, m_Pbbb;

	/// face integrals (Data structure from Brian Mirtich)
	double m_Fa, m_Fb, m_Fc, m_Faa, m_Fbb, m_Fcc, m_Faaa, m_Fbbb, m_Fccc, m_Faab, m_Fbbc, m_Fcca;

	/// volume integrals (Data structure from Brian Mirtich)
	double m_T0, m_T1[3], m_T2[3], m_TP[3];

	/// Triangle mesh associated to this MeshShape
	std::shared_ptr<TriMesh> m_mesh;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/MeshShape-inl.h"

#endif // SURGSIM_MATH_MESHSHAPE_H
