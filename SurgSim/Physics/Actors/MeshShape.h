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

#ifndef SURGSIM_PHYSICS_MESHSHAPE_H
#define SURGSIM_PHYSICS_MESHSHAPE_H

#include <SurgSim/Physics/Actors/RigidShape.h>
#include <SurgSim/DataStructures/TriangleMesh.h>

namespace SurgSim
{

namespace Physics
{

class Mesh;

/// Mesh shape
class MeshShape: public RigidShape
{

public:
	/// Constructor
	/// \param mesh The mesh to build the shape from
	explicit MeshShape(const std::shared_ptr<Mesh> mesh)
	{
		m_mesh = mesh;
	}

	/// Get mesh
	/// \return The mesh associated to this MeshShape
	const std::shared_ptr<Mesh> getMesh() const
	{
		return m_mesh;
	}

	/// Calculate the volume of the mesh
	/// \return The volume of the mesh (in m-3)
	virtual double calculateVolume() const
	{
		return 0;
	}

	/// Calculate the mass center of the mesh
	/// \return The mass center of the mesh
	Vector3d calculateMassCenter() const
	{
		return Vector3d(0.0, 0.0, 0.0);
	}

	/// Calculate the inertia of the mesh
	/// \param rho The mass density (in Kg.m-3)
	/// \return The 3x3 symmetric inertia matrix of the mesh
	Matrix33d calculateInertia(double rho) const
	{
		return Matrix33d::Identity();
	}

private:
	/// Mesh associated to this MeshShape
	std::shared_ptr<Mesh> m_mesh;
};

}; /// Physics

}; /// SurgSim

#endif /// SURGSIM_PHYSICS_MESHSHAPE_H

