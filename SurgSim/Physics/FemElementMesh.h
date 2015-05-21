// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_PHYSICS_FEMELEMENTMESH_H
#define SURGSIM_PHYSICS_FEMELEMENTMESH_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

using SurgSim::DataStructures::EmptyData;

namespace SurgSim
{
namespace Physics
{

namespace FemElementStructs
{
struct RotationVectorData
{
	bool operator==(const RotationVectorData& rhs) const
	{
		return (thetaX == rhs.thetaX && thetaY == rhs.thetaY && thetaZ == rhs.thetaZ);
	}
	double thetaX;
	double thetaY;
	double thetaZ;
};

struct FemElement
{
	std::string type;   // “LinearBeam”, “CorotationalTetrahedron”…

	std::vector<size_t> nodeIds;
	double youngModulus;
	double poissonRatio;
	double massDensity;
};

struct FemElement1D : public FemElement
{
	double radius;
	bool enableShear;
};

struct FemElement2D : public FemElement
{
	double thickness;
};

struct FemElement3D : public FemElement {};
} // namespace FemElementStructs

template <class VertexData, class EdgeData, class TriangleData, class Element>
class FemElementMesh : public SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData>
{
public:
	FemElementMesh();

	size_t addFemElement(std::shared_ptr<Element> element);

	size_t getNumElements() const;

	const std::vector<std::shared_ptr<Element>>& getFemElements() const;
	std::vector<std::shared_ptr<Element>>& getFemElements();

	std::shared_ptr<Element> getFemElement(size_t id) const;

	void removeFemElement(size_t id);

	size_t addBoundaryCondition(size_t boundaryCondition);

	const std::vector<size_t>& getBoundaryConditions() const;
	std::vector<size_t>& getBoundaryConditions();

	size_t getBoundaryCondition(size_t id) const;

	void removeBoundaryCondition(size_t id);

protected:
	std::vector<std::shared_ptr<Element>> m_femElements;
	std::vector<size_t> m_boundaryConditions;
};

class FemElement1DMesh : public FemElementMesh<FemElementStructs::RotationVectorData,
												EmptyData, EmptyData, FemElementStructs::FemElement1D>
{
public:
	FemElement1DMesh();

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

class FemElement2DMesh : public FemElementMesh<FemElementStructs::RotationVectorData,
												EmptyData, EmptyData, FemElementStructs::FemElement2D>
{
public:
	FemElement2DMesh();

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

class FemElement3DMesh : public FemElementMesh<EmptyData, EmptyData, EmptyData,
												FemElementStructs::FemElement3D>
{
public:
	FemElement3DMesh();

protected:

	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

} // namespace DataStructures
} // namespace SurgSim

#include "SurgSim/Physics/FemElementMesh-inl.h"

#endif // SURGSIM_PHYSICS_FEMELEMENTMESH_H
