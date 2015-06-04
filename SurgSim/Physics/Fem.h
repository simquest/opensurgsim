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

#ifndef SURGSIM_PHYSICS_FEM_H
#define SURGSIM_PHYSICS_FEM_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/MeshElement.h"
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

// These 3 structs are to differentiate for m_classname in TriangleMesh
struct Fem1DVectorData : public RotationVectorData{};
struct Fem2DVectorData : public RotationVectorData{};
struct Fem3DVectorData : public RotationVectorData{};

struct FemElementParameter
{
	virtual ~FemElementParameter(){}

	std::string type;   // “LinearBeam”, “CorotationalTetrahedron”…

	double youngModulus;
	double poissonRatio;
	double massDensity;
};

struct FemElement1DParameter : public FemElementParameter
{
	double radius;
	bool enableShear;
};

struct FemElement2DParameter : public FemElementParameter
{
	double thickness;
};

struct FemElement3DParameter : public FemElementParameter {};
} // namespace FemElementStructs

typedef SurgSim::DataStructures::MeshElement<2, std::shared_ptr<FemElementStructs::FemElement1DParameter>> BeamType;
typedef SurgSim::DataStructures::MeshElement<3, std::shared_ptr<FemElementStructs::FemElement2DParameter>> TriangleType;
typedef SurgSim::DataStructures::MeshElement<4, std::shared_ptr<FemElementStructs::FemElement3DParameter>> TetrahedronType;
typedef SurgSim::DataStructures::MeshElement<8, std::shared_ptr<FemElementStructs::FemElement3DParameter>> CubeType;

/// Base class for a data structure for holding FEM mesh data of different dimensions
///
/// Fem itself should not be used directly itself as it contains no overrde for doLoad since the implementation is
/// depedent on the dimension of the FEM you are trying to load. Each dimension overrides the doLoad function present
/// in Asset using its own version of an FemPlyReaderDelegate. Each dimension supports loading both linear and
/// corotational models.
///
/// \tparam VertexData  Type of extra data stored in each vertex
/// \tparam	EdgeData	Type of extra data stored in each edge
/// \tparam	TriangleData	Type of extra data stored in each triangle
/// \tparam Element		Type of FEM element the mesh will be storing
/// \sa	TriangleMesh
template <class VertexData, class EdgeData, class TriangleData, class Element>
class Fem : public SurgSim::DataStructures::TriangleMesh<VertexData, EdgeData, TriangleData>
{
public:
	/// Default constructor
	Fem();

	/// Adds FEM element to mesh of Element template type
	/// \param element A shared pointer of the Element template type
	/// \return The new size of the vector of elements
	size_t addElement(std::shared_ptr<Element> element);

	/// Gets number of FEM elements in the mesh
	/// \return The number of FEM elements stored
	size_t getNumElements() const;

	/// Gets entire FEM element vector
	/// \return A const vector of all FEM elements stored in the mesh
	const std::vector<std::shared_ptr<Element>>& getElements() const;
	std::vector<std::shared_ptr<Element>>& getElements();

	/// Gets entire FEM element vector (non-const)
	/// \return A vector of all FEM elements stored in the mesh
	std::shared_ptr<Element> getElement(size_t id) const;

	/// Removes FEM element from mesh
	/// \param id The element to remove
	void removeElement(size_t id);

	size_t addBoundaryCondition(size_t boundaryCondition);

	const std::vector<size_t>& getBoundaryConditions() const;
	std::vector<size_t>& getBoundaryConditions();

	size_t getBoundaryCondition(size_t id) const;

	void removeBoundaryCondition(size_t id);

protected:
	/// Vector of individual elements
	std::vector<std::shared_ptr<Element>> m_elements;
	std::vector<size_t> m_boundaryConditions;
};

/// Fem class data structure implementation for 1-Dimensional FEMs
/// \sa Fem
class Fem1D : public Fem<FemElementStructs::Fem1DVectorData, EmptyData, EmptyData, BeamType>
{
public:
	Fem1D();

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

/// Fem class data structure implementation for 2-Dimensional FEMs
/// \sa Fem
class Fem2D : public Fem<FemElementStructs::Fem2DVectorData, EmptyData, EmptyData, TriangleType>
{
public:
	Fem2D();

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;
};

/// Fem class data structure implementation for 3-Dimensional FEMs
/// \sa Fem
class Fem3D : public Fem<FemElementStructs::Fem3DVectorData, EmptyData, EmptyData, TetrahedronType>
{
public:
	Fem3D();

	size_t addCube(std::shared_ptr<CubeType> cube);

	size_t getNumCubes() const;

	const std::vector<std::shared_ptr<CubeType> >& getCubes() const;
	std::vector<std::shared_ptr<CubeType>>& getCubes();

	std::shared_ptr<CubeType> getCube(size_t id) const;

	void removeCube(size_t id);

protected:
	// Asset API override
	bool doLoad(const std::string& filePath) override;

	std::vector<std::shared_ptr<CubeType>> m_cubeElements;
};

} // namespace DataStructures
} // namespace SurgSim

#include "SurgSim/Physics/Fem-inl.h"

#endif // SURGSIM_PHYSICS_FEM_H
