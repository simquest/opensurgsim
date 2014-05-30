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

#ifndef SURGSIM_MATH_OCTREESHAPE_H
#define SURGSIM_MATH_OCTREESHAPE_H

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/OctreeNode.h"
#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Math/Shape.h"

namespace SurgSim
{

namespace Math
{

/// Octree Shape
/// A defined by an octree data structure
class OctreeShape : public Shape, public SurgSim::Framework::Asset
{
public:
	typedef SurgSim::DataStructures::OctreeNode<SurgSim::DataStructures::EmptyData> NodeType;

	/// Constructor
	OctreeShape();

	SURGSIM_CLASSNAME(SurgSim::Math::OctreeShape);

	/// Construct an OctreeShape by copying data from an OctreeNode
	/// NOTE: The Data stored in the octree node will not be copied into the
	/// OctreeShape.
	/// \tparam T octree node data structure to build Octree Shape from
	/// \param node octree node data structure to build Octree Shape from
	template<class T>
	explicit OctreeShape(const SurgSim::DataStructures::OctreeNode<T>& node);

	/// Destructor
	virtual ~OctreeShape();

	/// \return the type of shape
	virtual int getType() override;

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

	/// Get the root node
	/// \return the octree root node of this shape
	std::shared_ptr<NodeType> getRootNode();

	/// const version to get the root node
	/// \return A const reference of the shared pointer, which points to the octree root node of this shape.
	const std::shared_ptr<const NodeType> getRootNode() const;

	/// Set the root node
	/// \param node the octree root node of this shape
	void setRootNode(std::shared_ptr<NodeType> node);

	virtual bool doInitialize(const std::string& fileName) override;

private:
	/// Root node of the octree datastructure
	std::shared_ptr<NodeType> m_rootNode;
};

}; // Math
}; // SurgSim

#include "SurgSim/Math/OctreeShape-inl.h"

#endif // SURGSIM_MATH_OCTREESHAPE_H