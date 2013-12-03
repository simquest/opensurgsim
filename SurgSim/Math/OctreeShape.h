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

#include <SurgSim/DataStructures/OctreeNode.h>
#include <SurgSim/Math/Shape.h>

namespace SurgSim
{

namespace Math
{

template<class NodeData>
class OctreeShape : public Shape
{
public:

	/// Constructor
	/// \param node octree root node
	explicit OctreeShape(std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> node = nullptr);

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

	/// Get the second central moment of the shape, commonly used
	/// to calculate the moment of inertia matrix
	/// \return The 3x3 symmetric second moment matrix
	virtual Matrix33d getSecondMomentMatrix() const override;

	/// Get the root node
	/// \return the octree root node of this shape
	virtual std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> getRootNode();

	/// Set the root node
	/// \param node the octree root node of this shape
	virtual void setRootNode(std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> node);

	/// Get the name of the class
	/// \return the class name
	virtual std::string getClassName() override;

private:
	/// Root node of the octree datastructure
	std::shared_ptr<SurgSim::DataStructures::OctreeNode<NodeData>> m_rootNode;
};


}; // Math

}; // SurgSim

#include <SurgSim/Math/OctreeShape-inl.h>

#endif // SURGSIM_MATH_OCTREESHAPE_H
