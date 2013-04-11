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

#ifndef SURGSIM_UNIFORM_BSP_TREE_H
#define SURGSIM_UNIFORM_BSP_TREE_H

#include "BspTree.h"

#include "UniformBspTreeNode.h"

#include "SurgSim/Math/Vector.h"

namespace SurgSim
{

template <unsigned int D, typename P>
class UniformBspTree : public BspTree<D>
{
public:
	UniformBspTree() : BspTree<D>()
	{
	}
	virtual ~UniformBspTree()
	{
	}

	virtual void setDimensions(const Eigen::Matrix<P, D, 1, Eigen::DontAlign>& dimensions)
	{
		m_dimensions = dimensions;
	}
	const Eigen::Matrix<P, D, 1, Eigen::DontAlign>& getDimensions() const
	{
		return m_dimensions;
	}

	UniformBspTreeNode<D, P>* findPosition(const Eigen::Matrix<P, D, 1, Eigen::DontAlign>& position, typename UniformBspTreeNode<D, P>::Layer layer) const;

protected:
	virtual bool isEqual(const Tree& tree) const
	{
		const UniformBspTree& uniformBspTree = static_cast<const UniformBspTree&>(node);
		return BspTree<D, P>::isEqual(tree) && m_dimensions == uniformBspTree.m_dimensions;
	}

private:
	Eigen::Matrix<P, D, 1, Eigen::DontAlign> m_dimensions;

};

};  // namespace SurgSim

#endif  // UNIFORM_BSP_TREE_H
