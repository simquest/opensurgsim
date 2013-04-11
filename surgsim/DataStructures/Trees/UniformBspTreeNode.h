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

#ifndef SURGSIM_UNIFORM_BSP_TREE_NODE_H
#define SURGSIM_UNIFORM_BSP_TREE_NODE_H

#include "BspTreeNode.h"

#include <Assert.h>
#include <Eigen/Core>

namespace SurgSim
{

template <unsigned int D, typename P>
class UniformBspTreeNode : public BspTreeNode<D>
{
public:
	typedef unsigned char Layer;
	typedef unsigned int LocationCode;
	struct Location
	{
		LocationCode codes[D];

		bool operator==(const Location& location) const
		{
			for (unsigned int i = 0; i < D; ++i)
			{
				if (codes[i] != location.codes[i])
				{
					return false;
				}
			}

			return true;
		}
	};

	UniformBspTreeNode() : BspTreeNode<D>(), m_location(Location()), m_layer(0)
	{
		BspTreeNode<D>::setNumChildren(Eigen::internal::pow<unsigned int>(2, D));
	}
	virtual ~UniformBspTreeNode()
	{
	}

	virtual void setLocation(Location location)
	{
		m_location = location;
	}
	Location getLocation() const
	{
		return m_location;
	}

	virtual void setLayer(Layer layer)
	{
		m_layer = layer;
	}
	Layer getLayer() const
	{
		return m_layer;
	}

	virtual void setNumChildren(unsigned int numChildren)
	{
		Framework::SURGSIM_FAILURE() << "Number of children is fixed at " << getNumChildren() << ".";
	}
	virtual void addChild(std::shared_ptr<TreeNode>& node)
	{
		Framework::SURGSIM_FAILURE() << "Number of children is fixed at " << getNumChildren() << 
			", use setChild(unsigned int, std::shared_ptr<AbstractTreeNode>) instead.";
	}

protected:
	virtual bool isEqual(const TreeNode& node) const
	{
		const UniformBspTreeNode& uniformBspTreeNode = static_cast<const UniformBspTreeNode&>(node);
		return BspTreeNode<D, P>::isEqual(node) && m_layer == uniformBspTreeNode.m_layer && 
			m_location == uniformBspTreeNode.m_location;
	}

private:
	Location m_location;
	Layer m_layer;

};

}; // namespace SurgSim

#endif  // UNIFORM_BSP_TREE_NODE_H
