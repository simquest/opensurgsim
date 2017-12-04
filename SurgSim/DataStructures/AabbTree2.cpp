#include "AabbTree2.h"

namespace SurgSim {
	namespace Experimental {
		
		void AabbTree::build(std::vector<Math::Aabbd>& aabbs, std::vector<size_t>* indices)
		{
			m_nodes.reserve(indices->size() * 2);
			m_nodes.clear();
			build(aabbs, indices, 0, std::begin(*indices), std::end(*indices));
		}

		void AabbTree::spatialJoin(const AabbTree & other, std::vector<std::pair<size_t, size_t>>* triangles)
		{
			std::vector<std::pair<size_t, size_t>> stack;
			stack.emplace_back(0, 0);

			while (!stack.empty())
			{
				size_t myIndex, otherIndex;
				std::tie(myIndex, otherIndex) = stack.back();
				stack.pop_back();

				if (m_nodes[myIndex].aabb.intersects(other.m_nodes[otherIndex].aabb))
				{
					unsigned char value = (isLeaf(m_nodes[myIndex])) ? 1 : 0;
					value |= (isLeaf(m_nodes[otherIndex])) ? 2 : 0;
					
					switch (value) 
					{
						case 0:
							stack.emplace_back(m_nodes[myIndex].left, other.m_nodes[otherIndex].left);
							stack.emplace_back(m_nodes[myIndex].left, other.m_nodes[otherIndex].right);
							stack.emplace_back(m_nodes[myIndex].right, other.m_nodes[otherIndex].left);
							stack.emplace_back(m_nodes[myIndex].right, other.m_nodes[otherIndex].right);
							break;
						case 1:
							stack.emplace_back(myIndex, other.m_nodes[otherIndex].left);
							stack.emplace_back(myIndex, other.m_nodes[otherIndex].right);
							break;
						case 2:
							stack.emplace_back(m_nodes[myIndex].left, otherIndex);
							stack.emplace_back(m_nodes[myIndex].right, otherIndex);
							break;
						case 3:
							triangles->emplace_back(myIndex, otherIndex);
							break;
						default:
							break;
					}
				}
			}
		}

		size_t AabbTree::build(
			std::vector<Math::Aabbd>& aabb,
			std::vector<size_t>* indices,
			size_t parent,
			iterator_t start,
			iterator_t end)
		{
			switch (end - start)
			{
				case 0:
					SURGSIM_FAILURE() << "Should not have called recursion with empty set of nodes.";
					return -1;
					break;
				case 1:
				{
					AabbTreeNode node;
					node.parent = parent;
					node.left = -1;
					node.triangle = *start;
					node.aabb = aabb[*start];
					m_nodes.emplace_back(std::move(node));
					return m_nodes.size() - 1;
					break;
				}
				default:
				{
					Math::Vector3d min(std::numeric_limits<double>::max(),
						std::numeric_limits<double>::max(),
						std::numeric_limits<double>::max());
					Math::Vector3d max(std::numeric_limits<double>::min(),
						std::numeric_limits<double>::min(), 
						std::numeric_limits<double>::min());

					Math::Aabbd extent;
					for (auto item = start; item < end; ++item)
					{
						extent.extend(aabb[*item]);
					}

					Math::Vector3d mid = extent.max() - extent.min();
					size_t axis;
					mid.maxCoeff(&axis);
					double cutoff = extent.min()[axis] + mid[axis] * 0.5;
					auto midpoint = std::partition(start, end, [axis, cutoff, &aabb](size_t index) {
						return aabb[index].center()(axis) < cutoff;
					});

					//insert non-leaf node ...
					// insert node ... 
					AabbTreeNode node;
					node.parent = parent;
					m_nodes.push_back(node);

					// Need to guard on left or right hand side being empty ... just split in the middle ...
					if (midpoint == start || midpoint == end)
					{
						midpoint = start + (end - start) / 2;
					}

					auto current = m_nodes.size() - 1;
					
					size_t left = build(aabb, indices, current, start, midpoint);
					size_t right = build(aabb, indices, current, midpoint, end);
					m_nodes[current].left = left;
					m_nodes[current].right = right;
					m_nodes[current].aabb = m_nodes[left].aabb.extend(m_nodes[right].aabb);
					return current;
				}
			}
		}

		void AabbTree::update(std::vector<Math::Aabbd>& aabbs)
		{
			// As all child nodes are _after_ their respective parent nodes
			// we can go through the array from the back and update in that order
			std::for_each(std::rbegin(m_nodes), std::rend(m_nodes),
				[this, &aabbs](AabbTreeNode& node) {
					if (isLeaf(node))
					{
						node.aabb = aabbs[node.triangle];
					}
					else
					{
						node.aabb = m_nodes[node.left].aabb.extend(m_nodes[node.right].aabb);
					}
			});
		}

	}
}
