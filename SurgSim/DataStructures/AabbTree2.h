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

#include <SurgSim/Math/Aabb.h>
#include <SurgSim/Math/Vector.h>

#include <vector>

namespace SurgSim {
	
namespace Experimental {

struct AabbTreeNode {
	size_t parent;
	size_t left;
	union {
		size_t right;
		size_t triangle;
	};
	Math::Aabbd aabb;
};

class AabbTree {
public:
	typedef std::vector<size_t>::iterator iterator_t;

	void build(std::vector<Math::Aabbd>& aabbs, std::vector<size_t>* indices);

	void update(std::vector<Math::Aabbd>& aabbs);

	inline bool isLeaf(const AabbTreeNode& node)
	{
		return node.left == -1;
	}

	void spatialJoin(const AabbTree & other, std::vector<std::pair<size_t, size_t>>* triangles);

	void recursiveSpatialJoin(const AabbTree & other, std::vector<std::pair<size_t, size_t>>* triangles, size_t myNode, size_t otherNode);

private:
	std::vector<AabbTreeNode> m_nodes;
	
	size_t build(std::vector<Math::Aabbd>& aabb, std::vector<size_t>* indices, size_t parent, iterator_t start, iterator_t end);

};

}
}