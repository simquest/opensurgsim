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

#ifndef SURGSIM_GRAPHICS_OCTREEREPRESENTATION_H
#define SURGSIM_GRAPHICS_OCTREEREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{
namespace Graphics
{

/// Graphic representation of an Octree
template <class Data>
class OctreeRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param name Name of OctreeRepresentation
	explicit OctreeRepresentation(const std::string& name) : Representation(name)
	{
	}

	/// Destructor
	virtual ~OctreeRepresentation()
	{
	}

	/// Get the Octree
	/// \return	The octree.
	virtual std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> getOctree() const = 0;

	/// Set the Octree
	/// \param The Octree to be used in this representation.
	virtual void setOctree(std::shared_ptr<SurgSim::DataStructures::OctreeNode<Data>> octree) = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OCTREEREPRESENTATION_H