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

#include <string>

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/OctreeShape.h"

namespace SurgSim
{
namespace Graphics
{

/// Graphic representation of an Octree
class OctreeRepresentation : public virtual Representation
{
public:
	/// Constructor
	/// \param name Name of OctreeRepresentation
	explicit OctreeRepresentation(const std::string& name) : Representation(name)
	{
		SURGSIM_ADD_SERIALIZABLE_PROPERTY(OctreeRepresentation, std::shared_ptr<SurgSim::Math::Shape>, OctreeShape,
										  getOctreeShape, setOctreeShape);
	}

	/// Destructor
	virtual ~OctreeRepresentation()
	{
	}

	/// Set the OctreeShape of this representation. The Octree is retrieved and visualized.
	/// \param shape The OctreeShape from which the octree is retrieved and visualized.
	virtual void setOctreeShape(const std::shared_ptr<SurgSim::Math::Shape>& shape) = 0;

	/// \return The OctreeShape from which the Octree is retrieved.
	virtual std::shared_ptr<SurgSim::Math::OctreeShape> getOctreeShape() const = 0;

	/// Mark the OctreeNode visible/invisible in the given a OctreePath (typedef-ed in OctreeNode.h).
	/// \param path An OctreePath, giving the path leads to the OctreeNode whose visibility to be changed.
	/// \param visibility Whether or not the OctreeNode specified by 'path' is visible or not.
	virtual	void setNodeVisible(const SurgSim::DataStructures::OctreePath& path, bool visibility) = 0;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OCTREEREPRESENTATION_H