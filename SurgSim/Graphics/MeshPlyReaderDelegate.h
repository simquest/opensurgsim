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

#ifndef SURGSIM_GRAPHICS_MESHPLYREADERDELEGATE_H
#define SURGSIM_GRAPHICS_MESHPLYREADERDELEGATE_H

#include <array>
#include <memory>

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/TriangleMeshPlyReaderDelegate.h"
#include "SurgSim/Graphics/Mesh.h"

namespace SurgSim
{
namespace Graphics
{

/// Implementation of PlyReaderDelegate for graphicsmeshes
class MeshPlyReaderDelegate : public SurgSim::DataStructures::TriangleMeshPlyReaderDelegate<Mesh>
{
public:

	/// Default constructor.
	MeshPlyReaderDelegate();

	/// Constructor.
	/// \param mesh The mesh to be used, it will be cleared by the constructor.
	explicit MeshPlyReaderDelegate(std::shared_ptr<MeshType> mesh);

	void processVertex(const std::string& elementName) override;

};

}
}

#endif
