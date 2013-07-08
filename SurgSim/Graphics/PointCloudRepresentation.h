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

#ifndef SURGSIM_GRAPHICS_POINTCLOUDREPRESENTATION_H
#define SURGSIM_GRAPHICS_POINTCLOUDREPRESENTATION_H

#include <memory>
#include <SurgSim/DataStructures/Mesh.h>
#include <SurgSim/Graphics/Representation.h>

namespace SurgSim
{
namespace Graphics
{

template <class Data>
class PointCloudRepresentation : public virtual Representation
{
public:

	/// Constructor
	explicit PointCloudRepresentation(const std::string& name) : Representation(name)
	{

	}

	~PointCloudRepresentation()
	{
	};

	virtual void setMesh(std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> mesh) = 0;
	virtual std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> getMesh() const = 0;

private:

};

}; // Graphics
}; // SurgSim

#endif