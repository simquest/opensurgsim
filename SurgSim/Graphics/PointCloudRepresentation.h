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
#include <SurgSim/Math/Vector.h>

namespace SurgSim
{
namespace Graphics
{
/// Graphic representation of a point cloud, hase a very basic interface and is intentionally kept generic. It takes
/// a Mesh<Data> as a template parameter, this will probably have to be subclassed for different kinds of data, e.g.
/// particles, depth data, etc ... .
/// \tparam	Data Type of the data.
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

	/// Sets the mesh for the point cloud.
	/// \param	mesh	The mesh.
	virtual void setMesh(std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> mesh) = 0;

	/// Pull the mesh.
	/// \return	The mesh.
	virtual std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> getMesh() const = 0;

	/// Sets point size for the point elements.
	/// \param	val	The value.
	virtual void setPointSize(double val) = 0;

	/// Gets point size.
	/// \return	The point size.
	virtual double getPointSize() const = 0;

	/// Sets a color for all of the points together.
	/// \param	color	The color.
	virtual void setColor(const SurgSim::Math::Vector4d& color) = 0;

	/// Gets the color.
	/// \return The current color.
	virtual SurgSim::Math::Vector4d getColor() const = 0;

private:

};

}; // Graphics
}; // SurgSim

#endif