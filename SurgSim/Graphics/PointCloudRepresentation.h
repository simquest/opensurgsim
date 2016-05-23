// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/DataStructures/EmptyData.h"
#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Math/MathConvert.h"
#include "SurgSim/Math/Vector.h"

namespace SurgSim
{
namespace Graphics
{

typedef SurgSim::DataStructures::Vertices<SurgSim::DataStructures::EmptyData> PointCloud;

/// Graphic representation of a point cloud, hase a very basic interface and is intentionally kept generic.
class PointCloudRepresentation : public virtual Representation
{
public:

	/// Constructor
	explicit PointCloudRepresentation(const std::string& name);

	virtual ~PointCloudRepresentation();

	/// Pull the vertices.
	/// \return	The mesh.
	virtual std::shared_ptr<PointCloud>	getVertices() const = 0;

	/// Sets point size for the point elements.
	/// The point size defines the the width and height of the drawn square, in window pixels.
	/// \param	val	The value.
	virtual void setPointSize(double val) = 0;

	/// Gets point size.
	/// The point size defines the the width and height of the drawn square, in window pixels.
	/// \return	The point size.
	virtual double getPointSize() const = 0;

	/// Sets a color for all of the points together.
	/// \param	color	The color.
	virtual void setColor(const SurgSim::Math::Vector4d& color) = 0;

	/// Gets the color.
	/// \return The current color.
	virtual SurgSim::Math::Vector4d getColor() const = 0;

	void updateVertices(const DataStructures::VerticesPlain& vertices);

	void updateVertices(DataStructures::VerticesPlain&& vertices);

protected:

	Framework::LockedContainer<DataStructures::VerticesPlain> m_locker;
};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_POINTCLOUDREPRESENTATION_H
