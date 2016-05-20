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

#ifndef SURGSIM_GRAPHICS_OSGPOINTCLOUDREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGPOINTCLOUDREPRESENTATION_H

#include <osg/Array>
#include <osg/Geometry>
#include <osg/Point>

#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/PointCloudRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"

namespace SurgSim
{
namespace DataStructures
{
class EmptyData;

template<class Data>
class Vertices;
}

namespace Graphics
{

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif
SURGSIM_STATIC_REGISTRATION(OsgPointCloudRepresentation);

/// Osg point cloud representation, implementation of a PointCloudRepresenation using OSG.
class OsgPointCloudRepresentation : public PointCloudRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	/// \param name The name of the Representation.
	explicit OsgPointCloudRepresentation(const std::string& name);

	/// Destructor
	~OsgPointCloudRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgPointCloudRepresentation);

	std::shared_ptr<PointCloud> getVertices() const override;

	void setPointSize(double val) override;

	double getPointSize() const override;

	void doUpdate(double dt) override;

	void setColor(const SurgSim::Math::Vector4d& color) override;

	SurgSim::Math::Vector4d getColor() const override;

private:

	/// Local pointer to vertices with data
	std::shared_ptr<PointCloud> m_vertices;

	/// OSG vertex data for updating
	osg::ref_ptr<osg::Vec3Array> m_vertexData;

	/// OSG Geometry node holding the data
	osg::ref_ptr<osg::Geometry> m_geometry;

	/// OSG DrawArrays for local operations
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;

	/// OSG::Point for local operations
	osg::ref_ptr<osg::Point> m_point;

	/// Color backing variable
	SurgSim::Math::Vector4d m_color;

	/// Update the geometry
	/// \param vertices new vertices
	void updateGeometry(const DataStructures::VerticesPlain& vertices);
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGPOINTCLOUDREPRESENTATION_H
