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

#ifndef SURGSIM_GRAPHICS_OSGPOINTCLOUDREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGPOINTCLOUDREPRESENTATION_H

#include <SurgSim/Graphics/PointCloudRepresentation.h>
#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/DataStructures/Mesh.h>

#include <osg/PrimitiveSet>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/Point>

namespace SurgSim
{
namespace Graphics
{

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

/// Osg point cloud representation, implementation of a PointCloudRepresenation using OSG.
template <class Data>
class OsgPointCloudRepresentation : public PointCloudRepresentation<Data>, public OsgRepresentation
{
public:

	/// Constructor
	explicit OsgPointCloudRepresentation(const std::string& name);

	/// Destructor
	~OsgPointCloudRepresentation();

	/// Sets a mesh.
	/// \param	mesh	The mesh.
	virtual void setMesh(std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> mesh) override;

	/// Gets the mesh.
	/// \return	The mesh.
	virtual std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> getMesh() const override;

	/// Sets point size.
	/// \param	val	The value.
	virtual void setPointSize(double val) override;

	/// Gets point size.
	/// \return	The point size.
	virtual double getPointSize() const override;

	/// Executes the update operation.
	/// \param	dt	The dt.
	virtual void doUpdate(double dt) override;

	/// Sets a color.
	/// \param	color	The color.
	virtual void setColor(const SurgSim::Math::Vector4d& color) override;

private:

	/// Local pointer to mesh with data
	std::shared_ptr<SurgSim::DataStructures::Mesh<Data>> m_mesh;

	/// OSG vertex data for updating
	osg::ref_ptr<osg::Vec3Array> m_vertexData;

	/// OSG Geometry node holding the data
	osg::ref_ptr<osg::Geometry> m_geometry;

	/// OSG DrawArrays for local operations
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;

	/// OSG::Point for local operations
	osg::ref_ptr<osg::Point> m_point;

};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#include <SurgSim/Graphics/OsgPointCloudRepresentation-inl.h>

#endif