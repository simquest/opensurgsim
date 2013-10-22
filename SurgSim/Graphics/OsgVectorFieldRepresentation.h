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

#ifndef SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_H

#include <SurgSim/Graphics/VectorFieldRepresentation.h>
#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/DataStructures/Vertices.h>

#include <osg/PrimitiveSet>
#include <osg/Geometry>
#include <osg/Array>
#include <osg/LineWidth>

namespace SurgSim
{
namespace Graphics
{

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif


using SurgSim::DataStructures::Vertices;
using SurgSim::Math::Vector4d;

/// Osg vector field representation, implementation of a VectorFieldRepresenation using OSG.
template <class Data>
class OsgVectorFieldRepresentation : public VectorFieldRepresentation<Data>, public OsgRepresentation
{
public:

	/// Constructor
	explicit OsgVectorFieldRepresentation(const std::string& name);

	/// Destructor
	~OsgVectorFieldRepresentation();

	/// Sets Vertices.
	/// \param	mesh	The mesh.
	virtual void setVertices(std::shared_ptr<Vertices<Data>> mesh) override;

	/// Gets the vertices.
	/// \return	The vertices.
	virtual std::shared_ptr<Vertices<Data>> getVertices() const override;

	/// Sets line width.
	/// \param	val	Width of line.
	virtual void setLineWidth(double width) override;

	/// Gets line width.
	/// \return	The line width.
	virtual double getLineWidth() const override;

	/// Executes the update operation.
	/// \param	dt	The dt.
	virtual void doUpdate(double dt) override;

	/// Sets a color.
	/// \param	color	The color.
	virtual void setColors(const std::vector<Vector4d>& color) override;

	/// Gets the color.
	/// \return The current color.
	virtual std::vector<Vector4d> getColors() const override;

private:

	/// Local pointer to vertices with data
	std::shared_ptr<Vertices<Data>> m_vertices;

	/// OSG vertex data for updating
	osg::ref_ptr<osg::Vec3Array> m_vertexData;

	/// OSG Geometry node holding the data
	osg::ref_ptr<osg::Geometry> m_geometry;

	/// OSG DrawArrays for local operations
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;

	/// OSG::LineWidth for local operations
	osg::ref_ptr<osg::LineWidth> m_line;

	/// A vector of colors for each vector in vector field
	std::vector<Vector4d> m_colors;

};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#include <SurgSim/Graphics/OsgVectorFieldRepresentation-inl.h>

#endif // SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_H