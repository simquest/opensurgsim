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

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/VectorFieldRepresentation.h>

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

/// OSG vector field representation, implements a VectorFieldRepresenation using OSG.
class OsgVectorFieldRepresentation : public VectorFieldRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	explicit OsgVectorFieldRepresentation(const std::string& name);
	/// Destructor
	~OsgVectorFieldRepresentation();

	/// Sets vector field
	/// \param	vertices A Vertices (data structure) contains a (mathematical) vector and an optional color
	virtual void setVertices(std::shared_ptr< SurgSim::Graphics::VectorField > vertices) override;
	/// Gets the vector field 
	/// \return	A Vertices (data structure) contains a (mathematical) vector and an optional color information
	virtual std::shared_ptr< SurgSim::Graphics::VectorField > getVertices() const override;

	/// Sets vector line width
	/// \param	val	Width of vector line
	virtual void setLineWidth(double width) override;
	/// Gets line width
	/// \return	The line width
	virtual double getLineWidth() const override;

	/// Sets a scale to scale vector
	/// \param scale The scale
	void setScale(double scale);
	/// Gets the scale used to scale vector
	/// \return The scale
	double getScale() const;

	/// Executes the update operation
	/// \param	dt	The time difference
	virtual void doUpdate(double dt) override;

private:
	/// Vertices (data structure) holding a list of vertices/points in 3D space
	/// Each point is associated with a vector and an optional color
	std::shared_ptr< SurgSim::Graphics::VectorField > m_vertices;

	/// OSG vertex data structure
	osg::ref_ptr<osg::Vec3Array> m_vertexData;

	/// OSG::Geometry node holding OSG vertex
	osg::ref_ptr<osg::Geometry> m_lineGeometry;

	/// OSG::DrawArrays specifying how vertices wiil be drawn
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;

	/// OSG::LineWidth for representing vector
	osg::ref_ptr<osg::LineWidth> m_line;

	/// A scale to scale the length of vector
	double m_scale;
	
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim


#endif // SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_H