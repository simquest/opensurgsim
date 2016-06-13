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

#ifndef SURGSIM_GRAPHICS_OSGCURVEREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGCURVEREPRESENTATION_H

#include "SurgSim/Graphics/CurveRepresentation.h"
#include "SurgSim/Graphics/OsgRepresentation.h"

#include <osg/Array>
#include <osg/ref_ptr>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Geometry;
class DrawArrays;
}

namespace SurgSim
{

namespace Graphics
{

SURGSIM_STATIC_REGISTRATION(OsgCurveRepresentation)

/// Implements the CurveRepresentation for OpenSceneGraph, it uses Catmull Rom interpolation, to draw the line
/// as a GL_LINESTRIP. use the material_curve.vert shader for rendering. This class will also deposit the information
/// of the segment in the normal information for the vertex.
class OsgCurveRepresentation : public OsgRepresentation, public CurveRepresentation
{
public:
	/// Constructor
	/// \param name Name of the representation
	explicit OsgCurveRepresentation(const std::string& name);

	~OsgCurveRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgCurveRepresentation);

	bool doInitialize() override;

	bool doWakeUp() override;

	void doUpdate(double dt) override;

	void setSubdivisions(size_t num) override;

	size_t getSubdivisions() const override;

	void setTension(double tension) override;

	double getTension() const override;

	void setColor(const SurgSim::Math::Vector4d& color) override;

	Math::Vector4d getColor() const override;

	void setWidth(double width) override;

	double getWidth() const override;

	void setAntiAliasing(bool val) override;

	bool isAntiAliasing() const override;

private:

	/// Update the OSG structure with the information of the control points
	/// \param controlPoints to use
	void updateGraphics(const DataStructures::VerticesPlain& controlPoints);

	///@{
	/// OSG handles for updating
	osg::ref_ptr<osg::Geometry> m_geometry;
	osg::ref_ptr<osg::Vec3Array> m_vertexData;
	osg::ref_ptr<osg::Vec3Array> m_normalData;
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;
	///@}

	/// @{
	/// Members for the CurveRepresentation properties
	Math::Vector4d m_color;
	size_t m_subdivision;
	double m_tension;
	double m_width;
	///@}

	///@{
	/// Local structures to keep allocations to a minimum
	std::vector<Math::Vector3d> m_controlPoints;
	std::vector<Math::Vector3d> m_vertices;
	///@}

};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}
}

#endif
