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

#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Graphics/OsgRepresentation.h"
#include "SurgSim/Graphics/VectorFieldRepresentation.h"
#include "SurgSim/Graphics/VectorField.h"

#include <osg/Array>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Point>

namespace SurgSim
{
namespace Graphics
{

SURGSIM_STATIC_REGISTRATION(OsgVectorFieldRepresentation);

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

/// OSG vector field representation, implements a VectorFieldRepresenation using OSG.
class OsgVectorFieldRepresentation : public VectorFieldRepresentation, public OsgRepresentation
{
public:
	/// Constructor
	/// \param name Name of OsgVectorFieldRepresentation
	explicit OsgVectorFieldRepresentation(const std::string& name);
	/// Destructor
	~OsgVectorFieldRepresentation();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgVectorFieldRepresentation);

	std::shared_ptr<SurgSim::Graphics::VectorField> getVectorField() const override;

	void setLineWidth(double width) override;
	double getLineWidth() const override;

	void setScale(double scale) override;
	double getScale() const override;

	/// Sets the size of point indicating the starting of vector
	/// \param size	Size of starting point of a vector
	virtual void setPointSize(double size);

	/// Gets the size of starting point of a vector
	/// \return The size of starting point of a vector
	virtual	double getPointSize() const;

	void doUpdate(double dt) override;

	void updateVectorField(const VectorField& vectorfield) override;

private:
	void privateUpdate(const std::vector<DataStructures::Vertex<VectorFieldData>>& vertices);

	/// Vector Field holds a list of vertices/points (X,Y,Z) in 3D space
	/// Each point is associated with a vector and an optional color
	std::shared_ptr<SurgSim::Graphics::VectorField> m_vectorField;

	SurgSim::Framework::LockedContainer<VectorField> m_writeBuffer;

	/// OSG vertex data structure
	osg::ref_ptr<osg::Vec3Array> m_vertexData;

	/// OSG::Geometry node holding OSG representation of vectors
	osg::ref_ptr<osg::Geometry> m_lineGeometry;
	/// OSG::Geometry node holding OSG representation of vector starting points
	osg::ref_ptr<osg::Geometry> m_pointGeometry;

	/// An OSG::DrawArrays object specifying how vectors will be drawn
	osg::ref_ptr<osg::DrawArrays> m_drawArrays;
	/// An OSG::DrawElementUInt object specifying how vector starting points will be drawn
	osg::ref_ptr<osg::DrawElementsUInt> m_drawPoints;

	/// OSG::LineWidth for representing vector
	osg::ref_ptr<osg::LineWidth> m_line;
	/// OSG::Point for representing vector starting point
	osg::ref_ptr<osg::Point> m_point;

	/// OSG::Vec4Array to hold color for each vector
	osg::ref_ptr<osg::Vec4Array> m_colors;

	/// A scale to scale the length of vector
	double m_scale;
};

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGVECTORFIELDREPRESENTATION_H
