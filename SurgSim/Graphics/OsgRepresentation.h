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

#ifndef SURGSIM_GRAPHICS_OSGREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGREPRESENTATION_H

#include <memory>

#include <osg/ref_ptr>

#include "SurgSim/Graphics/Representation.h"
#include "SurgSim/Graphics/OsgUniform.h"

namespace osg
{
class Group;
class Node;
class PositionAttitudeTransform;
class Switch;
}

namespace SurgSim
{
namespace Graphics
{

class OsgMaterial;
class TangentSpaceGenerator;
class UniformBase;

/// OSS default value for opengl values that have to be assigned a fixed number
///@{
static const int TANGENT_VERTEX_ATTRIBUTE_ID = 6;
static const int BITANGENT_VERTEX_ATTRIBUTE_ID = 7;
static const int DIFFUSE_TEXTURE_UNIT = 0;
static const int NORMAL_TEXTURE_UNIT = 1;
static const int SHADOW_TEXTURE_UNIT = 8;
///@}

/// Base OSG implementation of a graphics representation.
///
/// Wraps an osg::Node which serves as the root for the representation's portion of the scene graph.
class OsgRepresentation : public virtual Representation
{
public:
	/// Constructor
	explicit OsgRepresentation(const std::string& name);
	/// Destructor
	virtual ~OsgRepresentation();

	/// Returns the root OSG Node for this representations portion of the scene graph
	osg::ref_ptr<osg::Node> getOsgNode() const;

	/// Sets the material that defines the visual appearance of the representation
	/// \param	material	Graphics material
	/// \return	True if set successfully, otherwise false
	/// \note	OsgPlaneRepresentation only accepts subclasses of OsgMaterial.
	bool setMaterial(std::shared_ptr<Material> material) override;

	/// Gets the material that defines the visual appearance of the representation
	/// \return	Graphics material
	std::shared_ptr<Material> getMaterial() const override;

	/// Removes the material from the representation
	void clearMaterial() override;

	void setDrawAsWireFrame(bool val) override;

	bool getDrawAsWireFrame() const override;

	/// Enable the generation of tangents
	/// When enabled it is up to the subclasses responsibility to react to changes and trigger the regeneration of
	/// Tangents. Tangents will be stored for every geometry node that contains a vertex, normal and texture array.
	/// \note the \sa TangentSpaceGenerator is used to create the appropriate vertex attribute arrays.
	/// These are stored as vertex attribute arrays at the indices indicated by TANGENT_ARRAY_ATTRIBUTE_ID,
	/// and BITANGENT_ARRAY_ATTRIBUTE_ID with the format osg::ArrayVec4.
	/// The tangents will be made orthonormal by default.
	/// tangents via \sa updateTangents()
	void setGenerateTangents(bool value) override;

	bool isGeneratingTangents() const override;

	/// Updates the representation.
	/// \param	dt	The time in seconds of the preceding timestep.
	void update(double dt) override;

	void addUniform(std::shared_ptr<SurgSim::Graphics::UniformBase> uniform);

protected:
	virtual void doUpdate(double dt);

	/// Set the visibility of this representation
	/// \param val The visibility
	void setVisible(bool val);

	/// Causes the tangents to be recalculated if tangent generation is enabled
	/// Subclasses should call this whenever the state changes in a way that need tangents to be recalculated
	void updateTangents();

	/// Switch used to toggle the visibility of the representation
	osg::ref_ptr<osg::Switch> m_switch;

	/// Transform used to pose the representation
	osg::ref_ptr<osg::PositionAttitudeTransform> m_transform;

	/// Holder for attributes coming from OsgMaterial
	osg::ref_ptr<osg::Group> m_materialProxy;

	/// Material defining the visual appearance of the representation
	std::shared_ptr<OsgMaterial> m_material;

	/// Visitor to generate tangents
	osg::ref_ptr<TangentSpaceGenerator> m_tangentGenerator;

	/// Indicates if the representation is rendered as a wireframe.
	bool m_drawAsWireFrame;

	std::shared_ptr<OsgUniform<SurgSim::Math::Matrix44f>> m_modelMatrixUniform;

};

}; // Graphics
}; // SurgSim

#endif // SURGSIM_GRAPHICS_OSGREPRESENTATION_H
