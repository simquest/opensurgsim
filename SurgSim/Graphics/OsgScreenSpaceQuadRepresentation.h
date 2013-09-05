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

#ifndef SURGSIM_GRAPHICS_OSGSCREENSPACEQUADREPRESENTATION_H
#define SURGSIM_GRAPHICS_OSGSCREENSPACEQUADREPRESENTATION_H

#include <SurgSim/Graphics/OsgRepresentation.h>
#include <SurgSim/Graphics/Representation.h>
#include <SurgSim/Graphics/ScreenSpaceQuadRepresentation.h>
#include <osg/Vec3>

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4250)
#endif

namespace osg
{
class Projection;
class Geode;
class Geometry;
}

namespace SurgSim
{
namespace Graphics
{

class View;
class UniformBase;
class Texture;
class OsgTexture2d;
class OsgTextureRectangle;

class OsgScreenSpaceQuadRepresentation : public OsgRepresentation, public ScreenSpaceQuadRepresentation
{
public:

	/// Constructor
	OsgScreenSpaceQuadRepresentation(const std::string& name, std::shared_ptr<View> view);
	~OsgScreenSpaceQuadRepresentation();

	/// Sets the size for the quad in screen coordinates.
	/// \param	width 	The width of the quad in screen coordinates.
	/// \param	height	The height of the quad in screen coordinates.
	virtual void setSize(int width, int height) override;

	/// Gets the size of the quad.
	/// \param [out]	width 	If non-null, the width.
	/// \param [out]	height	If non-null, the height.
	virtual void getSize(int* width, int* height) const override;

	/// Sets the current pose of the representation
	/// \param	pose	Rigid transformation that describes the current pose of the representation
	virtual void setPose(const SurgSim::Math::RigidTransform3d& pose) override;

	/// Sets a Texture for this quad, this should replace a current texture, this is a convenience function and
	/// this will use the uniform name "diffuseMap" for the uniform in this operation. This can be accomplished
	/// from the outside as well by using the material.
	/// \param	texture	The texture to be set on the quad.
	/// \return	true if it succeeds, false if it fails.
	virtual bool setTexture(std::shared_ptr<Texture> texture) override;

	/// Sets a Texture2d for this quad, this should replace a current texture, this is a convenience function and
	/// this will use the uniform name "diffuseMap" for the uniform in this operation. This can be accomplished
	/// from the outside as well by using the material.
	/// \param	texture	The texture to be set on the quad.
	/// \return	true if it succeeds, false if it fails.
	bool setTexture(std::shared_ptr<OsgTexture2d> texture);

	/// Sets a rectangular texture for this quad, this should replace a current texture,
	/// this is a convenience function and will use the uniform name "diffuseMap" for the uniform in this operation.
	/// \param	texture	The texture to be set on the quad.
	/// \return	true if it succeeds, false if it fails.
	bool setTexture(std::shared_ptr<OsgTextureRectangle> texture);

private:

	/// The view that is being used
	std::shared_ptr<View> m_view;

	/// Local geode to contain geometry
	osg::ref_ptr<osg::Geode> m_geode;

	/// Local geometry pointer
	osg::ref_ptr<osg::Geometry> m_geometry;

	/// Projection matrix, needs to be updated when the view is changed
	osg::ref_ptr<osg::Projection> m_projection;

	/// Size of the quad
	osg::Vec3 m_scale;

	///@{
	/// Cached view extensions
	int m_displayWidth;
	int m_displayHeight;
	///@}

	/// Overridden from Representation, execute local updates
	virtual void doUpdate(double dt) override;

	/// Sets texture coordinates for the quad.
	/// \param	left, bottom, right, top	The extents for the texture coordinates for the corners of the quad.
	void setTextureCoordinates(float left, float bottom, float right, float top);

	/// Replace a uniform in the material, will create the material if necessary
	/// \param	name	  	The name of the uniform to replace.
	/// \param	newUniform	The new uniform.
	/// \return	true if it succeeds, false if it fails.
	bool replaceUniform(const std::string& name, std::shared_ptr<SurgSim::Graphics::UniformBase> newUniform);
};

}; // Graphics
}; // SurgSim

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif