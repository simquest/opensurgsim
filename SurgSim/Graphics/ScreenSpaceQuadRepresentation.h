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

#ifndef SURGSIM_GRAPHICS_SCREENSPACEQUADREPRESENTATION_H
#define SURGSIM_GRAPHICS_SCREENSPACEQUADREPRESENTATION_H

#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{
namespace Graphics
{

class View;
class Texture;

/// A quad to display on the screen in screen space coordinates, use setPose() to set the position but
/// x,y are presumed to be in screen space with 0|0 being in the lower left corner
class ScreenSpaceQuadRepresentation : public virtual Representation
{
public:

	/// Constructor.
	/// \param	name	The name.
	explicit ScreenSpaceQuadRepresentation(const std::string name) : Representation(name)
	{

	}

	~ScreenSpaceQuadRepresentation()
	{

	}

	/// Sets the location in screen space.
	/// \param	x,y	The x and y coordinates.
	virtual void setLocation(double x, double y) = 0;

	/// Gets the location in screen space.
	/// \param [out]	x,y	If non-null the x and y coordinates, may throw if null is passed.
	virtual void getLocation(double* x, double* y) = 0;

	/// Sets the size for the quad in screen coordinates.
	/// \param	width 	The width of the quad in screen coordinates.
	/// \param	height	The height of the quad in screen coordinates.
	virtual void setSize(double width, double height) = 0;

	/// Gets the size of the quad.
	/// \param [out]	width 	If non-null, the width, may throw if null is passed.
	/// \param [out]	height	If non-null, the height, may throw if null is passed.
	virtual void getSize(double* width, double* height) const = 0;

	/// Sets a Texture for this quad, this should replace a current texture, this is a convenience function and
	/// this will use the uniform name "diffuseMap" for the uniform in this operation. This can be accomplished
	/// from the outside as well by using the material.
	/// \param	texture	The texture to be set on the quad.
	/// \return	true if it succeeds, false if it fails.
	virtual bool setTexture(std::shared_ptr<Texture> texture) = 0;

	/// \return the pose of the quad, this disregards the SceneElement pose
	SurgSim::Math::RigidTransform3d getPose() const override
	{
		return getLocalPose();
	}

};

}; // Graphics
}; // SurgSim

#endif