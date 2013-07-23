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

#include <SurgSim/Graphics/Representation.h>

namespace SurgSim
{
namespace Graphics
{

class View;

/// A quad to display on the screen in screen space coordinates, use setPose() to set the position but 
/// x,y are presumed to be in screen space with 0|0 being in the lower left corner
class ScreenSpaceQuadRepresentation : public virtual Representation
{
public:

	/// Constructor.
	/// \param	name	The name.
	/// \param	view	The view that will contain this quad.
	ScreenSpaceQuadRepresentation(const std::string name, std::shared_ptr<View> view) : Representation(name)
	{

	}

	~ScreenSpaceQuadRepresentation()
	{

	}

	/// Sets the size for the quad in screen coordinates.
	/// \param	width 	The width of the quad in screen coordinates.
	/// \param	height	The height of the quad in screen coordinates.
	virtual void setSize(int width, int height) = 0;

	/// Gets the size of the quad.
	/// \param [in,out]	width 	If non-null, the width.
	/// \param [in,out]	height	If non-null, the height.
	virtual void getSize(int* width, int* height) const = 0;

};

}; // Graphics
}; // SurgSim

#endif