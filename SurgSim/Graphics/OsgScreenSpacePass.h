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

#ifndef SURGSIM_GRAPHICS_OSGSCREENSPACEPASS_H
#define SURGSIM_GRAPHICS_OSGSCREENSPACEPASS_H

#include <string>

#include "SurgSim/Graphics/RenderPass.h"

#include <osg/ref_ptr>

namespace osg
{
class Camera;
}

namespace SurgSim
{
namespace Graphics
{

/// Special RenderPass to draw items using a orthogonal projection, this is specific to the
/// Osg implementation of the SurgSim rendering
class OsgScreenSpacePass : public RenderPass
{
public:

	/// Constructor.
	/// \param name The name of the component
	explicit OsgScreenSpacePass(const std::string& name);

	/// Destructor
	virtual ~OsgScreenSpacePass();

	/// Set viewport dimensions
	void setViewPort(int width, int height);

	/// Initialize this Component
	bool doInitialize() override;


private:

	/// Update the projection matrix
	void updateViewport(int width, int height);

	/// The osg camera reference
	osg::ref_ptr<osg::Camera> m_camera;

	/// The width of the viewport
	int m_width;

	/// The height of the viewport
	int m_height;
};

}
}

#endif // SURGSIM_GRAPHICS_OSGSCREENSPACEPASS_H
