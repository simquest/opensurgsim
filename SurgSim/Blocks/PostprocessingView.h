// This file is a part of the OpenSurgSim project.
// Copyright 2013-2017, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_TONEMAPPING_H
#define SURGSIM_BLOCKS_TONEMAPPING_H

#include "SurgSim/Framework/BasicSceneElement.h"

#include <string>
#include <memory>

namespace SurgSim
{

namespace Graphics
{
class OsgCamera;
class OsgView;
class OsgMaterial;
}

namespace Blocks
{

class PostprocessingView : public Framework::SceneElement
{
public:
	explicit PostprocessingView(const std::string& name);

	void enableManipulator(bool val);

private:
	/// Camera to take the default content and render it i.e the main source of the post processing pipeline
	std::shared_ptr<Graphics::OsgCamera> m_defaultCamera;

	/// Camera to take the last texture and put it to the screen, i.e. the last sink of the post processing pipeline
	std::shared_ptr<Graphics::OsgCamera> m_toScreenCamera;

	std::shared_ptr<Graphics::OsgView> m_view;

	std::shared_ptr<Graphics::OsgMaterial> m_material;

protected:
	virtual bool doInitialize() override;

};


}
}

#endif
