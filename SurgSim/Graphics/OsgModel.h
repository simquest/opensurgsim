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

#ifndef SURGSIM_GRAPHICS_OSGMODEL_H
#define SURGSIM_GRAPHICS_OSGMODEL_H


#include "SurgSim/Framework/Macros.h"
#include "SurgSim/Graphics/Model.h"
#include <osg/ref_ptr>

namespace osg
{
class Node;
}

namespace SurgSim
{
namespace Graphics
{

/// Osg implementation of the Model class, inheriting from Asset, this class knows how to load models that can be
/// handled by osg.
class OsgModel : public Model
{
public:

	/// Constructor
	explicit OsgModel();

	virtual ~OsgModel();

	SURGSIM_CLASSNAME(SurgSim::Graphics::OsgModel);

	/// \return the Node that is the root of the loaded model, nullptr if no model is loaded
	osg::ref_ptr<osg::Node> getOsgNode();

private:

	virtual bool doLoad(const std::string& filePath) override;

	osg::ref_ptr<osg::Node> m_root;
};

}
}

#endif
