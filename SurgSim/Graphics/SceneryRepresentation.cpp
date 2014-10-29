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

#include "SurgSim/Graphics/SceneryRepresentation.h"

#include "SurgSim/Framework/Asset.h"
#include "SurgSim/Graphics/Model.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{
namespace Graphics
{


SceneryRepresentation::SceneryRepresentation(const std::string& name) : Representation(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(SceneryRepresentation, std::shared_ptr<SurgSim::Framework::Asset>,
									  Model , getModel, setModel);


	// Enables the alternative use of the model file instead of the actual mesh object
	DecoderType decoder = std::bind(&SceneryRepresentation::loadModel,
									this,
									std::bind(&YAML::Node::as<std::string>, std::placeholders::_1));
	setDecoder("ModelFileName", decoder);

	SetterType setter = std::bind(&SceneryRepresentation::loadModel,
								  this,
								  std::bind(SurgSim::Framework::convert<std::string>, std::placeholders::_1));

	setSetter("ModelFileName", setter);
}


}
}

