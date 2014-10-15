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

#ifndef SURGSIM_GRAPHICS_OSGUNIFORMFACTORY_H
#define SURGSIM_GRAPHICS_OSGUNIFORMFACTORY_H

#include <string>

#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Graphics/UniformBase.h"

namespace SurgSim
{
namespace Graphics
{

/// This class can create the appropriate OsgUniform from an OpenGl glsl type, use the appropriate name
/// from glsl in the create() function to recieve the correctly typed uniform
class OsgUniformFactory : public SurgSim::Framework::ObjectFactory1<UniformBase, std::string>
{
public:
	OsgUniformFactory();

	virtual ~OsgUniformFactory();
};

}
}

#endif
