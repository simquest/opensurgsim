// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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

#ifndef SURGSIMEDITDEBUG_DRAWFUNCTIONS_H
#define SURGSIMEDITDEBUG_DRAWFUNCTIONS_H

#include <string>

#include "boost/any.hpp"

#include "SurgSim/Math/RigidTransform.h"

namespace SurgSim {
namespace Framework {
	class Scene;
	class Runtime;
	class SceneElement;
	class Component;
}
}

namespace SurgSim {
namespace EditDebug {
	/// Draw the whole scene, draws the left hand side with the list of components and calls 
	/// showComponentEditor to draw the right hand side
	/// \param runtime The runtime that is being debugged
	/// \param width The width of the current simulation window
	/// \param height The height of the current simulation window
	void showSceneEditor(Framework::Runtime* scene, int width, int height);
	
	/// Draw the editor for a single SceneElement, show all the components of that SceneElement
	/// \param element The element that should be shown
	void showElementEditor(const Framework::SceneElement* element);

	/// For a given component component show a list of properties  
	/// \param component The component to be shown
	void showComponentEditor(Framework::Component* component);
}
}
#endif