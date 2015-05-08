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

#ifndef SURGSIM_DEVICE_OCULUS_OCULUSVIEW_H
#define SURGSIM_DEVICE_OCULUS_OCULUSVIEW_H

#include "SurgSim/Graphics/OsgView.h"

namespace SurgSim
{

namespace Framework
{
class Component;
}

namespace Input
{
class InputComponent;
}

namespace Device
{

SURGSIM_STATIC_REGISTRATION(OculusView);

/// OculusView is a customization of SurgSim::Graphics::OsgView with projection matrices pulled from the Oculus device.
class OculusView : public SurgSim::Graphics::OsgView
{
public:
	/// Constructor
	/// \param	name	Name of the view
	explicit OculusView(const std::string& name);

	/// Destructor
	~OculusView();

	SURGSIM_CLASSNAME(SurgSim::Device::OculusView);

	/// Set the InputComponent this view connects to.
	/// \param input The InputComponent
	void setInputComponent(std::shared_ptr<Framework::Component> input);

	/// \return The InputComponnet this view connects
	std::shared_ptr<Input::InputComponent> getInputComponent() const;
protected:
	/// Wake up the view
	bool doWakeUp() override;

private:
	std::shared_ptr<Input::InputComponent> m_inputComponent;
};

};  // namespace Graphics
};  // namespace SurgSim

#endif  // SURGSIM_DEVICE_OCULUS_OCULUSVIEW_H
