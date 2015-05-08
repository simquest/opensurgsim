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

#include "SurgSim/Devices/Oculus/OculusView.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Device
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Device::OculusView, OculusView);

OculusView::OculusView(const std::string& name) : OsgView(name)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OculusView, std::shared_ptr<SurgSim::Framework::Component>, InputComponent, 
									  getInputComponent, setInputComponent);
}

OculusView::~OculusView()
{
}

bool OculusView::doWakeUp()
{
	OsgView::doWakeUp();
	osg::ref_ptr<SurgSim::Device::OculusDisplaySettings> displaySeetings =
		new SurgSim::Device::OculusDisplaySettings(getOsgView()->getDisplaySettings());
	
	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);
	SurgSim::DataStructures::DataGroup::DynamicMatrixType projectionMatrix;

	SURGSIM_ASSERT(
		dataGroup.matrices().get(SurgSim::DataStructures::Names::LEFT_PROJECTION_MATRIX, &projectionMatrix)) <<
		"No left projection matrix can be retrieved for device: " << m_inputComponent->getDeviceName();
	displaySeetings->setLeftEyeProjectionMatrix(projectionMatrix.block<4,4>(0, 0));

	SURGSIM_ASSERT(dataGroup.matrices().get(
		SurgSim::DataStructures::Names::RIGHT_PROJECTION_MATRIX, &projectionMatrix)) <<
		"No right projection matrix can be retrieved for device: " << m_inputComponent->getDeviceName();
	displaySeetings->setLeftEyeProjectionMatrix(projectionMatrix.block<4,4>(0, 0));

	getOsgView()->setDisplaySettings(displaySeetings);

	return true;
}

void OculusView::setInputComponent(std::shared_ptr<Framework::Component> input)
{
	m_inputComponent = Framework::checkAndConvert<Input::InputComponent>(input, "SurgSim::Input::InputComponent");
}

std::shared_ptr<Input::InputComponent> OculusView::getInputComponent() const
{
	return m_inputComponent;
}

}; // namespace Device
}; // namespace SurgSim