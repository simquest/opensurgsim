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

	// Default Settings of Oculus DK2
	setFullScreen(true);
	setDisplayType(View::DISPLAY_TYPE_HMD);
	setStereoMode(View::STEREO_MODE_HORIZONTAL_SPLIT);
	setScreenWidth(0.0631);
	setScreenHeight(0.071);
	setEyeSeparation(0.06);
	setScreenDistance(0.10);
	setTargetScreen(1); // Assume Oculus screen has ID 1

	std::array<int, 2> dimensions = {1920, 1080};
	setDimensions(dimensions);
}

OculusView::~OculusView()
{
}

void OculusView::setInputComponent(std::shared_ptr<Framework::Component> input)
{
	m_inputComponent = Framework::checkAndConvert<Input::InputComponent>(input, "SurgSim::Input::InputComponent");
}

std::shared_ptr<Input::InputComponent> OculusView::getInputComponent() const
{
	return m_inputComponent;
}

osg::ref_ptr<osg::DisplaySettings> OculusView::createDisplaySettings() const
{
	SURGSIM_ASSERT(m_inputComponent != nullptr) << "No InputComponent is connected to this view.";

	osg::ref_ptr<SurgSim::Device::OculusDisplaySettings> displaySettings =
		new SurgSim::Device::OculusDisplaySettings(OsgView::createDisplaySettings());

	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);
	SurgSim::DataStructures::DataGroup::DynamicMatrixType projectionMatrix;

	SURGSIM_ASSERT(
		dataGroup.matrices().get(SurgSim::DataStructures::Names::LEFT_PROJECTION_MATRIX, &projectionMatrix)) <<
		"No left projection matrix can be retrieved for device: " << m_inputComponent->getDeviceName();
	displaySettings->setLeftEyeProjectionMatrix(projectionMatrix.block<4,4>(0, 0));

	SURGSIM_ASSERT(
		dataGroup.matrices().get(SurgSim::DataStructures::Names::RIGHT_PROJECTION_MATRIX, &projectionMatrix)) <<
		"No right projection matrix can be retrieved for device: " << m_inputComponent->getDeviceName();
	displaySettings->setRightEyeProjectionMatrix(projectionMatrix.block<4,4>(0, 0));

	return displaySettings;
}

}; // namespace Device
}; // namespace SurgSim