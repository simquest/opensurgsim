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

#include <osg/DisplaySettings>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Devices
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Devices::OculusView, OculusView);

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
	setTargetScreen(1); // Assume Oculus HMD has ID '1'.

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

	osg::ref_ptr<SurgSim::Devices::OculusDisplaySettings> displaySettings =
		new SurgSim::Devices::OculusDisplaySettings(OsgView::createDisplaySettings());

	SurgSim::DataStructures::DataGroup dataGroup;
	m_inputComponent->getData(&dataGroup);
	SurgSim::DataStructures::DataGroup::DynamicMatrixType leftProjectionMatrix;
	SurgSim::DataStructures::DataGroup::DynamicMatrixType rightProjectionMatrix;

	if (dataGroup.matrices().get(SurgSim::DataStructures::Names::LEFT_PROJECTION_MATRIX, &leftProjectionMatrix) &&
		dataGroup.matrices().get(SurgSim::DataStructures::Names::RIGHT_PROJECTION_MATRIX, &rightProjectionMatrix))
	{
		displaySettings->setLeftEyeProjectionMatrix(leftProjectionMatrix.block<4,4>(0, 0));
		displaySettings->setRightEyeProjectionMatrix(rightProjectionMatrix.block<4,4>(0, 0));
	}
	else
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getLogger("OculusView")) <<
			"No projection matrices for left/right eye.";
	}
	return displaySettings;
}

}; // namespace Devices
}; // namespace SurgSim