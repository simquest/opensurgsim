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

#include "SurgSim/Devices/Oculus/OculusDisplaySettings.h"
#include "SurgSim/Framework/Component.h"
#include "SurgSim/Framework/FrameworkConvert.h"

namespace SurgSim
{
namespace Device
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Device::OculusView, OculusView);

OculusView::OculusView(const std::string& name) : OsgView(name), m_deviceName()
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(OculusView, std::string, DeviceName, getDeviceName, setDeviceName);
}

OculusView::~OculusView()
{
}

bool OculusView::doWakeUp()
{
	SURGSIM_ASSERT(!m_deviceName.empty()) <<
		"This view must be used with an Oculus device. But no name of Oculus device is set.";

	OsgView::doWakeUp();
	osg::ref_ptr<SurgSim::Device::OculusDisplaySettings> displaySeetings =
		new SurgSim::Device::OculusDisplaySettings(getOsgView()->getDisplaySettings());
	displaySeetings->retrieveDeviceProjectionMatrix(m_deviceName);

	getOsgView()->setDisplaySettings(displaySeetings);

	return true;
}

void OculusView::setDeviceName(const std::string& deviceName)
{
	m_deviceName = deviceName;
}

std::string OculusView::getDeviceName() const
{
	return m_deviceName;
}

}; // namespace Device
}; // namespace SurgSim