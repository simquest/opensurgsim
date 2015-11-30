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

#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"

#include "SurgSim/Devices/MultiAxis/RawMultiAxisDevice.h"
#include "SurgSim/Devices/DeviceFilters/PoseIntegrator.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Assert.h"


namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::MultiAxisDevice, MultiAxisDevice);

MultiAxisDevice::MultiAxisDevice(const std::string& uniqueName) :
	FilteredDevice(uniqueName),
	m_rawDevice(std::make_shared<RawMultiAxisDevice>(uniqueName + "_RawBase")),
	m_filter(std::make_shared<PoseIntegrator>(uniqueName + "_Integrator"))
{
	m_rawDevice->setPositionScale(defaultPositionScale());
	m_rawDevice->setOrientationScale(defaultOrientationScale());
	m_rawDevice->setAxisDominance(true);
	setDevice(m_rawDevice);

	m_filter->setNameForCallback(uniqueName);  // the filter should make callbacks as the entire device
	addFilter(m_filter);

	SURGSIM_ADD_SERIALIZABLE_PROPERTY(MultiAxisDevice, double, PositionScale,
									  getPositionScale, setPositionScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(MultiAxisDevice, double, OrientationScale,
									  getOrientationScale, setOrientationScale);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(MultiAxisDevice, bool, AxisDominance,
									  isUsingAxisDominance, setAxisDominance);
}

void MultiAxisDevice::setPositionScale(double scale)
{
	m_rawDevice->setPositionScale(scale);
}

double MultiAxisDevice::getPositionScale() const
{
	return m_rawDevice->getPositionScale();
}

void MultiAxisDevice::setOrientationScale(double scale)
{
	m_rawDevice->setOrientationScale(scale);
}

double MultiAxisDevice::getOrientationScale() const
{
	return m_rawDevice->getOrientationScale();
}

void MultiAxisDevice::setAxisDominance(bool onOff)
{
	m_rawDevice->setAxisDominance(onOff);
}

bool MultiAxisDevice::isUsingAxisDominance() const
{
	return m_rawDevice->isUsingAxisDominance();
}

double MultiAxisDevice::defaultPositionScale()
{
	return 0.00001; // The default position scale, in meters per tick.
}

double MultiAxisDevice::defaultOrientationScale()
{
	return 0.0001; // The default rotation scale, in radians per tick.
}

void MultiAxisDevice::setReset(const std::string& name)
{
	m_filter->setReset(name);
}

};  // namespace Devices
};  // namespace SurgSim
