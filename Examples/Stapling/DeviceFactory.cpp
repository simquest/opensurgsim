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

#include "Examples/Stapling/DeviceFactory.h"

#include "SurgSim/Devices/DeviceFilters/PoseTransform.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Devices/MultiAxis/MultiAxisDevice.h"
#include "SurgSim/Framework/Log.h"


#ifdef PHANTOM_LIBRARY_AVAILABLE
#include "SurgSim/Devices/Phantom/PhantomDevice.h"
#endif // PHANTOM_LIBRARY_AVAILABLE

#ifdef SIXENSE_LIBRARY_AVAILABLE
#include "SurgSim/Devices/Sixense/SixenseDevice.h"
#endif // SIXENSE_LIBRARY_AVAILABLE

#ifdef NOVINT_LIBRARY_AVAILABLE
#include "SurgSim/Devices/Novint/Novint7DofDevice.h"
#endif // NOVINT_LIBRARY_AVAILABLE

using SurgSim::Framework::Logger;


DeviceFactory::DeviceFactory()
{
}
DeviceFactory::~DeviceFactory()
{
}

std::shared_ptr<SurgSim::Input::DeviceInterface> DeviceFactory::getDevice(const std::string& name)
{
	auto logger = Logger::getLogger("Stapling/DeviceFactory");
	static std::shared_ptr<SurgSim::Input::DeviceInterface> device;

#ifdef NOVINT_LIBRARY_AVAILABLE
	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a Novint7DofDevice, the first available Falcon.";
	device = std::make_shared<SurgSim::Device::Novint7DofDevice>(name, "");
	//novintDevice->setPositionScale(novintDevice->getPositionScale() * 10.0);
	if (device->initialize())
	{
		return device;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the NovintDevice.";
#endif // NOVINT_LIBRARY_AVAILABLE

#ifdef PHANTOM_LIBRARY_AVAILABLE
	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a PhantomDevice.";
	device = std::make_shared<SurgSim::Device::PhantomDevice>(name + " raw", "Default PHANToM");
	if (device->initialize())
	{
		auto transform = std::make_shared<SurgSim::Device::PoseTransform>(name);
		transform->setTransform(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
			SurgSim::Math::Vector3d(0.0, 0.05, 0.05)));
		device->addInputConsumer(transform);
		device->setOutputProducer(transform);
		return transform;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the PhantomDevice.";
#endif // PHANTOM_LIBRARY_AVAILABLE

#ifdef SIXENSE_LIBRARY_AVAILABLE
	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a SixenseDevice.";
	device = std::make_shared<SurgSim::Device::SixenseDevice>(name + " raw");
	if (device->initialize())
	{
		auto transform = std::make_shared<SurgSim::Device::PoseTransform>(name);
		transform->setTransform(SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
			SurgSim::Math::Vector3d(0.0, -0.1, -0.1)));
		device->addInputConsumer(transform);
		device->setOutputProducer(transform);
		return transform;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the SixenseDevice.";
#endif // SIXENSE_LIBRARY_AVAILABLE

	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to try using a MultiAxisDevice.";
	device = std::make_shared<SurgSim::Device::MultiAxisDevice>(name);
	//multiAxisDevice->setPositionScale(multiAxisDevice->getPositionScale() * 10.0);
	//multiAxisDevice->setOrientationScale(multiAxisDevice->getOrientationScale() * 3.0);
	if (device->initialize())
	{
		return device;
	}
	SURGSIM_LOG_WARNING(logger) << "Could not initialize the MultiAxisDevice.";

	SURGSIM_LOG_INFO(logger) << "DeviceFactory is going to use an IdentityPoseDevice.";
	return std::make_shared<SurgSim::Device::IdentityPoseDevice>(name);
}
