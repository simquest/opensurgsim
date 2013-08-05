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

#ifndef SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H
#define SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>
#include <SurgSim/Input/InputConsumerInterface.h>
#include <SurgSim/Input/OutputProducerInterface.h>
#include <SurgSim/Math/RigidTransform.h>

namespace SurgSim
{
namespace Device
{


class PoseIntegrator : public SurgSim::Input::CommonDevice,
	public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	typedef SurgSim::Math::RigidTransform3d PoseType;

	PoseIntegrator(const std::string& filterName, const SurgSim::DataStructures::DataGroup& inputFilterData,
				   const std::string& callbackDeviceName);

	const PoseType& integrate(const PoseType& pose);

	virtual std::string getName() const override;

	virtual bool initialize() override;

	virtual bool finalize() override;

	bool isInitialized() const;

	virtual void initializeInput(const std::string& device,
		const SurgSim::DataStructures::DataGroup& inputData) override;

	virtual void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	virtual bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

private:
	std::string m_name;
	PoseType m_poseResult;
};


};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_MULTIAXIS_POSEINTEGRATOR_H
