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

#ifndef SIMPLE_SQUARE_FORCE_H
#define SIMPLE_SQUARE_FORCE_H

#include <SurgSim/Input/InputDeviceListenerInterface.h>
#include <SurgSim/Input/DataGroup.h>


class SimpleSquareForce : public SurgSim::Input::InputDeviceListenerInterface
{
public:
	SimpleSquareForce();

	virtual void handleInput(const std::string& device, const SurgSim::Input::DataGroup& inputData);

	virtual bool requestOutput(const std::string& device, SurgSim::Input::DataGroup* outputData);

protected:
	SurgSim::Math::Vector3d computeForce(const SurgSim::Math::Vector3d& position);

private:
	SurgSim::Input::DataGroup m_outputData;

	double m_squareHalfSize;
	double m_surfaceStiffness;
	double m_forceLimit;

	SurgSim::Math::Vector3d m_squareNormal;
	SurgSim::Math::Vector3d m_squareCenter;
	SurgSim::Math::Vector3d m_planeDirectionX;
	SurgSim::Math::Vector3d m_planeDirectionY;

	SurgSim::Math::Vector3d m_tipPoint;

};

#endif // SIMPLE_SQUARE_FORCE_H
