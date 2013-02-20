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


/// A simple listener to calculate collision force against a square area for the example application.
/// \sa SurgSim::Input::InputDeviceListenerInterface
class SimpleSquareForce : public SurgSim::Input::InputDeviceListenerInterface
{
public:
	/// Constructor.
	SimpleSquareForce();

	virtual void handleInput(const std::string& device, const SurgSim::Input::DataGroup& inputData);

	virtual bool requestOutput(const std::string& device, SurgSim::Input::DataGroup* outputData);

protected:
	/// Calculates the force as a function of device tip position.
	/// The calculation is very simple, for a simple demo of the device input/output functionality.
	///
	/// \param position The device tip position.
	/// \return The computed force.
	SurgSim::Math::Vector3d computeForce(const SurgSim::Math::Vector3d& position);

private:
	/// Internally stored output data (force and torque).
	SurgSim::Input::DataGroup m_outputData;

	/// One half of the edge length of the square we're colliding against, in meters.
	double m_squareHalfSize;
	/// The surface stiffness, in newtons per meter.
	double m_surfaceStiffness;
	/// The maximum force before the application allows the tool to pop through.
	double m_forceLimit;

	/// The unit normal vector of the square.
	SurgSim::Math::Vector3d m_squareNormal;
	/// The location of the center of the square in world coordinates.
	SurgSim::Math::Vector3d m_squareCenter;
	/// The unit direction along one of the pairs edges of the square.
	SurgSim::Math::Vector3d m_planeDirectionX;
	/// The unit direction along the other pair of edges of the square.
	SurgSim::Math::Vector3d m_planeDirectionY;

	/// The location of the "tip" (i.e. interacting point) of the tool, in the local frame relative to the tool pose.
	SurgSim::Math::Vector3d m_tipPoint;
};

#endif // SIMPLE_SQUARE_FORCE_H
