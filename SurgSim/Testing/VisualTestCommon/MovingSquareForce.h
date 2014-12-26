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

#ifndef SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREFORCE_H
#define SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREFORCE_H

#include <string>

#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/DataStructures/DataGroup.h"

#include "SurgSim/Framework/LockedContainer.h"


/// A simple listener to calculate collision force against a square area for the example application.
/// Includes support for the square being moved by a second tool.
/// \sa SurgSim::Input::InputConsumerInterface, SurgSim::Input::OutputProducerInterface
class MovingSquareForce : public SurgSim::Input::InputConsumerInterface, public SurgSim::Input::OutputProducerInterface
{
public:
	/// Constructor.
	MovingSquareForce(const std::string& toolDeviceName, const std::string& squareDeviceName);

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	bool requestOutput(const std::string& device, SurgSim::DataStructures::DataGroup* outputData) override;

protected:
	/// Updates the state of the tool as described by toolInputData.
	/// \param toolInputData The state of the device controlling the tool.
	void updateTool(const SurgSim::DataStructures::DataGroup& toolInputData);

	/// Updates the state of the square as described by squareInputData.
	/// \param squareInputData The state of the device controlling the colliding square.
	void updateSquare(const SurgSim::DataStructures::DataGroup& squareInputData);

	/// Calculates the force as a function of device tip position.
	/// The calculation is very simple, for a simple demo of the device input/output functionality.
	///
	/// \param position The device tip position.
	/// \return The computed force.
	SurgSim::Math::Vector3d computeForce(const SurgSim::Math::Vector3d& position);

private:
	/// State defined by the pose of the square.
	struct SquarePoseVectors
	{
		/// Constructor.
		SquarePoseVectors();

		/// The unit normal vector of the square.
		SurgSim::Math::Vector3d normal;
		/// The unit direction along one of the pairs edges of the square.
		SurgSim::Math::Vector3d edgeDirectionX;
		/// The unit direction along the other pair of edges of the square.
		SurgSim::Math::Vector3d edgeDirectionY;
		/// The location of the center of the square in world coordinates.
		SurgSim::Math::Vector3d center;
	};


	/// Name of the device used as the tool.
	const std::string m_toolDeviceName;
	/// Name of the device used to move the square.
	const std::string m_squareDeviceName;

	/// Internally stored output data (force and torque).
	SurgSim::DataStructures::DataGroup m_outputData;

	/// One half of the edge length of the square we're colliding against, in meters.
	double m_squareHalfSize;
	/// The surface stiffness, in newtons per meter.
	double m_surfaceStiffness;
	/// The maximum force before the application allows the tool to pop through.
	double m_forceLimit;

	/// Points and directions defined by the pose of the square.
	SurgSim::Framework::LockedContainer<SquarePoseVectors> m_square;
	/// The current sign of the direction of the normal vector of the square.
	double m_squareNormalDirection;

	/// The location of the "tip" (i.e. interacting point) of the tool, in the local frame relative to the tool pose.
	SurgSim::Math::Vector3d m_tipPoint;
};

#endif // SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREFORCE_H
