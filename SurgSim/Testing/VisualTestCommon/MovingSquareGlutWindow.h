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

#ifndef SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREGLUTWINDOW_H
#define SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREGLUTWINDOW_H

#include <boost/thread.hpp>

#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Input/OutputProducerInterface.h"
#include "SurgSim/DataStructures/DataGroup.h"

#include "SurgSim/Testing/VisualTestCommon/GlutRenderer.h"

/// A simple listener to display the simple scene composed of a square and tool for the example application.
/// Includes support for the square being moved by a second tool.
/// \sa SurgSim::Input::InputConsumerInterface
class MovingSquareGlutWindow : public SurgSim::Input::InputConsumerInterface
{
public:
	/// Constructor.
	MovingSquareGlutWindow(const std::string& toolDeviceName, const std::string& squareDeviceName);
	/// Destructor.
	~MovingSquareGlutWindow();

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

protected:

private:
	/// Render thread which runs the Glut main loop.
	boost::thread m_renderThread;

	/// Name of the tool device.
	const std::string m_toolDeviceName;
	/// Name of the square device.
	const std::string m_squareDeviceName;

	/// Camera which controls the view of the scene.
	std::shared_ptr<GlutCamera> m_camera;

	/// Tool composed of a sphere and axes that are moved with device input.
	std::shared_ptr<GlutGroup> m_tool;
	/// Sphere of the tool. Pointer is kept here so that the color can easily be changed based on the device's button
	/// state.
	std::shared_ptr<GlutSphere> m_toolSphere;

	/// Square that is moved with device input.
	std::shared_ptr<GlutSquare> m_square;

	/// Updates the tool based on the device input.
	/// \param inputData Input data from the device.
	void updateTool(const SurgSim::DataStructures::DataGroup& inputData);
	/// Updates the square based on the device input.
	/// \param inputData Input data from the device.
	void updateSquare(const SurgSim::DataStructures::DataGroup& inputData);

};

#endif // SURGSIM_TESTING_VISUALTESTCOMMON_MOVINGSQUAREGLUTWINDOW_H
