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

#ifndef MOVING_SQUARE_GLUT_WINDOW_H
#define MOVING_SQUARE_GLUT_WINDOW_H

#include <boost/thread.hpp>

#include <SurgSim/Input/InputDeviceListenerInterface.h>
#include <SurgSim/Input/DataGroup.h>

#include "GlutRenderer.h"

class MovingSquareGlutWindow : public SurgSim::Input::InputDeviceListenerInterface
{
public:
	/// Constructor.
	MovingSquareGlutWindow(const std::string& toolDeviceName, const std::string& squareDeviceName);
	/// Destructor.
	~MovingSquareGlutWindow();

	virtual void handleInput(const std::string& device, const SurgSim::Input::DataGroup& inputData);

	virtual bool requestOutput(const std::string& device, SurgSim::Input::DataGroup* outputData);

protected:

private:
	boost::thread m_renderThread;

	std::string m_toolDeviceName;
	std::string m_squareDeviceName;

	std::shared_ptr<GlutCamera> m_camera;

	std::shared_ptr<GlutSquare> m_square;
	std::shared_ptr<GlutTool> m_tool;

	void updateTool(const SurgSim::Input::DataGroup& inputData);
	void updateSquare(const SurgSim::Input::DataGroup& inputData);

};

#endif // MOVING_SQUARE_GLUT_WINDOW_H
