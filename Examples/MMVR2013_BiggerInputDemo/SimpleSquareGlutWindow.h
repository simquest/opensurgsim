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

#ifndef SIMPLE_SQUARE_GLUT_WINDOW_H
#define SIMPLE_SQUARE_GLUT_WINDOW_H

#include <boost/thread.hpp>

#include <SurgSim/Input/InputDeviceListenerInterface.h>
#include <SurgSim/Input/DataGroup.h>


/// A simple listener to calculate collision force against a square area for the example application.
/// \sa SurgSim::Input::InputDeviceListenerInterface
class SimpleSquareGlutWindow : public SurgSim::Input::InputDeviceListenerInterface
{
public:
	/// Constructor.
	SimpleSquareGlutWindow();
	/// Destructor.
	~SimpleSquareGlutWindow();

	virtual void handleInput(const std::string& device, const SurgSim::Input::DataGroup& inputData);

	virtual bool requestOutput(const std::string& device, SurgSim::Input::DataGroup* outputData);

protected:

private:
	boost::thread m_renderThread;

};



#endif // SIMPLE_SQUARE_GLUT_WINDOW_H
