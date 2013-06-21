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

#include "SurgSim/Testing/VisualTestCommon/MovingSquareGlutWindow.h"

using SurgSim::DataStructures::DataGroup;

using SurgSim::Math::Vector3d;
using SurgSim::Math::RigidTransform3d;

MovingSquareGlutWindow::MovingSquareGlutWindow(const std::string& toolDeviceName, const std::string& squareDeviceName) :
	m_toolDeviceName(toolDeviceName),
	m_squareDeviceName(squareDeviceName)
{
	m_camera = std::make_shared<GlutCamera>(Vector3d(-0.15, 0.15, 0.3), Vector3d(0.0, 0.0, 0.0),
											Vector3d(0.0, 1.0, 0.0), 45.0, 0.001, 1.0);
	GlutRenderer::setCamera(m_camera);

	m_square = std::make_shared<GlutSquare>(0.050, Vector3d(1.0, 1.0, 1.0), Vector3d(1.0, 0.0, 0.0),
											Vector3d(0.0, 0.0, 1.0));
	GlutRenderer::addObject(m_square);

	m_toolSphere = std::make_shared<GlutSphere>(0.010, Vector3d(1.0, 1.0, 1.0));
	std::shared_ptr<GlutAxes> toolAxes = std::make_shared<GlutAxes>(0.025, 5.0);
	m_tool = std::make_shared<GlutGroup>();
	m_tool->children.push_back(m_toolSphere);
	m_tool->children.push_back(toolAxes);
	GlutRenderer::addObject(m_tool);

	m_renderThread = boost::thread(boost::ref(GlutRenderer::run));
}

MovingSquareGlutWindow::~MovingSquareGlutWindow()
{
	if (m_renderThread.joinable())
	{
		m_renderThread.join();
	}
}

void MovingSquareGlutWindow::initializeInput(const std::string& device, const DataGroup& inputData)
{
}

void MovingSquareGlutWindow::handleInput(const std::string& device, const DataGroup& inputData)
{
	if (device == m_toolDeviceName)
	{
		updateTool(inputData);
	}
	else if (device == m_squareDeviceName)
	{
		updateSquare(inputData);
	}
	else
	{
		SURGSIM_FAILURE() << "Unknown device name '" << device << "'";
	}
}

void MovingSquareGlutWindow::updateTool(const DataGroup& inputData)
{
	RigidTransform3d devicePose;
	bool button1, button2, button3, button4;
	if (! inputData.poses().get("pose", &devicePose))
	{
		return;  // not much we can do without a pose...
	}
	if (! inputData.booleans().get("button1", &button1))
	{
		button1 = false;
	}
	if (! inputData.booleans().get("button2", &button2))
	{
		button2 = false;
	}
	if (! inputData.booleans().get("button3", &button3))
	{
		button3 = false;
	}
	if (! inputData.booleans().get("button4", &button4))
	{
		button4 = false;
	}

	m_tool->pose = devicePose;

	Vector3d color(1.0, 1.0, 1.0);
	if (button1)
	{
		color.x() = 0.4;
	}
	if (button2)
	{
		color.y() = 0.4;
	}
	if (button3)
	{
		color.z() = 0.4;
	}
	if (button4)
	{
		color = (color + Vector3d(0.7, 0.7, 0.7)) * 0.5;
	}
	m_toolSphere->color = color;
}

void MovingSquareGlutWindow::updateSquare(const DataGroup& inputData)
{
	RigidTransform3d devicePose;
	if (! inputData.poses().get("pose", &devicePose))
	{
		return;
	}

	m_square->pose = devicePose;
}
