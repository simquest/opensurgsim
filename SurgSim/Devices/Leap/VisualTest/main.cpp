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

#include <boost/thread.hpp>
#include <memory>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Devices/Leap/LeapDevice.h"
#include "SurgSim/Input/DeviceInterface.h"
#include "SurgSim/Input/InputConsumerInterface.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/VisualTestCommon/GlutRenderer.h"

using SurgSim::Input::DeviceInterface;
using SurgSim::Device::LeapDevice;
using SurgSim::Math::Vector3d;


class GlutWindow : public SurgSim::Input::InputConsumerInterface
{
public:
	GlutWindow()
	{
		m_camera = std::make_shared<GlutCamera>(Vector3d(0.0, -1.0, 0.0), Vector3d(0.0, 0.0, 0.0),
											Vector3d(0.0, 0.0, -1.0), 45.0, 0.001, 2.0);
		GlutRenderer::setCamera(m_camera);
		m_renderThread = boost::thread(boost::ref(GlutRenderer::run));
	}

	~GlutWindow()
	{
		if (m_renderThread.joinable())
		{
			m_renderThread.join();
		}
	}

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		m_numHands++;
		for (int i = 0; i < inputData.poses().getNumEntries(); i++)
		{
			auto sphere = std::make_shared<GlutSphere>(0.010, Vector3d::Unit(m_numHands % 3));
			GlutRenderer::addObject(sphere);
			m_spheres.insert(std::make_pair(device + inputData.poses().getName(i), sphere));
		}
	}

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override
	{
		SurgSim::Math::RigidTransform3d pose;
		for (int i = 0; i < inputData.poses().getNumEntries(); i++)
		{
			inputData.poses().get(i, &pose);
			m_spheres[device + inputData.poses().getName(i)]->pose = pose;
		}
	}

private:
	boost::thread m_renderThread;
	std::shared_ptr<GlutCamera> m_camera;
	size_t m_numHands;
	std::unordered_map<std::string, std::shared_ptr<GlutSphere>> m_spheres;
};

int main(int argc, char** argv)
{
	auto leftHand = std::make_shared<LeapDevice>("Left Hand");
	leftHand->setHandType(SurgSim::Device::HANDTYPE_LEFT);
	leftHand->initialize();

	auto rightHand = std::make_shared<LeapDevice>("Right Hand");
	rightHand->setHandType(SurgSim::Device::HANDTYPE_RIGHT);
	rightHand->initialize();

	std::shared_ptr<GlutWindow> window = std::make_shared<GlutWindow>();
	leftHand->addInputConsumer(window);
	rightHand->addInputConsumer(window);

	std::cout << std::endl << "**********************************************************************" << std::endl
		<< "Leap Visual Test" << std::endl << std::endl
		<< "When done, press Enter to quit the application." << std::endl
		<< "**********************************************************************" << std::endl;
	getc(stdin);
	std::cout << "Exiting..." << std::endl;

	leftHand->removeInputConsumer(window);
	rightHand->removeInputConsumer(window);

	exit(0);
}
